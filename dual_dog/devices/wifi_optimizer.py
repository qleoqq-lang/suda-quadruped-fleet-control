"""Local Wi-Fi survey and guarded Xiaomi channel optimization."""

import argparse
import concurrent.futures
import ctypes
from ctypes import wintypes
import getpass
import hashlib
import json
import logging
import os
from pathlib import Path
import re
import secrets
import subprocess
import threading
import time
import urllib.parse
import urllib.request

from dual_dog.devices.network_guard import NetworkLease


from dual_dog.paths import LOCAL_ROOT, REAL_CONFIG

OUTPUT = LOCAL_ROOT / "diagnostics" / "wifi_optimizer"
CANDIDATES = (36, 44, 149, 157)
LOG = logging.getLogger(__name__)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def fresh_scan():
    class Guid(ctypes.Structure):
        _fields_ = [("data", ctypes.c_ubyte * 16)]

    class Interface(ctypes.Structure):
        _fields_ = [("guid", Guid), ("description", wintypes.WCHAR * 256), ("state", wintypes.DWORD)]

    class Notification(ctypes.Structure):
        _fields_ = [("source", wintypes.DWORD), ("code", wintypes.DWORD), ("guid", Guid),
                    ("size", wintypes.DWORD), ("data", ctypes.c_void_p)]

    api = ctypes.WinDLL("wlanapi", use_last_error=True)
    callback_type = ctypes.WINFUNCTYPE(None, ctypes.POINTER(Notification), ctypes.c_void_p)
    api.WlanOpenHandle.argtypes = [wintypes.DWORD, ctypes.c_void_p, ctypes.POINTER(wintypes.DWORD), ctypes.POINTER(ctypes.c_void_p)]
    api.WlanEnumInterfaces.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)]
    api.WlanRegisterNotification.argtypes = [ctypes.c_void_p, wintypes.DWORD, wintypes.BOOL, callback_type, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(wintypes.DWORD)]
    api.WlanScan.argtypes = [ctypes.c_void_p, ctypes.POINTER(Guid), ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    api.WlanFreeMemory.argtypes = [ctypes.c_void_p]
    api.WlanCloseHandle.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    handle, memory, negotiated = ctypes.c_void_p(), ctypes.c_void_p(), wintypes.DWORD()
    finished = threading.Event()
    outcome = []

    def checked(result):
        if result:
            raise OSError(f"Windows WLAN 扫描失败，错误码 {result}")

    checked(api.WlanOpenHandle(2, None, ctypes.byref(negotiated), ctypes.byref(handle)))
    try:
        checked(api.WlanEnumInterfaces(handle, None, ctypes.byref(memory)))
        count = ctypes.cast(memory, ctypes.POINTER(wintypes.DWORD))[0]
        entries = (Interface * count).from_address(memory.value + 8)
        selected = next((entry for entry in entries if entry.state == 1), None)
        if selected is None:
            raise RuntimeError("电脑尚未连接 Wi-Fi，无法进行现场扫描")

        @callback_type
        def callback(pointer, context):
            notice = pointer.contents
            if bytes(notice.guid) == bytes(selected.guid) and notice.source == 8 and notice.code in (7, 8):
                outcome.append(notice.code)
                finished.set()

        previous = wintypes.DWORD()
        checked(api.WlanRegisterNotification(handle, 8, False, callback, None, None, ctypes.byref(previous)))
        checked(api.WlanScan(handle, ctypes.byref(selected.guid), None, None, None))
        if not finished.wait(10) or outcome[-1] != 7:
            raise RuntimeError("未收到成功扫描通知，本轮不推荐或切换信道")
        result = subprocess.run(["netsh", "wlan", "show", "networks", "mode=bssid"],
                                capture_output=True, timeout=10)
        if result.returncode:
            raise RuntimeError("netsh 无法读取扫描结果")
        try:
            output = result.stdout.decode("utf-8")
        except UnicodeDecodeError:
            output = result.stdout.decode("mbcs", errors="replace")
        return parse_scan(output)
    finally:
        api.WlanCloseHandle(handle, None)
        if memory:
            api.WlanFreeMemory(memory)


def parse_scan(text):
    networks, current, ssid = {}, None, ""
    for line in text.splitlines():
        match = re.match(r"\s*SSID\s+\d+\s*:\s*(.*)", line)
        if match:
            ssid = match.group(1).strip()
        match = re.match(r"\s*BSSID\s+\d+\s*:\s*([0-9a-f:]{17})", line, re.I)
        if match:
            current = {"ssid": ssid, "bssid": match.group(1).lower()}
            networks[current["bssid"]] = current
        if current is not None:
            match = re.match(r"\s*(?:Signal|信号)\s*:\s*(\d+)%", line, re.I)
            if match:
                current["signal_percent"] = int(match.group(1))
            match = re.match(r"\s*(?:Channel|信道|频道)\s*:\s*(\d+)\s*$", line, re.I)
            if match:
                current["channel"] = int(match.group(1))
    return [entry for entry in networks.values() if "channel" in entry and "signal_percent" in entry]


def rank_channels(networks, own_ssid, allowed):
    ranking = []
    for channel in CANDIDATES:
        if channel not in allowed:
            continue
        block = 36 if channel < 100 else 149
        neighbors = [entry for entry in networks if entry["ssid"] != own_ssid
                     and block <= entry["channel"] <= block + 12]
        score = sum((entry["signal_percent"] / 10) ** 2 for entry in neighbors)
        ranking.append({"channel": channel, "bandwidth": 40, "score": round(score, 2), "neighbors": len(neighbors)})
    return sorted(ranking, key=lambda row: (row["score"], row["channel"]))


class Router:
    def __init__(self, password, host="192.168.31.1"):
        self.base = "http://" + host
        self.password = password
        self.prefix = None

    def login(self):
        html = urllib.request.urlopen(self.base + "/cgi-bin/luci/web", timeout=6).read().decode("utf-8")
        key = re.search(r"key: '([^']+)'", html).group(1)
        device = re.search(r"deviceId = '([^']+)'", html).group(1)
        info = json.load(urllib.request.urlopen(self.base + "/cgi-bin/luci/api/xqsystem/init_info", timeout=6))
        digest = hashlib.sha256 if str(info.get("newEncryptMode")) == "1" else hashlib.sha1
        nonce = f"0_{device}_{int(time.time())}_{secrets.randbelow(10000)}"
        password = digest((nonce + digest((self.password + key).encode()).hexdigest()).encode()).hexdigest()
        body = urllib.parse.urlencode(dict(username="admin", password=password, logtype=2, nonce=nonce)).encode()
        auth = json.load(urllib.request.urlopen(self.base + "/cgi-bin/luci/api/xqsystem/login", body, timeout=6))
        if auth.get("code") != 0:
            raise RuntimeError("路由器登录失败，请核对管理密码")
        self.prefix = self.base + "/cgi-bin/luci/;stok=" + auth["token"]

    def request(self, endpoint, params=None):
        if self.prefix is None:
            self.login()
        body = None if params is None else urllib.parse.urlencode(params).encode()
        try:
            response = json.load(urllib.request.urlopen(self.prefix + endpoint, body, timeout=8))
        except Exception:
            self.prefix = None
            raise RuntimeError("路由器请求失败，稍后重新读取状态") from None
        if response.get("code") != 0:
            self.prefix = None
            raise RuntimeError(f"路由器返回错误码 {response.get('code')}")
        return response

    def wifi(self):
        return self.request("/api/xqnetwork/wifi_detail_all")

    def set_channel(self, snapshot, channel, bandwidth):
        if len(snapshot["info"]) != 2 or int(snapshot["bsd"]) != 0:
            raise RuntimeError("仅支持当前已核验的双频分离路由器配置")
        params = {"bsd": 0, "isDFS": int(int(bandwidth) == 0 or 52 <= int(channel) <= 64)}
        for index, radio in enumerate(snapshot["info"], 1):
            for target, source in (("on", "status"), ("ssid", "ssid"), ("pwd", "password"),
                                   ("encryption", "encryption"), ("channel", "channel"),
                                   ("bandwidth", "bandwidth"), ("hidden", "hidden"), ("txpwr", "txpwr")):
                params[f"{target}{index}"] = radio[source]
        params.update(channel2=channel, bandwidth2=bandwidth)
        return self.request("/api/xqnetwork/set_all_wifi", params)


def ping_fleet(ips):
    def probe(ip):
        result = subprocess.run(["ping", "-n", "3", "-w", "700", ip], capture_output=True,
                                text=True, errors="replace", timeout=6)
        replies = re.findall(r"(?:TTL=|ttl=)\d+", result.stdout)
        return {"ip": ip, "received": min(3, len(replies)), "sent": 3}
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(ips)) as executor:
        return list(executor.map(probe, ips))


def controller_running():
    command = r"Get-CimInstance Win32_Process | Where-Object { $_.Name -match '^python(w)?\.exe$|控制.*\.exe$' -and ($_.CommandLine -match '-m dual_dog(?:\s|$)' -or $_.Name -match '控制.*\.exe$') } | Select-Object -ExpandProperty ProcessId"
    result = subprocess.run(["powershell", "-NoProfile", "-Command", command], capture_output=True,
                            text=True, timeout=12)
    if result.returncode:
        raise RuntimeError("无法确认控制程序是否运行，禁止切换")
    return bool(result.stdout.strip())


def loss(samples):
    return 1 - sum(row["received"] for row in samples) / sum(row["sent"] for row in samples)


def reception_unsafe(baseline, observations, ips):
    if loss(observations) > max(loss(baseline) + 0.02, 0.03):
        return True
    for ip in ips:
        samples = [row for row in observations if row["ip"] == ip]
        sent = sum(row["sent"] for row in samples)
        received = sum(row["received"] for row in samples)
        if not sent or received == 0 or received / sent < 0.75:
            return True
    return False


def worth_switching(current, ranking):
    if not ranking:
        return False
    best = ranking[0]
    current_score = next((row["score"] for row in ranking if row["channel"] == current), None)
    return (
        current_score is not None
        and current_score >= 30
        and best["channel"] != current
        and best["score"] <= 0.01
        and best["neighbors"] == 0
    )


def change_and_verify(router, snapshot, candidate, ips, require_all_reachable=True):
    baseline = ping_fleet(ips)
    reachable_ips = [row["ip"] for row in baseline if row["received"] > 0]
    if require_all_reachable and len(reachable_ips) != len(ips):
        raise RuntimeError("有机器狗完全不可达，先处理离线设备，本轮不切换")
    if not reachable_ips:
        raise RuntimeError("没有任何机器狗可达，本轮不切换")
    radio = snapshot["info"][1]
    backup = {"time": time.time(), "channel": radio["channel"], "bandwidth": radio["bandwidth"],
              "ssid": radio["ssid"], "baseline": baseline}
    write_json(OUTPUT / "last_change.json", backup)
    LOG.warning("自动切换：%s → %s，频宽40MHz", radio["channel"], candidate)
    try:
        router.set_channel(snapshot, candidate, 40)
    except RuntimeError:
        LOG.warning("保存响应不确定，将读取实际配置，不重复下发")
    deadline = time.monotonic() + 75
    verified = False
    while time.monotonic() < deadline:
        time.sleep(5)
        try:
            actual = router.wifi()["info"][1]
            if int(actual["channel"]) == candidate and int(actual["bandwidth"]) == 40:
                verified = True
                break
        except (RuntimeError, OSError):
            pass
    if not verified:
        raise RuntimeError("无法回读目标信道；网络可能不可达，请检查路由器。原配置已记录")
    observations = []
    for attempt in range(3):
        time.sleep(5)
        observations.extend(ping_fleet(ips))
    write_json(OUTPUT / "after_change.json", {"time": time.time(), "channel": candidate, "samples": observations})
    if reception_unsafe(baseline, observations, reachable_ips):
        LOG.warning("切换后丢包恶化，尝试回退信道 %s", radio["channel"])
        router.set_channel(snapshot, radio["channel"], radio["bandwidth"])
        time.sleep(10)
        actual = router.wifi()["info"][1]
        if str(actual["channel"]) != str(radio["channel"]) or str(actual["bandwidth"]) != str(radio["bandwidth"]):
            raise RuntimeError("回退尚未确认，请人工检查路由器")
        LOG.warning("已确认恢复原信道与频宽")
    else:
        LOG.info("信道回读和网络恢复验证通过，丢包 %.1f%%", loss(observations) * 100)


def main():
    parser = argparse.ArgumentParser(description="现场 Wi-Fi 检测；--auto 启用自动优化")
    parser.add_argument("--auto", action="store_true")
    parser.add_argument("--interval", type=int, default=60)
    arguments = parser.parse_args()
    singleton = NetworkLease("Local\\BPX_Wifi_Optimizer")
    if not singleton.acquire():
        print("现场信道工具已运行，请勿重复启动。")
        return 1
    OUTPUT.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                        handlers=[logging.StreamHandler(), logging.FileHandler(OUTPUT / "optimizer.log", encoding="utf-8")])
    password = os.environ.pop("BPX_ROUTER_PASSWORD", None) or getpass.getpass("小米路由器管理密码（不保存）：")
    router = Router(password)
    router.login()
    stop_file = OUTPUT / "stop.request"
    stop_file.unlink(missing_ok=True)
    configs = json.loads(REAL_CONFIG.read_text(encoding="utf-8-sig"))["robots"][:10]
    ips = [row["ip"] for row in configs]
    candidate, repetitions = None, 0
    last_change = 0
    state_file = OUTPUT / "cooldown.json"
    if state_file.exists():
        last_change = float(json.loads(state_file.read_text(encoding="utf-8"))["time"])
    while True:
        if stop_file.exists():
            stop_file.unlink()
            LOG.info("收到停止请求，工具已停止")
            return 0
        started = time.monotonic()
        try:
            if arguments.auto and controller_running():
                candidate, repetitions = None, 0
                LOG.info("控制界面运行中，暂停主动扫描和切换；退出界面后自动优化")
                time.sleep(max(60, arguments.interval))
                continue
            snapshot = router.wifi()
            radio = snapshot["info"][1]
            networks = fresh_scan()
            if not any(row["ssid"] == radio["ssid"] for row in networks):
                raise RuntimeError("扫描未看到目标路由器，结果不可靠，本轮不切换")
            allowed = {int(item["c"]) for item in radio["available_channels"] if "40" in map(str, item["b"])}
            ranking = rank_channels(networks, radio["ssid"], allowed)
            current = int(radio["channelInfo"]["channel"])
            report = {"time": time.time(), "current_channel": current, "ranking": ranking, "networks": networks,
                      "note": "分数来自周边AP信号强度的保守估计，不代表实际空口占用率；越低越好"}
            write_json(OUTPUT / "latest_scan.json", report)
            LOG.info("当前信道 %s；候选评分 %s", current, ranking)
            if not arguments.auto:
                return 0
            best = ranking[0]["channel"] if ranking else None
            repetitions = repetitions + 1 if best == candidate else 1
            candidate = best
            if not worth_switching(current, ranking):
                LOG.info("未发现明显更优信道，保持当前网络")
            elif repetitions < 3 or time.time() - last_change < 600:
                LOG.info("等待连续3次一致推荐及10分钟冷却结束")
            elif controller_running():
                LOG.info("控制界面运行中，只检测不切换；退出界面后自动优化")
            else:
                lease = NetworkLease()
                if lease.acquire():
                    try:
                        last_change = time.time()
                        write_json(state_file, {"time": last_change})
                        change_and_verify(router, snapshot, candidate, ips)
                        repetitions = 0
                    finally:
                        lease.close()
        except KeyboardInterrupt:
            LOG.info("自动优化已停止")
            return 0
        except Exception as error:
            candidate, repetitions = None, 0
            LOG.error("本轮停止：%s", str(error).split(";stok=")[0])
            if not arguments.auto:
                return 1
        try:
            time.sleep(max(1, max(60, arguments.interval) - (time.monotonic() - started)))
        except KeyboardInterrupt:
            LOG.info("自动优化已停止")
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
