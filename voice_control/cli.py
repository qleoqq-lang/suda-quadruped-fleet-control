from __future__ import annotations

import argparse
import logging
from pathlib import Path
import re
import sys
import tempfile
import threading
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from dual_dog.config import load_config
from dual_dog.paths import DEMO_CONFIG, LOCAL_ROOT, REAL_CONFIG
from dual_dog.control.controller import FleetController
from dual_dog.devices.drivers import BpxDriver, SimDriver

from .asr import AsrError, load_dotenv, recognize_file, resolve_api_key
from .commands import VoiceIntent, parse_voice_intents
from .recorder import list_devices, record_wav, wait_for_enter
from .robot import execute_intent


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="语音指令控制 BPX 机器狗")
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="机器狗配置文件",
    )
    parser.add_argument(
        "--driver",
        choices=("sim", "bpx"),
        default="sim",
        help="sim=模拟驱动，bpx=真实 BPX SDK",
    )
    parser.add_argument(
        "--targets",
        default="all",
        help="默认目标（逗号分隔，如 dog01,dog02 或 all）",
    )
    parser.add_argument(
        "--once",
        type=Path,
        default=None,
        help="识别一个本地音频文件后退出",
    )
    parser.add_argument(
        "--model",
        default="paraformer-realtime-v2",
        help="DashScope 实时识别模型",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.3,
        help="语音前进/横移基础速度（m/s）",
    )
    parser.add_argument(
        "--yaw-speed",
        type=float,
        default=0.4,
        help="语音原地旋转基础速度（rad/s）",
    )
    parser.add_argument(
        "--silence-seconds",
        type=float,
        default=1.5,
        help="说话结束静音多少秒后自动停止录音",
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=15.0,
        help="单次录音最大秒数",
    )
    parser.add_argument(
        "--allow-stunts",
        action="store_true",
        help="允许执行特技/跑动等高风险指令",
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="执行每条指令前先输入 y 确认",
    )
    parser.add_argument(
        "--device",
        type=int,
        default=None,
        help="麦克风设备索引",
    )
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="列出可用录音设备后退出",
    )
    parser.add_argument(
        "--single",
        action="store_true",
        help="只识别一条语音指令后退出",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="显示详细日志",
    )
    return parser


def resolve_default_targets(raw: str, available_ids: list[str]) -> tuple[str, ...]:
    if not raw or raw.lower() == "all":
        return tuple(available_ids)
    targets: list[str] = []
    for part in raw.split(","):
        part = part.strip().lower()
        if part == "all":
            return tuple(available_ids)
        if part in available_ids:
            targets.append(part)
            continue
        match = re.fullmatch(r"dog(\d{1,2})", part)
        if match:
            candidate = f"dog{int(match.group(1)):02d}"
            if candidate in available_ids:
                targets.append(candidate)
                continue
        raise ValueError(f"未知目标: {part}")
    if not targets:
        raise ValueError("至少需要一个目标")
    return tuple(dict.fromkeys(targets))


def wait_for_online(
    fleet: FleetController,
    targets: tuple[str, ...],
    timeout: float = 20.0,
) -> tuple[str, ...]:
    """Wait for connections, then return the subset that actually came online."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshots = {snapshot.robot_id: snapshot for snapshot in fleet.snapshots()}
        if all(snapshots.get(target) is not None and snapshots[target].connected for target in targets):
            return targets
        time.sleep(0.1)
    return tuple(
        target for target in targets
        if snapshots.get(target) is not None and snapshots[target].connected
    )


def format_intent(intent: VoiceIntent) -> str:
    detail = ", ".join(intent.targets)
    if intent.command.value == "velocity":
        params = ", ".join(f"{key}={value:.2f}" for key, value in intent.params.items())
        detail += f" | {params} | {intent.duration_s:.1f}s"
    elif intent.duration_s is not None:
        detail += f" | {intent.duration_s:.1f}s"
    return f"{intent.label} -> {detail}"


def handle_text(
    fleet: FleetController,
    text: str,
    *,
    available_ids: list[str],
    default_targets: tuple[str, ...],
    speed: float,
    yaw_speed: float,
    allow_stunts: bool,
    confirm: bool,
) -> bool:
    text = (text or "").strip()
    if not text:
        return False
    intents = parse_voice_intents(
        text,
        available_ids,
        default_targets,
        base_speed=speed,
        base_yaw_speed=yaw_speed,
    )
    if not intents:
        print("未识别到可用指令")
        return False
    print(f"指令: {'；'.join(format_intent(intent) for intent in intents)}")
    if confirm and input("确认执行？(y/N): ").strip().lower() != "y":
        print("已取消")
        return False
    try:
        for intent in intents:
            execute_intent(fleet, intent, allow_stunts=allow_stunts)
    except Exception as exc:
        print(f"执行失败: {exc}")
        return False
    print("指令已提交")
    return True


def run_loop(
    fleet: FleetController,
    args: argparse.Namespace,
    api_key: str,
    available_ids: list[str],
    default_targets: tuple[str, ...],
) -> int:
    print("语音控制已启动。按 Enter 开始录音，静音自动结束；也可以直接输入文字指令。输入 q 退出。")
    try:
        while True:
            line = input("> ").strip()
            if line.lower() in {"q", "quit", "exit", "退出"}:
                return 0
            if line:
                handle_text(
                    fleet,
                    line,
                    available_ids=available_ids,
                    default_targets=default_targets,
                    speed=args.speed,
                    yaw_speed=args.yaw_speed,
                    allow_stunts=args.allow_stunts,
                    confirm=args.confirm,
                )
                if args.single:
                    return 0
                continue

            stop_event = threading.Event()
            enter_thread = threading.Thread(target=wait_for_enter, args=(stop_event,), daemon=True)
            enter_thread.start()
            print("录音中...（静音自动结束或按 Enter 提前结束）")
            try:
                with tempfile.TemporaryDirectory(prefix="voice_robot_") as tmp_dir:
                    audio_path = Path(tmp_dir) / "command.wav"
                    duration = record_wav(
                        audio_path,
                        device=args.device,
                        silence_seconds=args.silence_seconds,
                        max_seconds=args.max_seconds,
                        stop_event=stop_event,
                    )
                    if duration < 0.3:
                        print("录音太短，已忽略")
                        continue
                    text = recognize_file(audio_path, model=args.model, api_key=api_key)
            finally:
                stop_event.set()
                enter_thread.join(timeout=1.0)

            print(f"识别: {text}" if text else "未识别到语音")
            handle_text(
                fleet,
                text,
                available_ids=available_ids,
                default_targets=default_targets,
                speed=args.speed,
                yaw_speed=args.yaw_speed,
                allow_stunts=args.allow_stunts,
                confirm=args.confirm,
            )
            if args.single:
                return 0
    except (KeyboardInterrupt, EOFError):
        print("\n退出")
        return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.speed <= 0 or args.yaw_speed <= 0:
        print("--speed 和 --yaw-speed 必须为正数")
        return 2
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    load_dotenv(LOCAL_ROOT)

    if args.list_devices:
        try:
            print(list_devices())
        except Exception as exc:
            print(f"无法列出设备: {exc}")
            return 1
        return 0

    try:
        api_key = resolve_api_key(LOCAL_ROOT)
    except AsrError as exc:
        print(exc)
        return 2

    try:
        config_path = args.config or (REAL_CONFIG if args.driver == "bpx" else DEMO_CONFIG)
        _, robots, safety = load_config(config_path)
        available_ids = [robot.robot_id for robot in robots]
        default_targets = resolve_default_targets(args.targets, available_ids)
    except Exception as exc:
        print(f"配置读取失败: {exc}")
        return 2

    factory = SimDriver if args.driver == "sim" else BpxDriver
    fleet = FleetController(robots, safety, factory)
    try:
        print(f"尝试连接 {len(default_targets)} 台机器狗 ...")
        fleet.connect(default_targets)
        online_targets = wait_for_online(fleet, default_targets)
        if not online_targets:
            raise RuntimeError("没有在线机器狗，请检查网络和驱动配置")
        offline_targets = tuple(
            target for target in default_targets if target not in online_targets
        )
        print(f"在线 {len(online_targets)} 台: {', '.join(online_targets)}")
        if offline_targets:
            print(f"离线跳过: {', '.join(offline_targets)}")
        default_targets = online_targets
        if args.once is not None:
            text = recognize_file(args.once, model=args.model, api_key=api_key)
            print(f"识别: {text}" if text else "未识别到语音")
            handle_text(
                fleet,
                text,
                available_ids=available_ids,
                default_targets=default_targets,
                speed=args.speed,
                yaw_speed=args.yaw_speed,
                allow_stunts=args.allow_stunts,
                confirm=args.confirm,
            )
            return 0
        return run_loop(fleet, args, api_key, available_ids, default_targets)
    except KeyboardInterrupt:
        print("\n中断，正在安全停止...")
        fleet.emergency_stop()
        return 130
    except Exception as exc:
        print(f"运行失败: {exc}")
        return 1
    finally:
        fleet.close()


if __name__ == "__main__":
    sys.exit(main())
