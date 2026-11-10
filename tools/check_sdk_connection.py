"""Read-only SDK connection and telemetry check for the configured first ten dogs."""

import concurrent.futures
import importlib.metadata
import json
import multiprocessing
from pathlib import Path
import time


def check_robot(config):
    import bpx_sdk

    state = bpx_sdk.RequestRobotState()
    state.setRobotIp(config["ip"])
    state.setTcpLocalPort(0)
    state.setRobotStateUploadPort(19000 + config["session_id"])
    state.setRobotStateUploadRate(10)
    started = time.monotonic()
    result = {"robot": config["id"], "ip": config["ip"], "connected": False}
    try:
        state.connect()
        deadline = started + 15.0
        while time.monotonic() < deadline:
            if state.isConnected():
                battery = state.getBatteryLevel()
                rpy = state.getImuRpy()
                if battery is not None and rpy is not None:
                    result.update(
                        connected=True, seconds=round(time.monotonic() - started, 3),
                        battery=battery, rpy=rpy,
                        motion_state=state.getCurrentMotionState(),
                        control_mode=state.getControlMode(),
                        serial_number=state.getRobotSerialNumber(),
                        model=state.getRobotModel(),
                    )
                    break
            time.sleep(0.1)
        if not result["connected"]:
            result["error"] = "15秒内未同时收到连接确认、电量和姿态"
    except Exception as error:
        result["error"] = str(error)
    finally:
        state.disconnect()
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    configs = json.loads((root / "_local/config/robots.json").read_text(encoding="utf-8-sig"))["robots"][:10]
    with concurrent.futures.ProcessPoolExecutor(max_workers=10, mp_context=multiprocessing.get_context("spawn")) as executor:
        results = list(executor.map(check_robot, configs))
    report = {"sdk": importlib.metadata.version("bpx-sdk-open"), "read_only": True, "results": results}
    report_path = root / "diagnostics/sdk109_connection_check.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    for result in results:
        print(result["robot"], "已连接" if result["connected"] else "未连接",
              "battery=" + str(result.get("battery")), result.get("error", ""), flush=True)
    print(report_path, flush=True)


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()
