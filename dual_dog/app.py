"""Select a launch profile and start the Qt console."""

import argparse
from importlib.metadata import version
import logging
import multiprocessing as mp
from pathlib import Path
import sys

from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication, QMessageBox

from dual_dog.config import load_config
from dual_dog.devices.network_guard import NetworkLease
from dual_dog.paths import DEMO_CONFIG, REAL_CONFIG, asset_path
from dual_dog.ui.gui import MainWindow, OPENING_CEREMONY_STYLE_SHEET


def check_sdk() -> str:
    import bpx_sdk

    sdk_version = version("bpx-sdk-open")
    if sdk_version != "1.0.9" or not hasattr(bpx_sdk.MotionLevelControl, "isConnected"):
        raise RuntimeError("需要配套的 BPX SDK 1.0.9。请运行 setup.ps1 -Real。")
    return sdk_version


def run(argv: list[str] | None = None) -> int:
    mp.freeze_support()
    parser = argparse.ArgumentParser(description="苏州大学四足机器人群控")
    parser.add_argument("--real", action="store_true", help="使用本地真机配置与 BPX SDK")
    parser.add_argument("--config", type=Path, help="覆盖当前模式的设备配置文件")
    parser.add_argument("--check", action="store_true", help="验证环境与配置后退出")
    args = parser.parse_args(argv)
    config_path = (args.config or (REAL_CONFIG if args.real else DEMO_CONFIG)).resolve()
    expected_mode = "real" if args.real else "sim"
    try:
        mode, configs, _ = load_config(config_path)
        if mode != expected_mode:
            raise ValueError(f"配置文件的 mode 必须为 {expected_mode}。")
        sdk_version = check_sdk() if args.real else None
    except (ImportError, OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        print(f"启动检查失败：{exc}", file=sys.stderr)
        return 1

    if args.check:
        print(f"环境检查通过 | 模式 {mode} | 设备位 {len(configs)}")
        if sdk_version:
            print(f"BPX SDK {sdk_version} 已加载")
        return 0

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stdout,
    )
    app = QApplication([sys.argv[0]])
    lease = NetworkLease() if args.real else None
    if lease is not None and not lease.acquire():
        QMessageBox.information(None, "网络维护中", "正在调整网络，或已有真机控制界面运行。")
        return 1
    try:
        app.setStyle("Fusion")
        app.setStyleSheet(OPENING_CEREMONY_STYLE_SHEET)
        icon = asset_path("suda_app_icon.ico")
        if icon.is_file():
            app.setWindowIcon(QIcon(str(icon)))
        window = MainWindow(config_path)
        window.show()
        return app.exec()
    finally:
        if lease is not None:
            lease.close()
