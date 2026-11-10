from __future__ import annotations

import json
import logging
import math
import os
from pathlib import Path
import time

from PySide6.QtCore import QObject, QTimer, Qt, QUrl, Signal
from PySide6.QtGui import QCloseEvent, QColor, QIcon, QKeyEvent
from PySide6.QtMultimedia import QAudioOutput, QMediaPlayer
from PySide6.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from dual_dog.ui.suda_opening_theme import SudaOpeningHeader
from dual_dog.ui.network_panel import NetworkPanel
from dual_dog.config import load_application_name, load_config
from dual_dog.control.controller import (
    FleetController,
    effective_flip_lock_seconds,
    effective_flip_trigger_seconds,
)
from dual_dog.control.choreography import (
    DancePhase, DanceStep, PhaseKind, TRICK_POST_ACTION_CORRECTION_SECONDS,
    build_multitrack_dance,
    dance_steps_from_payload, dance_steps_to_payload,
    keep_connected_dance_phases, overlapping_dance_cue_messages,
)
from dual_dog.devices.drivers import BpxDriver, SimDriver
from dual_dog.paths import DEMO_CONFIG, LOCAL_ROOT, asset_path
from dual_dog.ui.dance_editor import ChoreographyEditorDialog
from dual_dog.diagnostics import StraightLineCsvRecorder
from dual_dog.ui.odometry_window import OdometryCalibrationWindow
from dual_dog.ui.position_window import PositionWindow
from dual_dog.devices.positioning import control_stale_seconds, inside_stage, unit_to_meters
from dual_dog.control.feedback import (
    common_yaw_from_raw,
    raw_target_for_common_yaw,
    yaw_calibration_offset,
)
from dual_dog.models import Command, RobotConfig, Telemetry
from dual_dog.control.speed_sync import FormationProgressSynchronizer
from voice_control.commands import STUNT_COMMANDS as VOICE_STUNT_COMMANDS
from voice_control.qt import VoiceExecutor, VoiceRecognitionWorker

LOG = logging.getLogger(__name__)
CONNECT_STAGGER_SECONDS = 0.6

ODOMETRY_SCALE_SPECS = (
    ("forward", "前进"),
    ("backward", "后退"),
    ("left", "左移"),
    ("right", "右移"),
    ("turn_left", "左转"),
    ("turn_right", "右转"),
)
OPENING_CEREMONY_STYLE_SHEET = """
QMainWindow, QWidget#contentRoot {
    background: #f6f0e6;
    color: #3b2929;
    font-family: "Microsoft YaHei UI";
    font-size: 14px;
}
QWidget#headerBar {
    background: transparent;
    border: 1px solid #dcc896;
    border-radius: 18px;
}
QWidget#headerBar QCheckBox { color: #fff8e8; }
QWidget#headerBar QLabel { color: #fff8e8; }
QLabel#applicationTitle {
    color: #fff8e8;
    font-size: 25px;
    font-weight: 650;
}
QLabel#applicationSubtitle {
    color: #f0d79b;
    font-size: 21px;
    font-weight: 600;
}
QLabel#citySignature {
    color: #e2c17d;
    font-size: 12px;
    letter-spacing: 3px;
    min-height: 28px;
}
QGroupBox {
    background: #fffdf9;
    border: 1px solid #e7d9c5;
    border-radius: 16px;
    margin-top: 14px;
    padding: 16px 12px 12px 12px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 16px;
    padding: 0 6px;
    color: #7a0000;
}
QPushButton {
    min-height: 34px;
    padding: 2px 16px;
    color: #523a34;
    background: #f7f1e8;
    border: 1px solid #e4d6c2;
    border-radius: 11px;
    font-weight: 500;
}
QPushButton:hover {
    color: #7a0000;
    background: #f1e4d4;
    border-color: #d9bd8d;
}
QPushButton:pressed {
    background: #ead8c2;
}
QPushButton:disabled {
    color: #a99f98;
    background: #f7f4ef;
    border-color: #eee7dc;
}
QPushButton[role="primary"] {
    color: #fff8e8;
    background: #8c0000;
    border-color: #8c0000;
    font-weight: 600;
}
QPushButton[role="primary"]:hover { background: #760000; border-color: #760000; }
QPushButton[role="primary"]:pressed { background: #620000; border-color: #620000; }
QPushButton[role="motion"] {
    color: #8c0000;
    background: #fff6e9;
    border-color: #e7cfa5;
    font-weight: 600;
}
QPushButton[role="motion"]:hover {
    color: #6f0000;
    background: #f9e8cf;
    border-color: #d8b56a;
}
QPushButton[role="motion"]:pressed { background: #f0d8b4; }
QPushButton[role="stop"] {
    color: #a72d34;
    background: #fff1f2;
    border-color: #efcdd1;
    font-weight: 600;
}
QPushButton[role="stop"]:hover {
    color: #8f2028;
    background: #fbe3e5;
    border-color: #e5bfc4;
}
QPushButton[role="stop"]:pressed { background: #f4d5d8; }
QPushButton#emergencyButton {
    min-height: 100px;
    max-height: 100px;
    color: #ffffff;
    background: #bd333c;
    border: 1px solid #bd333c;
    border-radius: 14px;
    font-size: 19px;
    font-weight: 700;
}
QPushButton#emergencyButton:hover { background: #aa2932; border-color: #aa2932; }
QPushButton#emergencyButton:pressed { background: #8f2028; border-color: #8f2028; }
QComboBox, QSpinBox, QDoubleSpinBox {
    min-height: 32px;
    padding: 0 10px;
    background: #fffdf9;
    border: 1px solid #dfcfb8;
    border-radius: 9px;
    selection-background-color: #8c0000;
}
QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {
    border: 2px solid #b50010;
}
QComboBox::drop-down {
    width: 28px;
    border: none;
}
QCheckBox { spacing: 8px; color: #604a43; }
QTextEdit {
    color: #4c3a36;
    background: #fffdf9;
    border: 1px solid #e5d8c5;
    border-radius: 14px;
    padding: 10px;
    selection-background-color: #8c0000;
}
QStatusBar {
    color: #79675f;
    background: #f6f0e6;
    border-top: 1px solid #e3d3bb;
}
QScrollArea {
    background: #f6f0e6;
    border: none;
}
QTabWidget::pane {
    background: transparent;
    border: none;
    top: 8px;
}
QTabBar::tab {
    min-width: 130px;
    min-height: 34px;
    margin-right: 6px;
    padding: 2px 18px;
    color: #735f57;
    background: #eee1d1;
    border: none;
    border-radius: 10px;
    font-weight: 600;
}
QTabBar::tab:hover {
    color: #7a0000;
    background: #e8d5bf;
}
QTabBar::tab:selected {
    color: #fff8e8;
    background: #8c0000;
}
QTableWidget {
    color: #3f3330;
    background: #fffdf9;
    alternate-background-color: #fcf5ed;
    border: 1px solid #eadfce;
    border-radius: 10px;
    gridline-color: #eee7dc;
    selection-background-color: #efd8b4;
    selection-color: #351e1c;
}
QHeaderView::section {
    min-height: 30px;
    padding: 4px 8px;
    color: #fff8e8;
    background: #7a0000;
    border: none;
    border-bottom: 1px solid #e5d8c5;
    font-weight: 600;
}
QToolTip {
    color: #fff8e8;
    background: #4a2426;
    border: 1px solid #b99154;
    padding: 6px;
}
"""


class LogEmitter(QObject):
    message = Signal(str)


class LogHandler(logging.Handler):
    def __init__(self, sink: QTextEdit) -> None:
        super().__init__()
        self.emitter = LogEmitter(sink)
        self.emitter.message.connect(sink.append, Qt.QueuedConnection)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self.emitter.message.emit(self.format(record))
        except (RuntimeError, TypeError):
            # The window can disappear while a worker is finishing shutdown.
            pass


class RobotCard(QGroupBox):
    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config.name)
        self.config = config
        self.connection = QLabel("● 离线")
        self.connection.setStyleSheet("color: #888")
        self.details = QLabel()
        self.details.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(f"IP: {config.ip}    UDP: {config.state_port}"))
        layout.addWidget(self.connection)
        layout.addWidget(self.details)
        self.update_data(Telemetry(config.robot_id))

    def update_data(self, data: Telemetry) -> None:
        if data.connected:
            self.connection.setText("● 已连接")
            self.connection.setStyleSheet("color: #1a9b45; font-weight: bold")
        else:
            self.connection.setText("● 离线")
            self.connection.setStyleSheet("color: #888")
        battery = "--" if data.battery is None else f"{data.battery}%"
        temperature = "--" if data.max_temperature is None else f"{data.max_temperature:.1f} °C"
        if data.rpy is None:
            attitude = "--"
        else:
            attitude = "/".join(f"{value:.2f}" for value in data.rpy)
        error = (chr(10) + "错误：" + str(data.error)) if data.error else ""
        position = (
            "--" if data.odom_position is None
            else "/".join(f"{value:.2f}" for value in data.odom_position)
        )
        self.details.setText(
            f"动作：{data.action}\n电量：{battery}    最高温度：{temperature}\n"
            f"姿态 R/P/Y：{attitude}\n里程计 X/Y/Z：{position}\n"
            f"固件状态：motion={data.motion_state} gait={data.gait} sub={data.sub_gait}\n"
            f"反馈：{data.feedback}\n保护：{data.protection}{error}"
        )


class FleetStatusTable(QTableWidget):
    COLUMNS = ("设备", "IP", "连接", "电量", "Yaw", "动作", "反馈", "保护 / 错误")

    def __init__(self, configs: list[RobotConfig]) -> None:
        super().__init__(len(configs), len(self.COLUMNS))
        self.rows = {config.robot_id: row for row, config in enumerate(configs)}
        self.setHorizontalHeaderLabels(self.COLUMNS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        self.setAlternatingRowColors(True)
        self.setShowGrid(False)
        self.setWordWrap(False)
        header = self.horizontalHeader()
        for column in range(len(self.COLUMNS)):
            mode = QHeaderView.Stretch if column in {5, 6, 7} else QHeaderView.ResizeToContents
            header.setSectionResizeMode(column, mode)
        self.setFixedHeight(150)
        self.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        for config in configs:
            row = self.rows[config.robot_id]
            initial = (config.name, config.ip, "离线", "--", "--", "离线", "待命", "正常")
            for column, value in enumerate(initial):
                self.setItem(row, column, QTableWidgetItem(value))

    def update_data(self, data: Telemetry) -> None:
        row = self.rows[data.robot_id]
        connection = "已连接" if data.connected else "离线"
        battery = "--" if data.battery is None else f"{data.battery}%"
        yaw = "--" if data.rpy is None else f"{math.degrees(data.rpy[2]):+.1f}°"
        protection = data.protection
        if data.error:
            protection = ("错误：" + str(data.error))
        values = (connection, battery, yaw, data.action, data.feedback, protection)
        for column, value in enumerate(values, start=2):
            self.item(row, column).setText(str(value))
        color = "#1a9b45" if data.connected else "#8e8e93"
        self.item(row, 2).setForeground(QColor(color))


class MainWindow(QMainWindow):
    def __init__(self, config_path: Path) -> None:
        super().__init__()
        self.application_name = load_application_name(config_path)
        configured_mode, self.configs, self.safety = load_config(config_path)
        settings_dir = (
            LOCAL_ROOT / "sim" if Path(config_path).resolve() == DEMO_CONFIG.resolve()
            else Path(config_path).parent
        )
        self.odometry_config_path = settings_dir / "odometry.json"
        self.position_config_path = settings_dir / "positioning.json"
        self.dance_directory = Path(config_path).parent
        self.odometry_scales = {
            config.robot_id: {key: 1.0 for key, _ in ODOMETRY_SCALE_SPECS}
            for config in self.configs
        }
        self.lateral_feedback_configs = {
            config.robot_id: {
                "kp": self.safety.lateral_kp,
                "ki": self.safety.lateral_ki,
                "kd": self.safety.lateral_kd,
                "max_correction": self.safety.max_lateral_correction,
            }
            for config in self.configs
        }
        self._load_odometry_config()
        self.fleet: FleetController | None = None
        self.odometry_window: OdometryCalibrationWindow | None = None
        self.position_window: PositionWindow | None = None
        self.tag_positions: dict[str, tuple[float, float, float]] = {}
        self.tag_origins: dict[str, tuple[float, float]] = {}
        self.tag_headings: dict[str, float] = {}
        self.tag_stale_seconds = 12.0
        self.tag_return_speed_percent = 100.0
        self.tag_continuous_return = True
        self.tag_return_latency_ms = 300.0
        self.tag_return_max_seconds = 4.0
        self.tag_direction_locks: dict[str, str] = {}
        self.tag_hold_targets: tuple[str, ...] = ()
        self.yaw_offsets = {config.robot_id: 0.0 for config in self.configs}
        self.yaw_calibrated: set[str] = set()
        self.dance_phases: tuple[DancePhase, ...] = ()
        self.dance_started_cues: set[int] = set()
        self.dance_finished_cues: set[int] = set()
        self.dance_started_at = 0.0
        self.dance_total_duration = 0.0
        self.dance_targets: list[str] = []
        self.dance_stunt_yaw_targets: dict[str, float] = {}
        self.formation_sync = FormationProgressSynchronizer()
        self.audio_output = QAudioOutput(self)
        self.audio_output.setVolume(0.65)
        self.music_player = QMediaPlayer(self)
        self.music_player.setAudioOutput(self.audio_output)
        self.music_path: Path | None = None
        self.active_velocity = (0.0, 0.0, 0.0)
        self.diagnostic_recorder: StraightLineCsvRecorder | None = None
        self.voice_worker: VoiceRecognitionWorker | None = None
        self.voice_executor: VoiceExecutor | None = None
        self.setWindowTitle(self.application_name.replace("\n", " "))
        self.resize(1280, 920)
        self.setMinimumSize(1100, 850)

        root = QWidget()
        root.setObjectName("contentRoot")
        outer = QVBoxLayout(root)
        outer.setContentsMargins(18, 12, 18, 10)
        outer.setSpacing(8)
        self.setCentralWidget(root)

        header = SudaOpeningHeader()
        header.setObjectName("headerBar")
        header_layout = QVBoxLayout(header)
        header_layout.setContentsMargins(18, 9, 18, 9)
        header_layout.setSpacing(4)
        title_lines = self.application_name.split("\n", 1)
        application_title = QLabel(title_lines[0])
        application_title.setObjectName("applicationTitle")
        application_title.setAlignment(Qt.AlignCenter)
        application_subtitle = QLabel(title_lines[1] if len(title_lines) > 1 else "")
        application_subtitle.setObjectName("applicationSubtitle")
        application_subtitle.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(application_title)
        header_layout.addWidget(application_subtitle)
        city_signature = QLabel("养天地正气  ·  法古今完人  ·  智启新程")
        city_signature.setObjectName("citySignature")
        city_signature.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(city_signature)
        header_layout.addSpacing(7)

        top = QHBoxLayout()
        top.setContentsMargins(0, 6, 0, 0)
        self.mode = QComboBox()
        self.mode.addItem(
            "真机模式" if configured_mode == "real" else "模拟模式", configured_mode
        )
        self.confirm_real = QCheckBox("我已清空场地并准备独立急停")
        self.confirm_real.setVisible(configured_mode == "real")
        self.connect_button = QPushButton("连接所选设备")
        self.connect_button.setProperty("role", "primary")
        self.disconnect_button = QPushButton("断开所选设备")
        self.restart_sdk_button = QPushButton("重启SDK连接")
        self.restart_sdk_button.setVisible(configured_mode == "real")
        self.restart_sdk_button.setToolTip(
            "销毁选中设备卡死的SDK会话，等待后重新连接；不会重启机器人系统或发送动作"
        )
        top.addWidget(QLabel("运行模式："))
        top.addWidget(self.mode)
        top.addWidget(self.confirm_real)
        top.addStretch()
        top.addWidget(self.connect_button)
        top.addWidget(self.restart_sdk_button)
        top.addWidget(self.disconnect_button)
        header_layout.addLayout(top)
        outer.addWidget(header)

        status_box = QGroupBox(f"设备状态（已配置 {len(self.configs)} 台）")
        status_layout = QVBoxLayout(status_box)
        self.status_table = FleetStatusTable(self.configs)
        status_layout.addWidget(self.status_table)
        outer.addWidget(status_box)

        self.control_tabs = QTabWidget()
        self.control_tabs.setDocumentMode(True)
        outer.addWidget(self.control_tabs, 1)

        basic_scroll = QScrollArea()
        basic_scroll.setWidgetResizable(True)
        basic_scroll.setFrameShape(QScrollArea.NoFrame)
        basic_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        basic_page = QWidget()
        basic_layout = QVBoxLayout(basic_page)
        basic_layout.setContentsMargins(0, 0, 0, 0)
        basic_layout.setSpacing(10)
        basic_scroll.setWidget(basic_page)
        self.control_tabs.addTab(basic_scroll, "基础运动")

        target_box = QGroupBox("控制目标")
        target_layout = QHBoxLayout(target_box)
        target_grid = QGridLayout()
        target_grid.setHorizontalSpacing(18)
        target_grid.setVerticalSpacing(8)
        self.targets: dict[str, QCheckBox] = {}
        for index, config in enumerate(self.configs):
            configured = configured_mode == "sim" or not config.ip.startswith("192.0.2.")
            check = QCheckBox(config.name if configured else config.name + "（待配置）")
            check.setChecked(index < 10 and configured)
            check.setEnabled(configured)
            self.targets[config.robot_id] = check
            target_grid.addWidget(check, index // 5, index % 5)
        target_layout.addLayout(target_grid, 1)
        target_actions = QVBoxLayout()
        select_all = QPushButton("全选")
        clear_all = QPushButton("清空")
        select_all.clicked.connect(lambda: self.set_all_targets(True))
        clear_all.clicked.connect(lambda: self.set_all_targets(False))
        target_actions.addWidget(select_all)
        target_actions.addWidget(clear_all)
        target_layout.addLayout(target_actions)
        basic_layout.addWidget(target_box)

        control_box = QGroupBox("基础运动控制（按住移动，松开停止）")
        controls = QGridLayout(control_box)
        self.speed = QSpinBox()
        self.speed.setRange(20, 100)
        self.speed.setValue(50)
        self.speed.setSuffix(" %")
        controls.addWidget(QLabel("速度档位"), 0, 0)
        controls.addWidget(self.speed, 0, 1)
        self.target_yaw = QDoubleSpinBox()
        self.target_yaw.setRange(-180.0, 180.0)
        self.target_yaw.setDecimals(1)
        self.target_yaw.setSingleStep(1.0)
        self.target_yaw.setSuffix(" °")
        self.yaw_hold = QCheckBox("启用平移 Yaw PID（前后/横移）")
        self.yaw_hold.setToolTip(
            "前进、后退、左移和右移时保持统一目标 Yaw；人工左转/右转仍然优先"
        )
        self.lateral_hold = QCheckBox("实验性二维轨迹纠偏（默认关闭）")
        self.lateral_hold.setToolTip(
            "前后走时修正左右偏移，横移时修正前后偏移；需要有效的融合里程计"
        )
        self.speed_sync = QCheckBox("启用六方向速度 / 进度同步")
        self.speed_sync.setToolTip(
            "平移使用标定后的腿部里程计，旋转使用 IMU 角速度；补偿各机速度差和行进进度差"
        )
        self.speed_sync.setChecked(True)
        self.record_diagnostics = QCheckBox("记录控制诊断 CSV")
        self.record_diagnostics.setToolTip(
            "记录实际下发指令、IMU、motion_state/gait、SDK/EKF 速度和四腿筛选结果到 diagnostics 文件夹"
        )
        self.adaptive_slip = QCheckBox("启用在线侧滑自适应")
        self.adaptive_slip.setToolTip(
            "每台机器狗分别学习前进、后退、左移、右移的横向侧滑；"
            "停止和转向时冻结学习，重新直行时继续使用"
        )
        self.adaptive_slip.setChecked(True)
        self.position_hold = QCheckBox("启用标签零位保持 / 特技后自动回位")
        self.position_hold.setToolTip(
            "先打开 TAG 定位窗口，为每只狗标定零位；启用后立即回到零位，"
            "特技结束后也会自动回位。手动方向键和急停始终优先。"
        )
        controls.addWidget(QLabel("统一目标 Yaw"), 0, 2)
        controls.addWidget(self.target_yaw, 0, 3)
        controls.addWidget(self.yaw_hold, 0, 4)
        controls.addWidget(self.lateral_hold, 0, 5, 1, 2)
        secondary_controls = QWidget()
        secondary_controls_layout = QHBoxLayout(secondary_controls)
        secondary_controls_layout.setContentsMargins(0, 0, 0, 0)
        secondary_controls_layout.setSpacing(24)
        secondary_controls_layout.addWidget(self.record_diagnostics)
        secondary_controls_layout.addStretch(1)
        secondary_controls_layout.addWidget(self.adaptive_slip)
        secondary_controls_layout.addWidget(self.speed_sync)
        controls.addWidget(secondary_controls, 3, 0, 1, 7)
        tag_controls = QWidget()
        tag_controls_layout = QHBoxLayout(tag_controls)
        tag_controls_layout.setContentsMargins(0, 0, 0, 0)
        tag_controls_layout.addWidget(self.position_hold)
        tag_controls_layout.addStretch(1)
        tag_controls_layout.addWidget(QLabel("TAG1～TAG10 默认对应 dog01～dog10"))
        controls.addWidget(tag_controls, 4, 0, 1, 7)
        lateral_limit = QLabel(
            f"纯横移起步：{self.safety.min_lateral_actuation:.2f}～"
            f"{self.safety.max_y:.2f} m/s"
        )
        lateral_limit.setStyleSheet("color: #a05a00; font-weight: bold")
        controls.addWidget(lateral_limit, 2, 5, 1, 2)
        self.stand = QPushButton("站立")
        self.sit = QPushButton("坐下")
        self.walk = QPushButton("普通行走")
        self.calibrate_yaw = QPushButton("标定所选机器狗当前位置为零位（x/y/yaw=0）")
        controls.addWidget(self.calibrate_yaw, 2, 0, 1, 2)
        controls.addWidget(self.stand, 2, 2)
        controls.addWidget(self.sit, 2, 3)
        controls.addWidget(self.walk, 2, 4)

        direction_specs = [
            ("左转", 1, 0, (0, 0, 1)),
            ("左移", 1, 1, (0, 1, 0)),
            ("前进", 1, 2, (1, 0, 0)),
            ("停止", 1, 3, (0, 0, 0)),
            ("后退", 1, 4, (-1, 0, 0)),
            ("右移", 1, 5, (0, -1, 0)),
            ("右转", 1, 6, (0, 0, -1)),
        ]
        for text, row, column, vector in direction_specs:
            button = QPushButton(text)
            button.setProperty("role", "stop" if vector == (0, 0, 0) else "motion")
            button.setMinimumHeight(34)
            if vector == (0, 0, 0):
                button.clicked.connect(self.stop_motion)
            else:
                button.pressed.connect(lambda v=vector: self.start_motion(v))
                button.released.connect(self.stop_motion)
            controls.addWidget(button, row, column)
        basic_layout.addWidget(control_box)

        odometry_actions = QHBoxLayout()
        self.open_odometry_button = QPushButton("打开里程计标定窗口...")
        self.open_odometry_button.setProperty("role", "primary")
        self.open_position_button = QPushButton("打开 TAG1～TAG10 串口定位...")
        self.open_position_button.setProperty("role", "primary")
        odometry_actions.addStretch()
        odometry_actions.addWidget(self.open_position_button)
        odometry_actions.addWidget(self.open_odometry_button)
        basic_layout.addLayout(odometry_actions)
        basic_layout.addStretch(1)
        # At Windows display scaling above 100%, Qt's style-sheet minimum
        # heights can exceed QGridLayout's reported minimum.  Without a
        # scrollable minimum-height page the grid then places adjacent rows on
        # top of one another.  Keep the content at its real preferred height;
        # the tab scrolls when the main window is too short.
        basic_page.setMinimumHeight(basic_layout.sizeHint().height())

        stunt_page = QWidget()
        stunt_page_layout = QVBoxLayout(stunt_page)
        stunt_page_layout.setContentsMargins(0, 0, 0, 0)
        self.control_tabs.addTab(stunt_page, "特技动作")
        stunt_box = QGroupBox("单次特技（动作后自动恢复普通 Walk）")
        stunt_layout = QGridLayout(stunt_box)
        self.confirm_stunts = QCheckBox("确认场地、落地区域和独立急停均已准备")
        stunt_layout.addWidget(self.confirm_stunts, 0, 0, 1, 4)
        self.non_flip_seconds = QDoubleSpinBox()
        self.non_flip_seconds.setRange(0.5, 10.0)
        self.non_flip_seconds.setDecimals(1)
        self.non_flip_seconds.setSingleStep(0.5)
        self.non_flip_seconds.setValue(self.safety.non_flip_action_seconds)
        self.non_flip_seconds.setSuffix(" s")
        stunt_layout.addWidget(QLabel("非空翻动作持续"), 0, 4)
        stunt_layout.addWidget(self.non_flip_seconds, 0, 5)
        stunt_specs = [
            ("左翻（一次）", Command.LEFT_FLIP),
            ("右翻（一次）", Command.RIGHT_FLIP),
            ("Pronk（一次）", Command.PRONK),
            ("Pace（一次）", Command.PACE),
            ("Bound（一次）", Command.BOUND),
            ("双足正立（一次）", Command.BIPEDAL),
            ("双足倒立（一次）", Command.INV_BIPEDAL),
            ("Run（一次）", Command.RUN),
            ("Up Jump（一次）", Command.UP_JUMP),
            ("Front Jump（一次）", Command.FRONT_JUMP),
            ("Back Jump（一次）", Command.BACK_JUMP),
            ("Left Jump（一次）", Command.LEFT_JUMP),
            ("Right Jump（一次）", Command.RIGHT_JUMP),
            ("Upright（一次）", Command.UPRIGHT),
        ]
        for index, (text, command) in enumerate(stunt_specs):
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, c=command: self.trigger_one_shot(c))
            stunt_layout.addWidget(button, 1 + index // 6, index % 6)
        self.imu_heading_return = QCheckBox(
            "特技后用 IMU 回到统一目标 Yaw（面向观众）"
        )
        self.imu_heading_return.setChecked(True)
        self.imu_heading_return.setToolTip(
            "无 UWB 回零时使用；勾选“特技后立刻回标零位置”时，回位会立即开始并与航向纠正同时执行"
        )
        stunt_layout.addWidget(self.imu_heading_return, 4, 0, 1, 6)
        self.stunt_return_to_start = QCheckBox(
            "特技后立刻回标零位置（UWB）"
        )
        self.stunt_return_to_start.setToolTip(
            "先确认并标定初始位置；特技安全锁结束后立即开始，用前后/左右/旋转速度"
            "把机器人拉回标零位置，单次默认在 4 秒内完成"
        )
        stunt_layout.addWidget(self.stunt_return_to_start, 5, 0, 1, 6)

        flow_row = QHBoxLayout()
        flow_row.addWidget(QLabel("一键流程动作："))
        self.flow_action = QComboBox()
        for text, command in stunt_specs:
            self.flow_action.addItem(text, command.value)
        self.flow_action.setMinimumWidth(180)
        self.flow_run_button = QPushButton("一键：自动标零 → 做动作 → 回标零")
        self.flow_run_button.setProperty("role", "primary")
        self.flow_run_button.setToolTip(
            "以当前 TAG/UWB 位置为标零位置，执行一次动作，动作结束后立即回标零"
        )
        self.flow_run_button.clicked.connect(self.run_zero_action_return_flow)
        flow_row.addWidget(self.flow_action)
        flow_row.addWidget(self.flow_run_button, 1)
        stunt_layout.addLayout(flow_row, 6, 0, 1, 6)

        minimum_flip_interval = effective_flip_lock_seconds(self.safety) + 0.5
        self.left_flip_count = QSpinBox()
        self.left_flip_count.setRange(0, 10)
        self.left_flip_count.setValue(1)
        self.left_flip_count.setSuffix(" 次")
        self.right_flip_count = QSpinBox()
        self.right_flip_count.setRange(0, 10)
        self.right_flip_count.setValue(1)
        self.right_flip_count.setSuffix(" 次")
        self.flip_pair_interval = QDoubleSpinBox()
        self.flip_pair_interval.setRange(minimum_flip_interval, 30.0)
        self.flip_pair_interval.setDecimals(1)
        self.flip_pair_interval.setSingleStep(0.5)
        self.flip_pair_interval.setValue(max(5.0, minimum_flip_interval))
        self.flip_pair_interval.setSuffix(" s")
        self.flip_pair_button = QPushButton("开始自定义左右翻测试")
        self.flip_pair_button.setProperty("role", "primary")
        stunt_layout.addWidget(QLabel("左翻次数"), 7, 0)
        stunt_layout.addWidget(self.left_flip_count, 7, 1)
        stunt_layout.addWidget(QLabel("右翻次数"), 7, 2)
        stunt_layout.addWidget(self.right_flip_count, 7, 3)
        stunt_layout.addWidget(QLabel("动作间隔"), 7, 4)
        stunt_layout.addWidget(self.flip_pair_interval, 7, 5)
        stunt_layout.addWidget(self.flip_pair_button, 8, 0, 1, 6)
        stunt_page_layout.addWidget(stunt_box)
        stunt_page_layout.addStretch()

        voice_page = QWidget()
        voice_page_layout = QVBoxLayout(voice_page)
        voice_page_layout.setContentsMargins(0, 0, 0, 0)
        voice_page_layout.setSpacing(10)
        self.control_tabs.addTab(voice_page, "语音控制")
        voice_box = QGroupBox("语音指令（阿里云百炼）")
        voice_grid = QGridLayout(voice_box)
        self.voice_status = QLabel("就绪")
        self.voice_result = QLabel("识别结果和指令将在这里显示")
        self.voice_result.setWordWrap(True)
        self.voice_result.setMinimumHeight(64)
        self.voice_result.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.voice_button = QPushButton("开始语音识别")
        self.voice_button.setProperty("role", "primary")
        self.voice_cancel_button = QPushButton("取消录音")
        self.voice_cancel_button.setEnabled(False)
        self.voice_stop_button = QPushButton("停止语音指令")
        self.voice_stop_button.setProperty("role", "stop")
        self.voice_stop_button.setEnabled(False)
        self.voice_allow_stunts = QCheckBox("允许特技语音指令（跑步/翻跳/跳跃/双足等）")
        self.voice_speed = QDoubleSpinBox()
        self.voice_speed.setRange(0.1, min(1.0, self.safety.max_x))
        self.voice_speed.setDecimals(2)
        self.voice_speed.setSingleStep(0.05)
        self.voice_speed.setValue(min(0.3, self.safety.max_x))
        self.voice_speed.setSuffix(" m/s")
        self.voice_yaw_speed = QDoubleSpinBox()
        self.voice_yaw_speed.setRange(0.1, min(1.0, self.safety.max_yaw))
        self.voice_yaw_speed.setDecimals(2)
        self.voice_yaw_speed.setSingleStep(0.05)
        self.voice_yaw_speed.setValue(min(0.4, self.safety.max_yaw))
        self.voice_yaw_speed.setSuffix(" rad/s")
        voice_grid.addWidget(QLabel("状态"), 0, 0)
        voice_grid.addWidget(self.voice_status, 0, 1, 1, 3)
        voice_grid.addWidget(self.voice_button, 1, 0)
        voice_grid.addWidget(self.voice_cancel_button, 1, 1)
        voice_grid.addWidget(self.voice_stop_button, 1, 2)
        voice_grid.addWidget(self.voice_allow_stunts, 1, 3)
        voice_grid.addWidget(QLabel("前进/横移速度"), 2, 0)
        voice_grid.addWidget(self.voice_speed, 2, 1)
        voice_grid.addWidget(QLabel("旋转速度"), 2, 2)
        voice_grid.addWidget(self.voice_yaw_speed, 2, 3)
        voice_grid.addWidget(self.voice_result, 3, 0, 1, 4)
        voice_page_layout.addWidget(voice_box)
        voice_page_layout.addStretch()

        self.dance_steps = [
            DanceStep(PhaseKind.TRICK, command=Command.LEFT_FLIP),
            DanceStep(PhaseKind.CORRECT_YAW, duration=3.0),
            DanceStep(PhaseKind.FORWARD, duration=3.0, speed=0.5),
            DanceStep(PhaseKind.STOP, duration=0.1),
            DanceStep(PhaseKind.TRICK, command=Command.RIGHT_FLIP),
            DanceStep(PhaseKind.CORRECT_YAW, duration=3.0),
            DanceStep(PhaseKind.FORWARD, duration=3.0, speed=0.5),
            DanceStep(PhaseKind.STOP, duration=0.1),
        ]
        dance_page = QWidget()
        dance_page_layout = QVBoxLayout(dance_page)
        dance_page_layout.setContentsMargins(0, 0, 0, 0)
        self.control_tabs.addTab(dance_page, "自定义编舞")
        dance_box = QGroupBox("编舞任务")
        dance_layout = QVBoxLayout(dance_box)
        self.dance_summary = QLabel()
        self.open_dance_editor_button = QPushButton("打开编舞编辑器…")
        self.load_dance_button = QPushButton("加载编舞方案…")
        self.save_dance_button = QPushButton("保存编舞方案…")
        self.start_dance_button = QPushButton("开始自定义编舞")
        self.start_dance_button.setProperty("role", "primary")
        self.stop_dance_button = QPushButton("停止编舞")
        self.stop_dance_button.setProperty("role", "stop")
        self.stop_dance_button.setEnabled(False)
        dance_layout.addWidget(self.dance_summary)
        dance_actions = QHBoxLayout()
        dance_actions.addWidget(self.open_dance_editor_button)
        dance_actions.addWidget(self.load_dance_button)
        dance_actions.addWidget(self.save_dance_button)
        dance_actions.addStretch()
        dance_actions.addWidget(self.start_dance_button)
        dance_actions.addWidget(self.stop_dance_button)
        dance_layout.addLayout(dance_actions)
        dance_page_layout.addWidget(dance_box)
        music_box = QGroupBox("编舞音乐")
        music_layout = QHBoxLayout(music_box)
        self.music_name = QLabel("未选择音乐")
        self.sync_music = QCheckBox("编舞开始时同步播放")
        self.sync_music.setChecked(True)
        choose_music = QPushButton("选择本地音乐…")
        self.preview_music_button = QPushButton("试听")
        choose_music.clicked.connect(self.choose_music)
        self.preview_music_button.clicked.connect(self.toggle_music_preview)
        self.music_player.playbackStateChanged.connect(self.update_music_button)
        music_layout.addWidget(self.music_name, 1)
        music_layout.addWidget(self.sync_music)
        music_layout.addWidget(choose_music)
        music_layout.addWidget(self.preview_music_button)
        dance_page_layout.addWidget(music_box)
        dance_page_layout.addStretch()
        demo_music = asset_path("demo_choreography.wav")
        if demo_music.exists():
            self.set_music(demo_music)
        self.update_dance_summary()

        emergency = QPushButton("全体急停 / 阻尼")
        emergency.setObjectName("emergencyButton")
        emergency.setMinimumSize(260, 108)
        emergency.clicked.connect(self.emergency_stop)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setFixedHeight(108)
        self.log.setPlaceholderText("运行日志将在这里显示")
        bottom_bar = QHBoxLayout()
        bottom_bar.setSpacing(10)
        bottom_bar.addWidget(emergency)
        bottom_bar.addWidget(self.log, 1)
        outer.addLayout(bottom_bar)

        self.log_handler = LogHandler(self.log)
        self.log_handler.setFormatter(
            logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%H:%M:%S")
        )
        logging.getLogger().addHandler(self.log_handler)
        logging.getLogger().setLevel(logging.INFO)
        self.file_log_handler: logging.FileHandler | None = None
        self.control_log_path: Path | None = None
        if os.environ.get("QT_QPA_PLATFORM", "").lower() != "offscreen":
            try:
                log_directory = LOCAL_ROOT / "diagnostics"
                log_directory.mkdir(parents=True, exist_ok=True)
                self.control_log_path = (
                    log_directory / f"control_{time.strftime('%Y%m%d_%H%M%S')}.log"
                )
                self.file_log_handler = logging.FileHandler(
                    self.control_log_path, encoding="utf-8"
                )
                self.file_log_handler.setFormatter(
                    logging.Formatter(
                        "%(asctime)s %(levelname)s %(name)s %(message)s"
                    )
                )
                logging.getLogger().addHandler(self.file_log_handler)
                LOG.info("控制事件日志：%s", self.control_log_path)
            except OSError as exc:
                self.file_log_handler = None
                self.control_log_path = None
                LOG.warning("无法创建控制事件日志：%s", exc)

        self.connect_button.clicked.connect(self.connect_selected)
        self.disconnect_button.clicked.connect(self.disconnect_selected)
        self.restart_sdk_button.clicked.connect(self.restart_selected_sdk)
        self.stand.clicked.connect(lambda: self.send(Command.STAND))
        self.sit.clicked.connect(lambda: self.send(Command.SIT))
        self.walk.clicked.connect(lambda: self.send(Command.WALK))
        self.calibrate_yaw.clicked.connect(self.calibrate_common_yaw)
        self.yaw_hold.toggled.connect(self.yaw_hold_changed)
        self.lateral_hold.toggled.connect(self.lateral_hold_changed)
        self.position_hold.toggled.connect(self.position_hold_changed)
        self.record_diagnostics.toggled.connect(self.diagnostics_recording_changed)
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.open_odometry_button.clicked.connect(self.open_odometry_window)
        self.open_position_button.clicked.connect(self.open_position_window)
        self.open_dance_editor_button.clicked.connect(self.open_dance_editor)
        self.load_dance_button.clicked.connect(self.load_dance)
        self.save_dance_button.clicked.connect(self.save_dance)
        self.start_dance_button.clicked.connect(self.start_dance)
        self.stop_dance_button.clicked.connect(self.cancel_dance)
        self.flip_pair_button.clicked.connect(self.start_flip_pair_test)
        self.voice_button.clicked.connect(self.start_voice_recognition)
        self.voice_cancel_button.clicked.connect(self.cancel_voice_recognition)
        self.voice_stop_button.clicked.connect(self.stop_voice_command)

        self.motion_timer = QTimer(self)
        self.motion_timer.setInterval(50)
        self.motion_timer.timeout.connect(self.repeat_motion)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(50)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start()
        self.dance_timer = QTimer(self)
        self.dance_timer.setInterval(50)
        self.dance_timer.timeout.connect(self.dance_tick)
        self.flip_pair_timer = QTimer(self)
        self.flip_pair_timer.setSingleShot(True)
        self.flip_pair_timer.timeout.connect(self.run_next_flip_test)
        self.flip_pair_targets: tuple[str, ...] = ()
        self.flip_test_queue: list[Command] = []
        self.connection_targets = ()
        self.connection_report_after = 0.0
        self.connection_last_missing = None
        self._auto_checked_connected: set[str] = set()
        self.statusBar().showMessage(
            "真机模式：勾选安全确认后，请手动点击“连接所选设备”"
            if configured_mode == "real" else "模拟模式：选择设备后点击“连接所选设备”"
        )

        self.network_panel = NetworkPanel(self.configs, self.network_maintenance_reason, self)
        if configured_mode == "real":
            self.control_tabs.addTab(self.network_panel, "网络诊断")
        else:
            self.network_panel.hide()
        self.network_panel.busy_changed.connect(self.network_job_busy)

    def network_maintenance_reason(self) -> str:
        if self.motion_timer.isActive() or self.dance_timer.isActive() or self.flip_pair_timer.isActive() or self.flip_test_queue:
            return "请先停止运动、编舞和特技任务"
        if self.voice_worker is not None or self.voice_executor is not None or self.position_hold.isChecked():
            return "请先停止语音控制和标签位置保持"
        if self.fleet:
            if any(data.connected for data in self.fleet.snapshots()):
                return "请先在顶部断开已连接设备"
            if any(getattr(worker.driver, "connection_requested", False) for worker in self.fleet.robots.values()):
                return "仍有设备正在后台重连，请先断开这些设备"
        return ""

    def network_job_busy(self, busy: bool) -> None:
        self.connect_button.setEnabled(not busy)
        self.disconnect_button.setEnabled(not busy)
        for index in range(self.control_tabs.count()):
            if self.control_tabs.widget(index) is not self.network_panel:
                self.control_tabs.setTabEnabled(index, not busy)
        for window in (self.position_window, self.odometry_window):
            if window is not None:
                window.setEnabled(not busy)

    def network_is_busy(self) -> bool:
        return bool(getattr(getattr(self, "network_panel", None), "busy", False))

    def open_position_window(self) -> None:
        if self.position_window is None:
            self.position_window = PositionWindow(
                self.configs, self.position_config_path
            )
            self.position_window.sample_processed.connect(
                self._tag_sample_processed
            )
            self.position_window.zero_calibrated.connect(
                self._tag_zero_calibrated
            )
            self.position_window.target_move_requested.connect(
                self._tag_target_move_requested
            )
            self.position_window.target_stop_requested.connect(
                self._tag_target_stop_requested
            )
            self.position_window.target_choreography_requested.connect(
                self._tag_target_choreography_requested
            )
            self.position_window.closed.connect(self._position_window_closed)
            self.position_window.reader.connection_changed.connect(
                self._tag_serial_connection_changed
            )
        self.position_window.show()
        self.position_window.raise_()
        self.position_window.activateWindow()

    def _position_window_closed(self) -> None:
        # With no live serial stream the worker would stop on its stale timer;
        # cancel immediately so the UI and robot state cannot disagree.
        self._cancel_position_hold_ui()

    def _tag_serial_connection_changed(self, connected: bool) -> None:
        if not connected and self.position_hold.isChecked():
            self._cancel_position_hold_ui()
            self.statusBar().showMessage(
                "TAG 定位串口已断开，标签回位已安全取消", 5000
            )

    def _tag_sample_processed(self, payload) -> None:
        window = self.position_window
        if window is None:
            return
        choice = window.binding_choices.get(payload.get("tag"))
        robot_id = choice.currentData() if choice is not None else ""
        raw = payload.get("raw")
        if not robot_id:
            return
        scale = unit_to_meters(window.settings.unit)
        if scale is None:
            return
        if raw is None:
            return
        # 控制/回位直接使用串口原始 TAG 坐标，不做滤波。
        source = raw
        position = (
            float(source[0]) * scale,
            float(source[1]) * scale,
            float(payload["monotonic"]),
        )
        self.tag_positions[robot_id] = position
        if self.fleet:
            self.fleet.update_tag_position(robot_id, *position)

    def _tag_zero_calibrated(self, payload) -> None:
        robot_id = str(payload["robot_id"])
        if not self.fleet:
            self.statusBar().showMessage(
                "标签位置已读取，但控制零位未保存：请先连接机器狗再标定", 6000
            )
            return
        snapshots = {item.robot_id: item for item in self.fleet.snapshots()}
        snapshot = snapshots.get(robot_id)
        if (
            snapshot is None
            or not snapshot.connected
            or snapshot.corrected_pose is None
        ):
            self.statusBar().showMessage(
                f"{robot_id} 尚无有效位姿，标签控制零位未保存", 6000
            )
            return
        origin = (float(payload["meters"][0]), float(payload["meters"][1]))
        received_at = float(payload["received_at"])
        self.tag_origins[robot_id] = origin
        self.tag_headings[robot_id] = float(payload["stage_heading_deg"])
        self.tag_stale_seconds = float(payload["stale_seconds"])
        self.tag_return_speed_percent = float(payload["return_speed_percent"])
        self.tag_continuous_return = bool(payload["continuous_return"])
        self.tag_return_latency_ms = float(payload["return_latency_ms"])
        self.tag_return_max_seconds = float(
            payload.get("return_max_seconds", 4.0)
        )
        self.tag_direction_locks[robot_id] = str(
            payload.get("direction_lock", "xy")
        )
        self.tag_positions[robot_id] = (*origin, received_at)
        self.fleet.update_tag_position(robot_id, *self.tag_positions[robot_id])
        self.fleet.set_tag_zero(
            robot_id,
            origin[0],
            origin[1],
            self.tag_stale_seconds,
            stage_heading_deg=self.tag_headings[robot_id],
            return_speed_percent=self.tag_return_speed_percent,
            continuous_return=self.tag_continuous_return,
            return_latency_ms=self.tag_return_latency_ms,
            return_max_seconds=self.tag_return_max_seconds,
            direction_lock=self.tag_direction_locks.get(robot_id, "xy"),
        )
        self.statusBar().showMessage(
            f"{payload['tag']} / {robot_id} 零位已保存："
            f"({origin[0]:.3f}, {origin[1]:.3f}) m",
            6000,
        )
        LOG.info("%s 标签零位标定完成：%s", robot_id, payload)

    def _tag_target_move_requested(self, payload) -> None:
        if not self.fleet:
            QMessageBox.information(self, "尚未连接", "请先连接机器狗。")
            return
        if self.dance_timer.isActive():
            QMessageBox.information(self, "编舞正在运行", "请先停止编舞再执行地图目标点。")
            return
        if self.position_window is None or not self.position_window.reader.port.isOpen():
            QMessageBox.warning(self, "TAG 串口未连接", "请先连接 TAG 定位串口。")
            return
        entries = list(payload.get("entries", ()))
        snapshots = {item.robot_id: item for item in self.fleet.snapshots()}
        now = time.monotonic()
        stale_seconds = float(payload["stale_seconds"])
        problems = []
        for entry in entries:
            robot_id = str(entry["robot_id"])
            snapshot = snapshots.get(robot_id)
            position = self.tag_positions.get(robot_id)
            if snapshot is None or not snapshot.connected:
                problems.append(robot_id + "（未连接）")
            elif snapshot.corrected_pose is None:
                problems.append(robot_id + "（无有效航向/里程计）")
            elif position is None or now - position[2] > stale_seconds:
                problems.append(robot_id + "（TAG 位置过期）")
        if problems:
            QMessageBox.warning(
                self, "无法前往地图目标点", "请检查以下设备：\n" + "\n".join(problems)
            )
            return

        self._cancel_position_hold_ui()
        self.motion_timer.stop()
        self.active_velocity = (0.0, 0.0, 0.0)
        self.manual_turn_active = False
        targets = tuple(str(entry["robot_id"]) for entry in entries)
        self.tag_hold_targets = targets
        for entry in entries:
            robot_id = str(entry["robot_id"])
            x, y, received_at = self.tag_positions[robot_id]
            self.fleet.update_tag_position(robot_id, x, y, received_at)
            target_x, target_y = entry["meters"]
            self.fleet.start_tag_position_target(
                robot_id,
                float(target_x),
                float(target_y),
                stale_seconds=stale_seconds,
                stage_heading_deg=float(payload["stage_heading_deg"]),
                return_speed_percent=float(payload["return_speed_percent"]),
                continuous_return=bool(payload["continuous_return"]),
                return_latency_ms=float(payload["return_latency_ms"]),
                return_max_seconds=float(
                    payload.get(
                        "return_max_seconds",
                        max(4.0, float(payload.get("duration", 12.0))),
                    )
                ),
                direction_lock=str(
                    entry.get("direction_lock", "xy")
                ),
            )
        self.statusBar().showMessage(
            "地图目标点控制已启动：" + ", ".join(targets), 6000
        )

    def _tag_target_stop_requested(self) -> None:
        targets = tuple(self.tag_hold_targets)
        if self.fleet and targets:
            for robot_id in targets:
                self.fleet.stop_tag_position_hold(robot_id)
        self.tag_hold_targets = ()
        self.statusBar().showMessage("现场试走已停止。", 5000)

    def _tag_target_choreography_requested(self, payload) -> None:
        if self.dance_timer.isActive():
            QMessageBox.information(self, "编舞正在运行", "请先停止编舞再添加点位动作。")
            return
        try:
            existing = build_multitrack_dance(
                list(self.dance_steps),
                effective_flip_lock_seconds(self.safety),
                [config.robot_id for config in self.configs],
                allow_overlap=True,
            ) if self.dance_steps else ()
            start_time = max(
                (phase.start_time + phase.duration for phase in existing), default=0.0
            )
            additions = [
                DanceStep(
                    PhaseKind.TAG_TARGET,
                    duration=float(payload["duration"]),
                    speed=float(payload["return_speed_percent"]),
                    start_time=start_time,
                    targets=(str(entry["robot_id"]),),
                    target_x=float(entry["meters"][0]),
                    target_y=float(entry["meters"][1]),
                    target_name=(
                        f"{payload.get('snapshot_name', '').strip()}/{entry['tag']}"
                        if payload.get("snapshot_name", "").strip()
                        else str(entry["tag"])
                    ),
                )
                for entry in payload["entries"]
            ]
            build_multitrack_dance(
                [*self.dance_steps, *additions],
                effective_flip_lock_seconds(self.safety),
                [config.robot_id for config in self.configs],
                allow_overlap=True,
            )
        except (KeyError, TypeError, ValueError) as exc:
            QMessageBox.warning(self, "无法加入编舞", str(exc))
            return
        self.dance_steps.extend(additions)
        self.update_dance_summary()
        self.statusBar().showMessage(
            f"已在 {start_time:.2f}s 加入 {len(additions)} 条 TAG 目标点编舞指令",
            6000,
        )

    def _load_odometry_config(self) -> None:
        try:
            data = json.loads(self.odometry_config_path.read_text(encoding="utf-8-sig"))
            robot_values = data.get("robots", {})
            for robot_id, stored in robot_values.items():
                if robot_id not in self.odometry_scales:
                    continue
                scales = stored.get("scales", {})
                for key, _ in ODOMETRY_SCALE_SPECS:
                    value = float(scales.get(key, 1.0))
                    if not 0.5 <= value <= 1.5:
                        raise ValueError(f"{robot_id}.{key} 超出 0.5～1.5")
                    self.odometry_scales[robot_id][key] = value
                lateral = stored.get("lateral", {})
                limits = {
                    "kp": (0.0, 5.0),
                    "ki": (0.0, 2.0),
                    "kd": (0.0, 2.0),
                    "max_correction": (
                        0.01, min(self.safety.max_x, self.safety.max_y)
                    ),
                }
                for key, (minimum, maximum) in limits.items():
                    value = float(
                        lateral.get(key, self.lateral_feedback_configs[robot_id][key])
                    )
                    if not minimum <= value <= maximum:
                        raise ValueError(f"{robot_id}.lateral.{key} 超出允许范围")
                    self.lateral_feedback_configs[robot_id][key] = value
        except FileNotFoundError:
            pass
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            LOG.warning("忽略无效的里程计配置 %s: %s", self.odometry_config_path, exc)

    def _save_odometry_config(self) -> None:
        payload = {
            "version": 2,
            "robots": {
                robot_id: {
                    "scales": scales,
                    "lateral": self.lateral_feedback_configs[robot_id],
                }
                for robot_id, scales in self.odometry_scales.items()
            },
        }
        self.odometry_config_path.parent.mkdir(parents=True, exist_ok=True)
        self.odometry_config_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    def open_odometry_window(self) -> None:
        if self.odometry_window is None:
            self.odometry_window = OdometryCalibrationWindow(self)
            self.odometry_robot_changed()
        self.odometry_window.show()
        self.odometry_window.raise_()
        self.odometry_window.activateWindow()

    def current_odometry_robot(self):
        if self.odometry_window is None:
            return None
        return str(self.odometry_window.odometry_robot.currentData())

    def odometry_robot_changed(self, index: int = -1) -> None:
        del index
        window = self.odometry_window
        if window is None:
            return
        robot_id = self.current_odometry_robot()
        values = self.odometry_scales.get(robot_id, {})
        for key, editor in window.odometry_scale_inputs.items():
            editor.setValue(values.get(key, 1.0))
        lateral = self.lateral_feedback_configs.get(robot_id, {})
        for key, editor in window.lateral_feedback_inputs.items():
            editor.setValue(lateral.get(key, editor.minimum()))
        window.odometry_pose.setText("x=-- m    y=-- m    yaw=--°")
        window.odometry_fusion.setText("EKF：等待连接和传感器数据")
        window.odometry_lateral_status.setText("轨迹纠偏：等待启用")

    def set_odometry_pose(self) -> None:
        if not self.fleet:
            QMessageBox.information(self, "尚未初始化", "请先连接至少一台设备。")
            return
        window = self.odometry_window
        if window is None:
            return
        x, y, yaw = (editor.value() for editor in window.odometry_pose_inputs)
        robot_id = self.current_odometry_robot()
        self.fleet.set_odometry_pose(robot_id, x, y, yaw)
        LOG.info("%s 设置绝对位姿 x=%.3f y=%.3f yaw=%.2f°", robot_id, x, y, yaw)
        self.statusBar().showMessage(f"{robot_id} 绝对位姿已设置", 3000)

    def add_odometry_correction(self) -> None:
        if not self.fleet:
            QMessageBox.information(self, "尚未初始化", "请先连接至少一台设备。")
            return
        window = self.odometry_window
        if window is None:
            return
        dx, dy, delta_yaw = (editor.value() for editor in window.odometry_delta_inputs)
        robot_id = self.current_odometry_robot()
        self.fleet.add_odometry_correction(robot_id, dx, dy, delta_yaw)
        for editor in window.odometry_delta_inputs:
            editor.setValue(0.0)
        LOG.info(
            "%s 叠加里程补偿 dx=%+.3f dy=%+.3f dyaw=%+.2f°",
            robot_id,
            dx,
            dy,
            delta_yaw,
        )
        self.statusBar().showMessage(f"{robot_id} 误差补偿已叠加", 3000)

    def apply_odometry_scales(self) -> None:
        robot_id = self.current_odometry_robot()
        values = {
            key: editor.value() for key, editor in self.odometry_window.odometry_scale_inputs.items()
        }
        self.odometry_scales[robot_id] = values
        if self.fleet:
            self.fleet.set_odometry_scales(robot_id, values)
        try:
            self._save_odometry_config()
        except OSError as exc:
            QMessageBox.critical(self, "保存失败", str(exc))
            return
        LOG.info("%s 六方向里程修正系数已应用并保存", robot_id)
        self.statusBar().showMessage(f"{robot_id} 修正系数已保存", 3000)

    def apply_lateral_feedback_config(self) -> None:
        robot_id = self.current_odometry_robot()
        values = {
            key: editor.value() for key, editor in self.odometry_window.lateral_feedback_inputs.items()
        }
        self.lateral_feedback_configs[robot_id] = values
        if self.fleet:
            self.fleet.set_lateral_feedback_config(robot_id, **values)
        try:
            self._save_odometry_config()
        except OSError as exc:
            QMessageBox.critical(self, "保存失败", str(exc))
            return
        LOG.info("%s 轨迹纠偏参数已应用并保存：%s", robot_id, values)
        self.statusBar().showMessage(f"{robot_id} 轨迹纠偏参数已保存", 3000)

    def start_voice_recognition(self) -> None:
        if self.voice_worker is not None or self.voice_executor is not None:
            return
        if not self.ensure_fleet():
            return
        targets = self.selected()
        if not targets:
            QMessageBox.information(self, "没有目标", "请至少选择一台机器狗。")
            return
        snapshots = {item.robot_id: item for item in self.fleet.snapshots()}
        offline = [
            robot_id
            for robot_id in targets
            if not snapshots.get(robot_id) or not snapshots[robot_id].connected
        ]
        if offline:
            QMessageBox.warning(
                self,
                "尚未连接",
                "以下目标未连接：" + ", ".join(offline),
            )
            return
        if self.dance_timer.isActive() and self.dance_phases:
            QMessageBox.information(
                self,
                "编舞进行中",
                "请先停止自定义编舞再使用语音指令。",
            )
            return
        if self.flip_pair_timer.isActive() or self.flip_test_queue:
            QMessageBox.information(
                self,
                "批量特技进行中",
                "请先停止批量特技测试再使用语音指令。",
            )
            return
        try:
            from voice_control.asr import resolve_api_key

            api_key = resolve_api_key(LOCAL_ROOT)
        except Exception as exc:
            QMessageBox.critical(self, "语音配置错误", str(exc))
            return
        self.disable_yaw_hold_and_stop()
        self.voice_status.setText("正在准备麦克风…")
        self.voice_result.setText("")
        self.voice_button.setEnabled(False)
        self.voice_cancel_button.setEnabled(True)
        self.voice_stop_button.setEnabled(False)
        self.voice_worker = VoiceRecognitionWorker(
            api_key=api_key,
            model="paraformer-realtime-v2",
            device=None,
            silence_seconds=1.5,
            max_seconds=15.0,
            available_ids=[config.robot_id for config in self.configs],
            default_targets=tuple(targets),
            speed=self.voice_speed.value(),
            yaw_speed=self.voice_yaw_speed.value(),
        )
        self.voice_worker.status_changed.connect(self.voice_status.setText)
        self.voice_worker.result_ready.connect(self.handle_voice_result)
        self.voice_worker.failed.connect(self.handle_voice_error)
        self.voice_worker.finished.connect(self.voice_worker_finished)
        self.voice_worker.start()
        LOG.info("语音识别开始，目标：%s", ", ".join(targets))

    def cancel_voice_recognition(self) -> None:
        if self.voice_worker is not None:
            self.voice_worker.cancel()
            self.voice_status.setText("正在取消…")

    def stop_voice_command(self) -> None:
        if self.voice_executor is not None:
            self.voice_executor.stop()
            self.voice_status.setText("已截止，正在停止…")
            self.voice_result.setText(self.voice_result.text() + "\n操作员截止")

    def handle_voice_result(self, text: str, intents) -> None:
        if not text or not intents:
            self.voice_result.setText(
                f"识别：{text or '（无语音）'}\n未识别到可用指令。"
            )
            self.voice_status.setText("就绪")
            return
        blocked = [
            intent
            for intent in intents
            if intent.command in VOICE_STUNT_COMMANDS
            and not (
                self.confirm_stunts.isChecked()
                and self.voice_allow_stunts.isChecked()
            )
        ]
        if blocked:
            labels = "、".join(intent.label for intent in blocked)
            self.voice_result.setText(
                f"识别：{text}\n特技指令「{labels}」已拦截，"
                "请勾选特技页安全确认和语音特技开关。"
            )
            self.voice_status.setText("已拦截")
            LOG.warning("语音特技指令被拦截：%s", labels)
            return
        summary = "；".join(self._format_voice_intent(intent) for intent in intents)
        self.voice_result.setText(f"识别：{text}\n指令：{summary}")
        self.voice_status.setText("执行中…")
        LOG.info("语音指令：%s", summary)
        self.disable_yaw_hold_and_stop()
        executor = VoiceExecutor(self.fleet, intents)
        executor.step_changed.connect(self.voice_status.setText)
        executor.finished.connect(self.handle_voice_executed)
        executor.failed.connect(self.handle_voice_error)
        executor.start()
        self.voice_executor = executor
        self.voice_stop_button.setEnabled(True)

    @staticmethod
    def _format_voice_intent(intent) -> str:
        detail = f"{intent.label}->{', '.join(intent.targets)}"
        if intent.command is Command.VELOCITY:
            params = ", ".join(
                f"{key}={value:.2f}" for key, value in intent.params.items()
            )
            detail += f" | {params} | {intent.duration_s:.1f}s"
        return detail

    def handle_voice_executed(self, message: str) -> None:
        self.voice_status.setText("就绪")
        self.voice_result.setText(self.voice_result.text() + f"\n{message}")
        self.voice_executor = None
        self.voice_stop_button.setEnabled(False)
        if self.voice_worker is None:
            self.voice_button.setEnabled(True)
        LOG.info(message)

    def handle_voice_error(self, message: str) -> None:
        self.voice_status.setText("失败")
        self.voice_result.setText(f"语音控制失败：{message}")
        self.voice_executor = None
        self.voice_stop_button.setEnabled(False)
        if self.voice_worker is None:
            self.voice_button.setEnabled(True)
        LOG.error("语音控制失败：%s", message)

    def voice_worker_finished(self) -> None:
        self.voice_cancel_button.setEnabled(False)
        self.voice_worker = None
        if self.voice_executor is None:
            self.voice_button.setEnabled(True)

    def selected(self) -> list[str]:
        return [robot_id for robot_id, box in self.targets.items() if box.isChecked()]

    def connected_selected(self) -> list[str]:
        if self.fleet is None or not hasattr(self.fleet, "snapshots"):
            return self.selected()
        by_id = {item.robot_id: item for item in self.fleet.snapshots()}
        return [
            robot_id
            for robot_id in self.selected()
            if by_id.get(robot_id) is not None and by_id[robot_id].connected
        ]

    def set_all_targets(self, selected: bool) -> None:
        for box in self.targets.values():
            if box.isEnabled():
                box.setChecked(selected)

    def ensure_fleet(self) -> bool:
        if self.network_is_busy():
            self.statusBar().showMessage("网络检测/优化进行中，请等待完成后操作", 3000)
            return False
        real = self.mode.currentData() == "real"
        if real and not self.confirm_real.isChecked():
            QMessageBox.warning(self, "真机保护", "请先确认场地和独立急停已经准备好。")
            return False
        if self.fleet is not None:
            return True
        factory = BpxDriver if real else SimDriver
        try:
            self.fleet = FleetController(self.configs, self.safety, factory)
            self.fleet.connect_stagger_seconds = CONNECT_STAGGER_SECONDS if real else 0.0
            for robot_id, scales in self.odometry_scales.items():
                self.fleet.set_odometry_scales(robot_id, scales)
                self.fleet.set_lateral_feedback_config(
                    robot_id, **self.lateral_feedback_configs[robot_id]
                )
        except Exception as exc:
            QMessageBox.critical(self, "初始化失败", str(exc))
            return False
        self.mode.setEnabled(False)
        if real:
            self.confirm_real.setEnabled(False)
        LOG.info("已初始化%s模式", "真机" if real else "模拟")
        return True

    def report_connections(self, snapshots) -> None:
        if not self.connection_targets or time.monotonic() < self.connection_report_after:
            return
        by_id = {item.robot_id: item for item in snapshots}
        missing = tuple(robot_id for robot_id in self.connection_targets
                        if robot_id not in by_id or not by_id[robot_id].connected)
        if missing == self.connection_last_missing:
            return
        self.connection_last_missing = missing
        mode_label = "真机"
        if missing:
            configs = {item.robot_id: item for item in self.configs}
            details = [f"{robot_id} ({configs[robot_id].ip})："
                       + (getattr(by_id.get(robot_id), "error", "") or "未连接/未收到状态")
                       for robot_id in missing]
            LOG.warning("[%s] 未连接 %d/%d 台：%s", mode_label, len(missing),
                        len(self.connection_targets), "；".join(details))
        else:
            LOG.info("[%s] 本次目标 %d 台全部已连接", mode_label, len(self.connection_targets))

    def connect_selected(self) -> None:
        targets = self.selected()
        if not targets:
            QMessageBox.information(self, "没有目标", "请至少选择一台机器狗。")
            return
        if self.ensure_fleet():
            self.fleet.connect(targets)
            self.connection_targets = tuple(targets)
            self.connection_report_after = time.monotonic() + 12.0
            self.connection_last_missing = None
            LOG.info("提交连接请求：%s", ", ".join(targets))

    def disconnect_selected(self) -> None:
        if self.flip_pair_timer.isActive() or self.flip_test_queue:
            self.cancel_flip_pair_test("设备断开，剩余翻转已取消")
        self.disable_yaw_hold_and_stop()
        self.formation_sync.reset()
        if self.fleet:
            self.fleet.disconnect(self.selected())
        for robot_id in self.selected():
            self.yaw_calibrated.discard(robot_id)

    def restart_selected_sdk(self) -> None:
        targets = self.selected()
        if not targets:
            QMessageBox.information(self, "没有目标", "请至少选择一台机器狗。")
            return
        if not self.ensure_fleet():
            return
        if self.flip_pair_timer.isActive() or self.flip_test_queue:
            self.cancel_flip_pair_test("SDK重启，剩余翻转已取消")
        if self.dance_timer.isActive():
            self.cancel_dance(send_stop=True)
        self.disable_yaw_hold_and_stop()
        self.formation_sync.reset()
        self.connection_targets = ()
        self.connection_last_missing = None
        self.connection_report_after = 0.0
        self.restart_sdk_button.setEnabled(False)
        self.statusBar().showMessage("正在重启选中设备的SDK连接…", 3000)
        LOG.warning("重启SDK连接：%s", ", ".join(targets))
        if self.fleet is not None:
            self.fleet.disconnect(targets)
        QTimer.singleShot(
            1500,
            lambda selected=tuple(targets): self._reconnect_after_sdk_restart(selected),
        )

    def _reconnect_after_sdk_restart(self, targets: tuple[str, ...]) -> None:
        self.restart_sdk_button.setEnabled(True)
        if self.fleet is None or not targets:
            return
        self.fleet.connect(targets)
        self.connection_targets = targets
        self.connection_report_after = time.monotonic() + 12.0
        self.connection_last_missing = None
        self.statusBar().showMessage("已重新提交SDK连接请求", 3000)
        LOG.info("SDK重启后重新连接：%s", ", ".join(targets))

    def send(self, command: Command) -> None:
        if self.network_is_busy() and command not in {Command.STOP, Command.DAMPING}:
            return
        if self.fleet:
            self.fleet.submit(self.selected(), command)
        else:
            self.statusBar().showMessage("请先连接设备", 2500)

    def start_motion(self, vector: tuple[int, int, int]) -> None:
        if self.network_is_busy():
            return
        self._cancel_position_hold_ui()
        if self.dance_timer.isActive():
            self.cancel_dance(send_stop=True)
        if self.flip_pair_timer.isActive() or self.flip_test_queue:
            self.cancel_flip_pair_test("手动控制已接管，剩余翻转已取消")
        scale = self.speed.value() / 100.0
        lateral_speed = vector[1] * self.safety.max_y * scale
        if vector[1] and abs(lateral_speed) < self.safety.min_lateral_actuation:
            lateral_speed = math.copysign(
                self.safety.min_lateral_actuation, lateral_speed
            )
        self.active_velocity = (
            vector[0] * self.safety.max_x * scale,
            lateral_speed,
            vector[2] * self.safety.max_yaw * scale,
        )
        self.manual_turn_active = bool(vector[2])
        self.formation_sync.reset("manual")
        offline = [
            robot_id
            for robot_id in self.selected()
            if robot_id not in self.connected_selected()
        ]
        if offline:
            LOG.warning(
                "以下已选设备未连接，本次手动运动仅控制已连接设备：%s",
                ", ".join(offline),
            )
            self.statusBar().showMessage(
                "未连接设备已跳过：" + ", ".join(offline), 3000
            )
        self.repeat_motion()
        self.motion_timer.start()

    def formation_velocity_targets(
        self,
        targets: list[str] | tuple[str, ...],
        x: float,
        y: float,
        session_key,
    ) -> dict[str, tuple[float, float, str]]:
        defaults = {robot_id: (x, y, "") for robot_id in targets}
        synchronizer = getattr(self, "formation_sync", None)
        enabled = self.speed_sync.isChecked()
        if synchronizer is None or not enabled or math.hypot(x, y) < 1e-6:
            if synchronizer is not None:
                synchronizer.reset(session_key)
            return defaults
        snapshots = {item.robot_id: item for item in self.fleet.snapshots()}
        return synchronizer.apply(
            targets, x, y, snapshots, time.monotonic(), session_key=session_key,
            limits=(self.safety.max_x, self.safety.max_y),
        )

    def repeat_motion(self) -> None:
        if self.fleet:
            x, y, yaw = self.active_velocity
            common_target = math.radians(self.target_yaw.value())
            if hasattr(self.fleet, "snapshots"):
                by_id = {item.robot_id: item for item in self.fleet.snapshots()}
                targets = [
                    robot_id
                    for robot_id in self.selected()
                    if by_id.get(robot_id) is not None and by_id[robot_id].connected
                ]
            else:
                targets = self.selected()
            if not targets:
                return
            synchronized = MainWindow.formation_velocity_targets(
                self, targets, x, y, session_key="manual"
            )
            for robot_id in targets:
                synced_x, synced_y, formation_status = synchronized[robot_id]
                params = {
                    "x": synced_x,
                    "y": synced_y,
                    "yaw": yaw,
                    "operator_override": True,
                }
                if formation_status:
                    params["formation_status"] = formation_status
                if self.yaw_hold.isChecked() and abs(yaw) < 1e-6:
                    params["target_yaw"] = raw_target_for_common_yaw(
                        common_target, self.yaw_offsets[robot_id]
                    )
                params["path_correction"] = (
                    self.lateral_hold.isChecked()
                    and math.hypot(x, y) > 1e-6
                    and abs(yaw) < 1e-6
                )
                params["speed_sync"] = (
                    self.speed_sync.isChecked()
                    and (math.hypot(x, y) > 1e-6 or abs(yaw) > 1e-6)
                )
                adaptive_slip = getattr(self, "adaptive_slip", None)
                params["adaptive_slip"] = bool(
                    adaptive_slip is not None
                    and adaptive_slip.isChecked()
                    and math.hypot(x, y) > 1e-6
                    and abs(yaw) < 1e-6
                )
                self.fleet.submit([robot_id], Command.VELOCITY, **params)

    def calibrate_common_yaw(self) -> None:
        if not self.fleet:
            QMessageBox.information(self, "尚未连接", "请先连接所选机器狗。")
            return
        targets = self.selected()
        snapshots = {item.robot_id: item for item in self.fleet.snapshots()}
        unavailable = [
            robot_id for robot_id in targets
            if not snapshots[robot_id].connected or snapshots[robot_id].rpy is None
        ]
        if unavailable:
            QMessageBox.warning(
                self, "无法校准", "以下设备尚无有效 IMU 数据：" + ", ".join(unavailable)
            )
            return
        for robot_id in targets:
            raw_yaw = snapshots[robot_id].rpy[2]
            self.yaw_offsets[robot_id] = yaw_calibration_offset(raw_yaw, 0.0)
            self.yaw_calibrated.add(robot_id)
            self.fleet.set_odometry_pose(robot_id, 0.0, 0.0, 0.0)
            LOG.info(
                "%s 统一朝向校准：原始 yaw=%+.2f°，偏置=%+.2f°",
                robot_id,
                math.degrees(raw_yaw),
                math.degrees(self.yaw_offsets[robot_id]),
            )
        self.target_yaw.setValue(0.0)
        self.statusBar().showMessage(
            "所选机器狗零位标定完成：当前位置 x=0、y=0、yaw=0°", 5000
        )

    def run_zero_action_return_flow(self) -> None:
        """One-click flow: current UWB -> zero, run action, return to zero."""
        command = Command(str(self.flow_action.currentData()))
        if not self.fleet:
            QMessageBox.information(self, "尚未连接", "请先连接机器狗。")
            return
        targets = self.selected()
        if not targets:
            QMessageBox.information(self, "没有目标", "请至少选择一台机器狗。")
            return
        if not self.confirm_stunts.isChecked():
            QMessageBox.warning(
                self,
                "特技安全确认",
                "请先确认场地、落地区域以及独立急停已经准备好。",
            )
            return
        window = self.position_window
        if window is None or not window.reader.port.isOpen():
            QMessageBox.warning(
                self,
                "UWB 未连接",
                "请先打开 TAG 定位窗口并连接 UWB 串口，再使用一键流程。",
            )
            return
        settings = window.settings
        if not settings.xy_confirmed or not settings.heading_confirmed:
            QMessageBox.warning(
                self,
                "定位方向未确认",
                "请先在 TAG 定位窗口确认 X/Y 方向和标零机头方向。",
            )
            return
        scale = unit_to_meters(settings.unit)
        if scale is None:
            QMessageBox.warning(self, "单位未确认", "请先选择并确认 UWB 坐标单位。")
            return
        now = time.monotonic()
        stale_seconds = control_stale_seconds(settings.stale_seconds)
        snapshots = {item.robot_id: item for item in self.fleet.snapshots()}
        failed = []
        for robot_id in targets:
            position = self.tag_positions.get(robot_id)
            snapshot = snapshots.get(robot_id)
            if (
                snapshot is None
                or not snapshot.connected
                or snapshot.corrected_pose is None
            ):
                failed.append(robot_id + "（未连接或无航向）")
            elif position is None or now - position[2] > stale_seconds:
                failed.append(robot_id + "（TAG 位置过期）")
        if failed:
            QMessageBox.warning(
                self,
                "无法自动标零",
                "请等待 UWB 实时数据并重新选择设备：\n" + "\n".join(failed),
            )
            return

        self.motion_timer.stop()
        self.active_velocity = (0.0, 0.0, 0.0)
        self.manual_turn_active = False
        for robot_id in targets:
            x, y, received_at = self.tag_positions[robot_id]
            origin_x = float(x)
            origin_y = float(y)
            self.fleet.update_tag_position(robot_id, x, y, received_at)
            self.fleet.set_tag_zero(
                robot_id,
                origin_x=origin_x,
                origin_y=origin_y,
                stale_seconds=stale_seconds,
                stage_heading_deg=float(settings.zero_heading_deg),
                return_speed_percent=float(settings.return_speed_percent),
                continuous_return=bool(settings.continuous_return),
                return_latency_ms=float(settings.return_latency_ms),
                return_max_seconds=float(settings.return_max_seconds),
                direction_lock=str(
                    settings.direction_locks.get(robot_id, "xy")
                ),
            )
            self.tag_origins[robot_id] = (origin_x, origin_y)
            self.tag_headings[robot_id] = float(settings.zero_heading_deg)
            LOG.warning(
                "%s 一键流程标零位置=(%.3f, %.3f) m，随后执行动作",
                robot_id,
                origin_x,
                origin_y,
            )
        self.stunt_return_to_start.setChecked(True)
        self._set_position_hold_checked(False)
        self.statusBar().showMessage(
            "已用当前 UWB 位置自动标零，开始执行："
            + command.value
            + " → 回标零",
            5000,
        )
        LOG.warning("一键流程：已自动标零 %s，执行 %s", ", ".join(targets), command.value)
        self.trigger_one_shot(command)

    def trigger_one_shot(self, command: Command) -> None:
        LOG.warning(
            "收到特技按钮点击：%s；安全确认=%s，标签回位=%s，特技回标零=%s",
            command.value,
            self.confirm_stunts.isChecked(),
            self.position_hold.isChecked(),
            self.stunt_return_to_start.isChecked(),
        )
        if self.dance_timer.isActive():
            # A directly clicked one-shot is an explicit operator takeover,
            # just like the manual direction buttons.  A stale dance timer
            # must not silently make every stunt button look dead.
            LOG.warning("单次特技接管控制：先停止正在运行的编舞")
            self.cancel_dance(send_stop=True)
        if self.flip_pair_timer.isActive():
            LOG.warning("单次特技接管控制：取消正在等待的批量左右翻测试")
            self.cancel_flip_pair_test("单次特技已接管，剩余翻转已取消")
        if not self.fleet:
            LOG.error("拒绝 %s：控制器尚未初始化", command.value)
            QMessageBox.information(self, "尚未连接", "请先连接机器狗。")
            return
        if not self.confirm_stunts.isChecked():
            LOG.error("拒绝 %s：未勾选特技安全确认", command.value)
            QMessageBox.warning(
                self,
                "特技安全确认",
                "请先确认场地、落地区域以及独立急停已经准备好。",
            )
            return
        targets = self.selected()
        if not targets:
            LOG.error("拒绝 %s：没有选择目标机器狗", command.value)
            QMessageBox.information(self, "没有目标", "请至少选择一台机器狗。")
            return
        LOG.warning("特技目标已确认：%s -> %s", command.value, ", ".join(targets))
        try:
            yaw_targets = self.stunt_heading_targets(
                targets,
                # IMU heading recovery is optional and must never suppress an
                # explicitly confirmed stunt. Without shared-yaw calibration use
                # each robot's current yaw; without IMU, execute without yaw return.
                allow_current_fallback=True,
            )
        except Exception:
            LOG.exception(
                "%s 的 IMU 回正参数准备异常；本次降级为不回正，但继续执行特技",
                command.value,
            )
            yaw_targets = {}
        if yaw_targets is None:
            LOG.error(
                "%s 的 IMU 回正参数不可用；本次降级为不回正，但继续执行特技",
                command.value,
            )
            yaw_targets = {}
        self.motion_timer.stop()
        self.active_velocity = (0.0, 0.0, 0.0)
        self.manual_turn_active = False
        params = {}
        try:
            params.update(
                self._stunt_position_return_params(
                    self.stunt_return_to_start.isChecked()
                )
            )
        except Exception:
            # Position recovery is optional.  Never trade the explicitly
            # requested stunt for a GUI-side positioning/settings failure.
            LOG.exception(
                "%s 的位置回位参数准备异常；本次降级为不回位，但继续执行特技",
                command.value,
            )
            params["return_to_start"] = False
        if command not in {Command.LEFT_FLIP, Command.RIGHT_FLIP}:
            params["duration_s"] = self.non_flip_seconds.value()
        submitted = []
        for robot_id in targets:
            command_params = dict(params)
            if robot_id in yaw_targets:
                command_params["target_yaw"] = yaw_targets[robot_id]
            try:
                self.fleet.submit([robot_id], command, **command_params)
                submitted.append(robot_id)
                LOG.warning(
                    "特技已进入控制器队列：%s -> %s；参数=%s",
                    command.value,
                    robot_id,
                    command_params,
                )
            except Exception:
                LOG.exception(
                    "提交特技失败：%s -> %s", command.value, robot_id
                )
        if not submitted:
            QMessageBox.critical(
                self,
                "特技提交失败",
                f"{command.value} 未能进入任何机器狗的控制队列，请查看控制事件日志。",
            )
            return
        self.statusBar().showMessage(
            f"已提交 {command.value}：" + ", ".join(submitted), 4000
        )
        LOG.warning(
            "提交特技 %s：%s；持续 %.2f s，安全锁定 %d ms",
            command.value,
            ", ".join(submitted),
            (
                effective_flip_trigger_seconds(self.safety)
                if command in {Command.LEFT_FLIP, Command.RIGHT_FLIP}
                else self.non_flip_seconds.value()
            ),
            self.safety.one_shot_return_ms,
        )

    def _stunt_position_return_params(self, enabled: bool) -> dict:
        params = {"return_to_start": bool(enabled)}
        position_window = getattr(self, "position_window", None)
        if not enabled or position_window is None:
            return params
        settings = position_window.settings
        params.update({
            "stale_seconds": control_stale_seconds(settings.stale_seconds),
            "stage_heading_deg": float(settings.zero_heading_deg),
            "return_speed_percent": float(settings.return_speed_percent),
            "continuous_return": bool(settings.continuous_return),
            "return_latency_ms": float(settings.return_latency_ms),
            "return_max_seconds": float(settings.return_max_seconds),
            "direction_locks": dict(settings.direction_locks),
        })
        return params

    def stunt_heading_targets(
        self,
        targets: list[str] | tuple[str, ...],
        show_warning: bool = True,
        allow_current_fallback: bool = False,
    ) -> dict[str, float] | None:
        """Build per-robot raw IMU targets for the shared audience heading."""
        if not self.imu_heading_return.isChecked():
            return {}
        missing_calibration = sorted(set(targets) - self.yaw_calibrated)
        snapshots = {item.robot_id: item for item in self.fleet.snapshots()}
        unavailable_imu = sorted(
            robot_id
            for robot_id in targets
            if robot_id not in snapshots
            or not snapshots[robot_id].connected
            or snapshots[robot_id].rpy is None
        )
        if missing_calibration or unavailable_imu:
            if allow_current_fallback:
                # Returning to the action's own start does not require a
                # shared audience frame. Preserve each available robot's raw
                # pre-action yaw and never block the stunt merely because the
                # optional recovery reference is unavailable.
                return {
                    robot_id: snapshots[robot_id].rpy[2]
                    for robot_id in targets
                    if robot_id in snapshots
                    and snapshots[robot_id].rpy is not None
                }
            details = []
            if missing_calibration:
                details.append("尚未统一朝向标定：" + ", ".join(missing_calibration))
            if unavailable_imu:
                details.append("当前无有效 IMU：" + ", ".join(unavailable_imu))
            if show_warning:
                QMessageBox.warning(
                    self,
                    "无法启用特技后 IMU 回正",
                    "请先在基础控制页把机器狗朝向观众并点击“标定所选机器狗当前位置为零位”。\n"
                    + "\n".join(details),
                )
            return None
        common_target = math.radians(self.target_yaw.value())
        return {
            robot_id: raw_target_for_common_yaw(
                common_target, self.yaw_offsets[robot_id]
            )
            for robot_id in targets
        }

    def start_flip_pair_test(self) -> None:
        if self.flip_pair_timer.isActive():
            return
        if not self.fleet:
            QMessageBox.information(self, "尚未连接", "请先连接机器狗。")
            return
        if not self.confirm_stunts.isChecked():
            QMessageBox.warning(
                self,
                "特技安全确认",
                "请先确认场地、落地区域以及独立急停已经准备好。",
            )
            return
        targets = self.selected()
        if not targets:
            QMessageBox.information(self, "没有目标", "请至少选择一台机器狗。")
            return
        required_interval = effective_flip_lock_seconds(self.safety) + 0.5
        if self.stunt_return_to_start.isChecked() or self.position_hold.isChecked():
            required_interval += self.tag_return_max_seconds
        if self.flip_pair_interval.value() < required_interval:
            QMessageBox.warning(
                self,
                "左右翻间隔过短",
                f"当前模式至少需要 {required_interval:.1f} 秒，请增大间隔。",
            )
            return
        left_count = self.left_flip_count.value()
        right_count = self.right_flip_count.value()
        if left_count + right_count == 0:
            QMessageBox.information(
                self, "没有动作", "左翻和右翻次数不能同时为 0。"
            )
            return
        self.flip_pair_targets = tuple(targets)
        self.flip_test_queue = (
            [Command.LEFT_FLIP] * left_count
            + [Command.RIGHT_FLIP] * right_count
        )
        self.flip_pair_button.setEnabled(False)
        self.run_next_flip_test()

    def run_next_flip_test(self) -> None:
        if tuple(self.selected()) != self.flip_pair_targets:
            self.cancel_flip_pair_test("目标选择已变化，剩余翻转已取消")
            return
        if not self.flip_test_queue:
            self.cancel_flip_pair_test("自定义左右翻测试完成")
            return
        command = self.flip_test_queue.pop(0)
        self.trigger_one_shot(command)
        self.flip_pair_timer.start(round(self.flip_pair_interval.value() * 1000))
        if self.flip_test_queue:
            self.statusBar().showMessage(
                f"已执行{('左翻' if command is Command.LEFT_FLIP else '右翻')}一次；"
                f"剩余 {len(self.flip_test_queue)} 次"
            )
        else:
            self.statusBar().showMessage("最后一次翻转已触发，等待安全期结束")

    def cancel_flip_pair_test(self, message: str = "左右翻测试已取消") -> None:
        if hasattr(self, "flip_pair_timer"):
            self.flip_pair_timer.stop()
        if hasattr(self, "flip_pair_button"):
            self.flip_pair_button.setEnabled(True)
        self.flip_pair_targets = ()
        self.flip_test_queue.clear()
        self.statusBar().showMessage(message, 5000)

    def set_music(self, path: Path) -> None:
        self.music_path = path.resolve()
        self.music_player.setSource(QUrl.fromLocalFile(str(self.music_path)))
        self.music_name.setText("音乐：" + self.music_path.name)

    def choose_music(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self, "选择编舞音乐", str(Path.cwd()),
            "音频文件 (*.wav *.mp3 *.m4a *.aac *.flac);;所有文件 (*)",
        )
        if filename:
            self.set_music(Path(filename))

    def toggle_music_preview(self) -> None:
        if self.music_path is None:
            QMessageBox.information(self, "尚未选择音乐", "请先选择本地音乐文件。")
            return
        if self.music_player.playbackState() is QMediaPlayer.PlaybackState.PlayingState:
            self.music_player.stop()
        else:
            self.music_player.setPosition(0)
            self.music_player.play()

    def update_music_button(self, state) -> None:
        playing = state is QMediaPlayer.PlaybackState.PlayingState
        self.preview_music_button.setText("停止试听" if playing else "试听")

    def update_dance_summary(self) -> None:
        action_count = sum(step.kind is PhaseKind.TRICK for step in self.dance_steps)
        posture_count = sum(step.kind is PhaseKind.POSTURE for step in self.dance_steps)
        target_count = sum(step.kind is PhaseKind.TAG_TARGET for step in self.dance_steps)
        explicit_tracks = {target for step in self.dance_steps for target in step.targets}
        track_text = (str(len(explicit_tracks)) + " 条独立轨道") if explicit_tracks else "共享轨道"
        self.dance_summary.setText(
            str(len(self.dance_steps)) + " 条时间线指令 / " + track_text
            + " / " + str(action_count) + " 个特技 / "
            + str(posture_count) + " 个起立/坐下 / "
            + str(target_count) + " 个TAG点位"
        )

    def open_dance_editor(self) -> None:
        if self.dance_timer.isActive():
            QMessageBox.information(self, "编舞正在运行", "请先停止当前编舞，再修改时间线。")
            return
        dialog = ChoreographyEditorDialog(
            list(self.dance_steps),
            self.configs,
            self.safety.max_x,
            self.safety.max_y,
            self.safety.max_yaw,
            self.non_flip_seconds.value(),
            effective_flip_lock_seconds(self.safety),
            self,
        )
        if dialog.exec() == QDialog.Accepted:
            self.dance_steps = dialog.steps()
            self.update_dance_summary()
            LOG.info("已保存自定义编舞：%d 个步骤", len(self.dance_steps))

    def save_dance(self) -> None:
        output_dir = LOCAL_ROOT / "performances"
        output_dir.mkdir(parents=True, exist_ok=True)
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "保存编舞方案",
            str(output_dir / "编舞方案.json"),
            "编舞方案 (*.json);;所有文件 (*)",
        )
        if not filename:
            return
        payload = dance_steps_to_payload(self.dance_steps)
        if self.music_path is not None:
            try:
                payload["music"] = Path(
                    os.path.relpath(self.music_path, Path(filename).resolve().parent)
                ).as_posix()
            except ValueError:  # Windows paths can reside on separate drives.
                payload["music"] = str(self.music_path)
        try:
            Path(filename).write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        except OSError as exc:
            QMessageBox.critical(self, "保存编舞失败", str(exc))
            return
        self.statusBar().showMessage(f"编舞方案已保存：{filename}", 5000)
        LOG.info("编舞方案已保存：%s", filename)

    def load_dance(self) -> None:
        if self.dance_timer.isActive():
            QMessageBox.information(self, "编舞正在运行", "请先停止当前编舞，再加载方案。")
            return
        filename, _ = QFileDialog.getOpenFileName(
            self, "加载编舞方案", str(self.dance_directory), "编舞方案 (*.json);;所有文件 (*)"
        )
        if not filename:
            return
        try:
            payload = json.loads(Path(filename).read_text(encoding="utf-8-sig"))
            steps = dance_steps_from_payload(payload)
            build_multitrack_dance(
                steps,
                effective_flip_lock_seconds(self.safety),
                [config.robot_id for config in self.configs],
                allow_overlap=True,
                post_action_correction_seconds=TRICK_POST_ACTION_CORRECTION_SECONDS,
            )
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            QMessageBox.warning(self, "加载编舞失败", str(exc))
            return
        self.dance_steps = steps
        music = payload.get("music") if isinstance(payload, dict) else None
        if music:
            music_path = Path(str(music))
            if not music_path.is_absolute():
                music_path = Path(filename).resolve().parent / music_path
            if music_path.is_file():
                self.set_music(music_path)
            else:
                LOG.warning("编舞方案中的音乐文件不存在：%s", music_path)
        self.update_dance_summary()
        self.statusBar().showMessage(f"已加载 {len(steps)} 条编舞指令", 5000)
        LOG.info("已加载编舞方案：%s（%d 条指令）", filename, len(steps))

    def start_dance(self) -> None:
        if not self.fleet:
            QMessageBox.information(self, "尚未连接", "请先连接并校准参与编舞的机器狗。")
            return
        targets = self.selected()
        if not targets:
            QMessageBox.information(self, "没有目标", "请至少选择一台机器狗。")
            return
        if self.flip_pair_timer.isActive() or self.flip_test_queue:
            self.cancel_flip_pair_test("开始编舞，剩余翻转已取消")
        steps = list(self.dance_steps)
        lock_seconds = effective_flip_lock_seconds(self.safety)
        try:
            phases = build_multitrack_dance(
                steps,
                lock_seconds,
                [config.robot_id for config in self.configs],
                allow_overlap=True,
                post_action_correction_seconds=TRICK_POST_ACTION_CORRECTION_SECONDS,
            )
        except ValueError as exc:
            QMessageBox.warning(self, "编舞参数错误", str(exc))
            return
        dance_targets = sorted({target for phase in phases for target in phase.targets})
        snapshots = {item.robot_id: item for item in self.fleet.snapshots()}
        connected = {
            target for target in dance_targets
            if target in snapshots and snapshots[target].connected
        }
        selected_ids = set(targets)
        eligible = connected & selected_ids
        skipped = sorted(set(dance_targets) - eligible)
        if skipped:
            if not eligible:
                QMessageBox.warning(
                    self,
                    "没有可执行设备",
                    "当前编舞轨道均未勾选或未连接，无法开始："
                    + "、".join(skipped),
                )
                return
            answer = QMessageBox.question(
                self,
                "跳过不可执行设备",
                "以下编舞轨道未勾选或未连接："
                + "、".join(skipped)
                + "\n\n确认后将跳过这些设备，其余已勾选且已连接的设备继续执行编舞。\n确定开始吗？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
            phases = keep_connected_dance_phases(phases, eligible)
            dance_targets = sorted(
                {target for phase in phases for target in phase.targets}
            )
            self.statusBar().showMessage(
                "已跳过未勾选或未连接设备："
                + "、".join(skipped)
                + "，其余设备继续执行编舞",
                6000,
            )
            LOG.warning("编舞自动跳过不可执行设备：%s", ", ".join(skipped))
        overlaps = overlapping_dance_cue_messages(phases)
        if overlaps:
            answer = QMessageBox.question(
                self,
                "时间重叠确认",
                "以下轨道存在同一机器狗时间重叠，确认后将按时间线放行执行"
                "（同一机器狗仍按指令到达顺序依次处理）：\n"
                + "\n".join(overlaps)
                + "\n\n确定开始编舞吗？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        has_tricks = any(phase.kind is PhaseKind.TRICK for phase in phases)
        trick_targets = sorted({
            target
            for phase in phases
            if phase.kind is PhaseKind.TRICK
            for target in phase.targets
        })
        heading_targets = {
            target for phase in phases
            if phase.kind in {
                PhaseKind.CORRECT_YAW, PhaseKind.FORWARD, PhaseKind.MOVE,
            }
            for target in phase.targets
        }
        tag_target_phases = [
            phase for phase in phases if phase.kind is PhaseKind.TAG_TARGET
        ]
        if tag_target_phases:
            if self.position_window is None or not self.position_window.reader.port.isOpen():
                QMessageBox.warning(
                    self, "TAG 串口未连接", "编舞包含 TAG 目标点，请先打开定位窗口并连接串口。"
                )
                return
            settings = self.position_window.settings
            scale = unit_to_meters(settings.unit)
            if scale is None or not settings.xy_confirmed or not settings.heading_confirmed:
                QMessageBox.warning(
                    self, "TAG 设置未确认", "请先确认定位单位、X/Y 方向和机头方向。"
                )
                return
            invalid_targets = [
                phase for phase in tag_target_phases
                if not inside_stage((phase.x / scale, phase.y / scale), settings.corners)
            ]
            if invalid_targets:
                QMessageBox.warning(self, "TAG 目标点越界", "编舞中存在场地边界外的目标点。")
                return
            now = time.monotonic()
            stale_seconds = max(1.0, float(settings.stale_seconds))
            stale_targets = sorted({
                target
                for phase in tag_target_phases
                for target in phase.targets
                if target not in self.tag_positions
                or now - self.tag_positions[target][2] > stale_seconds
            })
            if stale_targets:
                QMessageBox.warning(
                    self, "TAG 数据过期", "以下目标没有实时 TAG 位置：" + ", ".join(stale_targets)
                )
                return
        if has_tricks and not self.confirm_stunts.isChecked():
            QMessageBox.warning(
                self, "安全确认", "编舞包含特技动作，请先勾选特技安全确认。"
            )
            return
        dance_stunt_yaw_targets = self.stunt_heading_targets(
            trick_targets,
            allow_current_fallback=any(
                phase.kind is PhaseKind.TRICK and phase.return_to_start
                for phase in phases
            ),
        )
        if dance_stunt_yaw_targets is None:
            return
        missing = heading_targets - self.yaw_calibrated
        if missing:
            QMessageBox.warning(
                self,
                "尚未校准",
                "请先完成以下设备的统一朝向校准：" + ", ".join(sorted(missing)),
            )
            return
        tag_target_devices = {
            target for phase in tag_target_phases for target in phase.targets
        }
        unavailable_odometry = [
            target for target in heading_targets | tag_target_devices
            if snapshots[target].corrected_pose is None
        ]
        if unavailable_odometry:
            QMessageBox.warning(
                self,
                "里程计尚未就绪",
                "以下设备无法启用表演轨迹/进度同步："
                + ", ".join(sorted(unavailable_odometry)),
            )
            return
        self.cancel_dance(send_stop=False)
        if tag_target_phases:
            self._cancel_position_hold_ui()
        self.formation_sync.reset()
        self.motion_timer.stop()
        self.yaw_hold.blockSignals(True)
        self.yaw_hold.setChecked(False)
        self.yaw_hold.blockSignals(False)
        self.dance_targets = dance_targets
        self.dance_stunt_yaw_targets = dance_stunt_yaw_targets
        self.dance_phases = phases
        self.dance_started_cues.clear()
        self.dance_finished_cues.clear()
        self.dance_started_at = time.monotonic()
        self.dance_total_duration = max(
            (phase.start_time + phase.duration for phase in phases), default=0.0
        )
        self.dance_timer.start()
        self.start_dance_button.setEnabled(False)
        self.stop_dance_button.setEnabled(True)
        if self.sync_music.isChecked() and self.music_path is not None:
            self.music_player.setPosition(0)
            self.music_player.play()
        LOG.warning(
            "多轨编舞开始：%s；时长 %.2f s", ", ".join(self.dance_targets),
            self.dance_total_duration,
        )
        self.dance_tick()

    def start_dance_cue(self, index: int, phase: DancePhase) -> None:
        self.dance_started_cues.add(index)
        target_text = ", ".join(phase.targets)
        self.statusBar().showMessage(
            "多轨编舞 " + format(phase.start_time, ".2f") + "s：" + target_text + " / " + phase.label
        )
        LOG.info(
            "时间线 %.2fs：%s -> %s（%.2fs）",
            phase.start_time, target_text, phase.label, phase.duration,
        )
        if phase.kind is PhaseKind.TRICK:
            params = MainWindow._stunt_position_return_params(
                self,
                phase.return_to_start
            )
            if phase.command_duration > 0:
                params["duration_s"] = phase.command_duration
            for robot_id in phase.targets:
                command_params = dict(params)
                target_yaw = self.dance_stunt_yaw_targets.get(robot_id)
                if target_yaw is not None:
                    command_params["target_yaw"] = target_yaw
                self.fleet.submit([robot_id], phase.command, **command_params)
        elif phase.kind is PhaseKind.POSTURE:
            self.fleet.submit(phase.targets, phase.command)
        elif phase.kind is PhaseKind.STOP:
            self.fleet.submit(phase.targets, Command.STOP)
        elif phase.kind is PhaseKind.TAG_TARGET:
            settings = self.position_window.settings
            for robot_id in phase.targets:
                x, y, received_at = self.tag_positions[robot_id]
                self.fleet.update_tag_position(robot_id, x, y, received_at)
                self.fleet.start_tag_position_target(
                    robot_id,
                    phase.x,
                    phase.y,
                    stale_seconds=max(1.0, float(settings.stale_seconds)),
                    stage_heading_deg=float(settings.zero_heading_deg),
                    return_speed_percent=phase.speed,
                    continuous_return=bool(settings.continuous_return),
                    return_latency_ms=float(settings.return_latency_ms),
                    return_max_seconds=max(
                        float(settings.return_max_seconds),
                        float(phase.duration) + 0.5,
                    ),
                    direction_lock=str(
                        settings.direction_locks.get(robot_id, "xy")
                    ),
                )
        else:
            self.submit_dance_velocity(phase)

    def finish_dance_cue(self, index: int, phase: DancePhase) -> None:
        self.dance_finished_cues.add(index)
        if phase.kind is PhaseKind.CORRECT_YAW and hasattr(self.fleet, "snapshots"):
            current = {data.robot_id: data for data in self.fleet.snapshots()}
            unfinished = []
            for robot_id in phase.targets:
                data = current.get(robot_id)
                if data is None or data.rpy is None or time.monotonic() - data.updated_at > 0.5:
                    unfinished.append(f"{robot_id}: 无新鲜 IMU")
                    continue
                target = raw_target_for_common_yaw(math.radians(self.target_yaw.value()), self.yaw_offsets[robot_id])
                error = math.atan2(math.sin(target-data.rpy[2]), math.cos(target-data.rpy[2]))
                rate = data.corrected_yaw_rate
                if not math.isfinite(error) or abs(error) > self.safety.feedback_deadband_rad or rate is None or not math.isfinite(rate) or abs(rate) > 0.04:
                    unfinished.append(f"{robot_id}: 误差 {math.degrees(error):+.2f}°")
            if unfinished:
                message = "纠正时间结束但未确认到位：" + "；".join(unfinished)
                LOG.warning(message)
                self.statusBar().showMessage(message, 8000)
        if phase.kind in {
            PhaseKind.CORRECT_YAW,
            PhaseKind.FORWARD,
            PhaseKind.MOVE,
            PhaseKind.TAG_TARGET,
        }:
            # Trick recovery is owned by the controller. Sending STOP at the
            # trick boundary cancels the dedicated Bipedal recover_walk
            # sequence and can drop the robot into a disabled state.
            self.fleet.submit(phase.targets, Command.STOP)

    def submit_dance_velocity(self, phase: DancePhase) -> None:
        common_target = math.radians(self.target_yaw.value())
        synchronized = MainWindow.formation_velocity_targets(
            self,
            phase.targets,
            phase.x,
            phase.y,
            session_key=("dance", id(phase)),
        )
        for robot_id in phase.targets:
            synced_x, synced_y, formation_status = synchronized[robot_id]
            params = {"x": synced_x, "y": synced_y, "yaw": phase.yaw}
            params["speed_sync"] = self.speed_sync.isChecked()
            adaptive_slip = getattr(self, "adaptive_slip", None)
            params["adaptive_slip"] = bool(
                adaptive_slip is not None
                and adaptive_slip.isChecked()
                and abs(phase.yaw) < 1e-9
                and (abs(phase.x) >= 1e-9 or abs(phase.y) >= 1e-9)
            )
            if formation_status:
                params["formation_status"] = formation_status
            if (
                abs(phase.yaw) < 1e-9
                and (
                    phase.kind is PhaseKind.CORRECT_YAW
                    or abs(phase.x) >= 1e-9
                    or abs(phase.y) >= 1e-9
                )
            ):
                params["target_yaw"] = raw_target_for_common_yaw(
                    common_target, self.yaw_offsets[robot_id]
                )
                path_hold = getattr(self, "lateral_hold", None)
                params["path_correction"] = bool(
                    path_hold is not None and path_hold.isChecked()
                )
            self.fleet.submit([robot_id], Command.VELOCITY, **params)

    def dance_tick(self) -> None:
        if not self.dance_phases or not self.dance_timer.isActive():
            return
        elapsed = time.monotonic() - self.dance_started_at
        for index, phase in enumerate(self.dance_phases):
            if index not in self.dance_started_cues and elapsed >= phase.start_time:
                self.start_dance_cue(index, phase)
            if index not in self.dance_started_cues or index in self.dance_finished_cues:
                continue
            if elapsed >= phase.start_time + phase.duration:
                self.finish_dance_cue(index, phase)
            elif phase.kind in {PhaseKind.CORRECT_YAW, PhaseKind.FORWARD, PhaseKind.MOVE}:
                self.submit_dance_velocity(phase)
        if elapsed >= self.dance_total_duration and len(self.dance_finished_cues) == len(self.dance_phases):
            self.cancel_dance(send_stop=True, completed=True)

    def cancel_dance(self, checked: bool = False, send_stop: bool = True, completed: bool = False) -> None:
        was_running = self.dance_timer.isActive() if hasattr(self, "dance_timer") else False
        if hasattr(self, "dance_timer"):
            self.dance_timer.stop()
        if send_stop and self.fleet and self.dance_targets:
            self.fleet.submit(self.dance_targets, Command.STOP)
        if was_running:
            self.music_player.stop()
        self.dance_phases = ()
        self.dance_started_cues.clear()
        self.dance_finished_cues.clear()
        self.dance_started_at = 0.0
        self.dance_total_duration = 0.0
        self.dance_targets = []
        self.dance_stunt_yaw_targets = {}
        synchronizer = getattr(self, "formation_sync", None)
        if synchronizer is not None:
            synchronizer.reset()
        if hasattr(self, "start_dance_button"):
            self.start_dance_button.setEnabled(True)
            self.stop_dance_button.setEnabled(False)
        if was_running:
            message = "编舞完成并停止" if completed else "编舞已由操作员停止"
            LOG.warning(message)
            self.statusBar().showMessage(message, 5000)

    def stop_motion(self) -> None:
        if self.dance_timer.isActive():
            self.cancel_dance(send_stop=False)
        if self.flip_pair_timer.isActive() or self.flip_test_queue:
            self.cancel_flip_pair_test("停止指令已触发，剩余翻转已取消")
        manual_turn_finished = getattr(self, "manual_turn_active", False)
        self.manual_turn_active = False
        self.active_velocity = (0.0, 0.0, 0.0)
        self.formation_sync.reset("manual")
        if self.yaw_hold.isChecked():
            if manual_turn_finished and not self.adopt_current_heading_target():
                self.motion_timer.stop()
                self.send(Command.STOP)
                return
            self.repeat_motion()
            self.motion_timer.start()
        else:
            self.motion_timer.stop()
            self.send(Command.STOP)

    def adopt_current_heading_target(self) -> bool:
        """Use the result of a manual turn as the new yaw-hold target."""
        if not self.fleet:
            return False
        snapshots = {data.robot_id: data for data in self.fleet.snapshots()}
        common_headings = []
        for robot_id in self.selected():
            data = snapshots.get(robot_id)
            if data is None or not data.connected or data.rpy is None:
                continue
            common_headings.append(
                common_yaw_from_raw(data.rpy[2], self.yaw_offsets[robot_id])
            )
        if not common_headings:
            return False
        sine = sum(math.sin(value) for value in common_headings)
        cosine = sum(math.cos(value) for value in common_headings)
        if abs(sine) < 1e-9 and abs(cosine) < 1e-9:
            return False
        common_target = math.atan2(sine, cosine)
        self.target_yaw.setValue(math.degrees(common_target))
        LOG.info(
            "人工转向结束，新统一目标 Yaw=%.1f°（%d 台有效设备）",
            math.degrees(common_target),
            len(common_headings),
        )
        return True

    def yaw_hold_changed(self, enabled: bool) -> None:
        if enabled:
            if not self.fleet:
                self.statusBar().showMessage("请先连接设备，再启用 Yaw PID", 3000)
                self.yaw_hold.blockSignals(True)
                self.yaw_hold.setChecked(False)
                self.yaw_hold.blockSignals(False)
                return
            missing = set(self.selected()) - self.yaw_calibrated
            if missing:
                QMessageBox.warning(
                    self,
                    "尚未统一校准",
                    "请先让机器狗物理朝向一致，再点击“校准统一朝向”。",
                )
                self.yaw_hold.blockSignals(True)
                self.yaw_hold.setChecked(False)
                self.yaw_hold.blockSignals(False)
                return
            self.active_velocity = (0.0, 0.0, 0.0)
            self.repeat_motion()
            self.motion_timer.start()
            LOG.info("启用指定 Yaw PID，目标 %.1f°", self.target_yaw.value())
        else:
            self.motion_timer.stop()
            self.send(Command.STOP)
            LOG.info("停止指定 Yaw PID")

    def lateral_hold_changed(self, enabled: bool) -> None:
        if enabled:
            if not self.fleet:
                self.statusBar().showMessage("请先连接设备，再启用轨迹纠偏", 3000)
                self.lateral_hold.blockSignals(True)
                self.lateral_hold.setChecked(False)
                self.lateral_hold.blockSignals(False)
                return
            snapshots = {data.robot_id: data for data in self.fleet.snapshots()}
            unavailable = [
                robot_id
                for robot_id in self.selected()
                if robot_id not in snapshots
                or not snapshots[robot_id].connected
                or snapshots[robot_id].corrected_pose is None
            ]
            if unavailable:
                QMessageBox.warning(
                    self,
                    "里程计尚未就绪",
                    "以下设备没有有效融合里程计：" + ", ".join(unavailable),
                )
                self.lateral_hold.blockSignals(True)
                self.lateral_hold.setChecked(False)
                self.lateral_hold.blockSignals(False)
                return
            LOG.info("启用二维直线轨迹纠偏：%s", ", ".join(self.selected()))
            self.statusBar().showMessage("二维直线轨迹纠偏已启用", 3000)
        else:
            if self.fleet and self.motion_timer.isActive():
                self.repeat_motion()
            LOG.info("停止二维直线轨迹纠偏")

    def position_hold_changed(self, enabled: bool) -> None:
        if not enabled:
            self._cancel_position_hold_ui()
            LOG.info("取消标签零位保持 / 特技后自动回位")
            return
        if not self.fleet:
            self.statusBar().showMessage("请先连接设备，再启用标签零位回位", 3000)
            self._set_position_hold_checked(False)
            return
        if (
            self.position_window is None
            or not self.position_window.reader.port.isOpen()
        ):
            QMessageBox.warning(
                self,
                "无法启用标签回位",
                "TAG 定位串口当前未连接，请打开定位窗口并连接串口。",
            )
            self._set_position_hold_checked(False)
            return
        targets = tuple(self.selected())
        if not targets:
            self.statusBar().showMessage("请至少选择一台机器狗", 3000)
            self._set_position_hold_checked(False)
            return
        position_settings = self.position_window.settings
        self.tag_return_speed_percent = float(
            position_settings.return_speed_percent
        )
        self.tag_continuous_return = bool(position_settings.continuous_return)
        self.tag_return_latency_ms = float(position_settings.return_latency_ms)
        self.tag_return_max_seconds = float(position_settings.return_max_seconds)
        if not position_settings.xy_confirmed or not position_settings.heading_confirmed:
            QMessageBox.warning(
                self,
                "无法启用标签回位",
                "请先在 TAG 定位窗口确认 X/Y 和“标零时机器狗机头朝向”，应用设置后重新逐台标零。",
            )
            self._set_position_hold_checked(False)
            return
        snapshots = {item.robot_id: item for item in self.fleet.snapshots()}
        now = time.monotonic()
        stale_limit = max(1.0, self.tag_stale_seconds)
        missing = []
        for robot_id in targets:
            snapshot = snapshots.get(robot_id)
            if snapshot is None or not snapshot.connected:
                missing.append(robot_id + "（未连接）")
            elif snapshot.corrected_pose is None:
                missing.append(robot_id + "（无有效航向）")
            elif robot_id not in self.tag_origins:
                missing.append(robot_id + "（未标定标签零位）")
            elif robot_id not in self.tag_headings:
                missing.append(robot_id + "（零位缺少舞台机头方向，请重新标零）")
            elif abs(
                ((self.tag_headings[robot_id] - position_settings.zero_heading_deg + 180.0) % 360.0)
                - 180.0
            ) > 1e-6:
                missing.append(robot_id + "（机头方向设置已改变，请重新标零）")
            elif (
                robot_id not in self.tag_positions
                or now - self.tag_positions[robot_id][2] > stale_limit
            ):
                missing.append(robot_id + "（标签位置过期）")
        if missing:
            QMessageBox.warning(
                self,
                "无法启用标签回位",
                "请打开 TAG 定位窗口、连接串口，并逐台点击“标定为零”：\n"
                + "\n".join(missing),
            )
            self._set_position_hold_checked(False)
            return

        self.motion_timer.stop()
        self.active_velocity = (0.0, 0.0, 0.0)
        self.manual_turn_active = False
        self.formation_sync.reset("tag-hold")
        self.yaw_hold.blockSignals(True)
        self.yaw_hold.setChecked(False)
        self.yaw_hold.blockSignals(False)
        self.lateral_hold.blockSignals(True)
        self.lateral_hold.setChecked(False)
        self.lateral_hold.blockSignals(False)
        self.tag_hold_targets = targets
        for robot_id in targets:
            origin_x, origin_y = self.tag_origins[robot_id]
            x, y, received_at = self.tag_positions[robot_id]
            self.fleet.update_tag_position(robot_id, x, y, received_at)
            self.fleet.start_tag_position_hold(
                robot_id,
                origin_x=origin_x,
                origin_y=origin_y,
                stale_seconds=self.tag_stale_seconds,
                stage_heading_deg=self.tag_headings[robot_id],
                return_speed_percent=self.tag_return_speed_percent,
                continuous_return=self.tag_continuous_return,
                return_latency_ms=self.tag_return_latency_ms,
                return_max_seconds=self.tag_return_max_seconds,
                direction_lock=self.tag_direction_locks.get(robot_id, "xy"),
            )
        self.statusBar().showMessage(
            "标签零位保持已启用（含特技后自动回位）：" + ", ".join(targets),
            6000,
        )
        LOG.info("启用标签零位保持：%s", ", ".join(targets))

    def _set_position_hold_checked(self, checked: bool) -> None:
        self.position_hold.blockSignals(True)
        self.position_hold.setChecked(checked)
        self.position_hold.blockSignals(False)

    def _cancel_position_hold_ui(self) -> None:
        targets = self.tag_hold_targets
        self.tag_hold_targets = ()
        if self.fleet:
            for robot_id in targets:
                self.fleet.stop_tag_position_hold(robot_id)
        self._set_position_hold_checked(False)

    def disable_yaw_hold_and_stop(self, cancel_tag_hold: bool = True) -> None:
        self.yaw_hold.blockSignals(True)
        self.yaw_hold.setChecked(False)
        self.yaw_hold.blockSignals(False)
        self.lateral_hold.blockSignals(True)
        self.lateral_hold.setChecked(False)
        self.lateral_hold.blockSignals(False)
        if cancel_tag_hold:
            self._cancel_position_hold_ui()
        self.motion_timer.stop()
        self.manual_turn_active = False
        self.active_velocity = (0.0, 0.0, 0.0)
        if cancel_tag_hold or not self.position_hold.isChecked():
            self.send(Command.STOP)

    def emergency_stop(self) -> None:
        self.cancel_dance(send_stop=False)
        self.cancel_flip_pair_test("急停已触发，剩余翻转已取消")
        self.motion_timer.stop()
        self.manual_turn_active = False
        self.active_velocity = (0.0, 0.0, 0.0)
        self.yaw_hold.blockSignals(True)
        self.yaw_hold.setChecked(False)
        self.yaw_hold.blockSignals(False)
        self.lateral_hold.blockSignals(True)
        self.lateral_hold.setChecked(False)
        self.lateral_hold.blockSignals(False)
        self._cancel_position_hold_ui()
        if self.fleet:
            self.fleet.emergency_stop()
            LOG.warning("操作员触发全体急停/阻尼")

    def diagnostics_recording_changed(self, enabled: bool) -> None:
        if self.diagnostic_recorder is not None:
            self.diagnostic_recorder.close()
            self.diagnostic_recorder = None
        if not enabled:
            self.statusBar().showMessage("控制诊断记录已停止", 3000)
            return
        try:
            recorder = StraightLineCsvRecorder(
                LOCAL_ROOT / "diagnostics"
            )
        except OSError as exc:
            self.record_diagnostics.blockSignals(True)
            self.record_diagnostics.setChecked(False)
            self.record_diagnostics.blockSignals(False)
            QMessageBox.critical(self, "无法创建诊断文件", str(exc))
            return
        self.diagnostic_recorder = recorder
        LOG.info("开始记录控制诊断：%s", recorder.path)
        self.statusBar().showMessage(f"正在记录：{recorder.path.name}", 5000)

    def refresh(self) -> None:
        if not self.fleet:
            return
        snapshots = self.fleet.snapshots()
        connected_ids = {data.robot_id for data in snapshots if data.connected}
        for robot_id in connected_ids - self._auto_checked_connected:
            box = self.targets.get(robot_id)
            if box is not None:
                box.setChecked(True)
        self._auto_checked_connected = connected_ids
        self.report_connections(snapshots)
        offline = {data.robot_id for data in snapshots if not data.connected}
        if offline.intersection(self.dance_targets) and self.dance_timer.isActive():
            LOG.warning("编舞目标断联，已取消编舞；重连后需重新启动")
            self.cancel_dance(send_stop=True)
        if offline.intersection(self.flip_pair_targets) and self.flip_pair_timer.isActive():
            self.cancel_flip_pair_test("目标断联，已取消剩余特技")
        if self.diagnostic_recorder is not None:
            try:
                self.diagnostic_recorder.record(snapshots)
            except OSError as exc:
                LOG.error("写入控制诊断失败：%s", exc)
                self.record_diagnostics.setChecked(False)
        for data in snapshots:
            self.status_table.update_data(data)
        window = self.odometry_window
        if window is None:
            return
        robot_id = self.current_odometry_robot()
        selected = next((data for data in snapshots if data.robot_id == robot_id), None)
        if selected is None or selected.corrected_pose is None:
            window.odometry_pose.setText("x=-- m    y=-- m    yaw=--°")
            window.odometry_fusion.setText("EKF：等待连接和传感器数据")
            window.odometry_lateral_status.setText("轨迹纠偏：等待里程计数据")
            return
        x, y, yaw = selected.corrected_pose
        window.odometry_pose.setText(f"x={x:.3f} m    y={y:.3f} m    yaw={yaw:.2f}°")
        if selected.odom_uncertainty is None:
            window.odometry_fusion.setText("EKF：等待不确定度")
        else:
            sigma_x, sigma_y, sigma_yaw = selected.odom_uncertainty
            velocity_text = ""
            if selected.corrected_velocity_world is not None:
                vx_world, vy_world = selected.corrected_velocity_world
                velocity_text = f"；速度=({vx_world:+.3f},{vy_world:+.3f})m/s"
            window.odometry_fusion.setText(
                f"EKF 1σ：x±{sigma_x:.3f} m，y±{sigma_y:.3f} m，"
                f"yaw±{sigma_yaw:.2f}°{velocity_text}；"
                f"当前采用 {selected.odom_support_legs} 条支撑腿约束"
            )
            window.odometry_lateral_status.setText("控制反馈：" + str(selected.feedback))

    def mode_changed(self) -> None:
        self.confirm_real.setEnabled(self.mode.currentData() == "real")

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.isAutoRepeat():
            return
        mapping = {
            Qt.Key_W: (1, 0, 0), Qt.Key_S: (-1, 0, 0),
            Qt.Key_A: (0, 1, 0), Qt.Key_D: (0, -1, 0),
            Qt.Key_Q: (0, 0, 1), Qt.Key_E: (0, 0, -1),
        }
        if event.key() == Qt.Key_Space:
            self.emergency_stop()
        elif event.key() in mapping:
            self.start_motion(mapping[event.key()])
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        if not event.isAutoRepeat() and event.key() in {
            Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D, Qt.Key_Q, Qt.Key_E
        }:
            self.stop_motion()
        else:
            super().keyReleaseEvent(event)

    def focusOutEvent(self, event) -> None:
        self.disable_yaw_hold_and_stop(cancel_tag_hold=False)
        super().focusOutEvent(event)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.network_is_busy():
            self.network_panel.auto.setChecked(False)
            self.statusBar().showMessage("网络检测/恢复尚未完成，请等待完成后关闭界面", 5000)
            event.ignore()
            return
        # Stop scheduled and manual motion before opening a modal confirmation;
        # otherwise a choreography or flip timer could fire while the operator
        # is deciding whether to close the application.
        self.cancel_dance(send_stop=True)
        self.cancel_flip_pair_test("关闭请求已暂停全部定时动作")
        self._cancel_position_hold_ui()
        self.motion_timer.stop()
        self.active_velocity = (0.0, 0.0, 0.0)
        if self.voice_worker is not None:
            self.voice_worker.cancel()
            self.voice_worker = None
        if self.voice_executor is not None:
            self.voice_executor.stop()
            self.voice_executor = None
        if self.fleet:
            self.fleet.submit(
                [config.robot_id for config in self.configs], Command.STOP
            )
        answer = QMessageBox.question(
            self,
            "退出确认",
            "确定关闭整个控制界面吗？未保存的操作将停止，断开前已连接设备会坐下。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            event.ignore()
            return
        self.music_player.stop()
        self.refresh_timer.stop()
        self.network_panel.timer.stop()
        if self.diagnostic_recorder is not None:
            self.diagnostic_recorder.close()
            self.diagnostic_recorder = None
        logging.getLogger().removeHandler(self.log_handler)
        if self.file_log_handler is not None:
            logging.getLogger().removeHandler(self.file_log_handler)
            self.file_log_handler.close()
            self.file_log_handler = None
        if self.position_window is not None:
            self.position_window.close()
            self.position_window = None
        if self.odometry_window is not None:
            self.odometry_window.close()
            self.odometry_window = None
        if self.fleet:
            self.fleet.close(timeout=5.0, final_posture=Command.SIT)
        event.accept()
