"""Independent stage-map window. Closing it releases only the positioning port."""
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import time

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QPushButton,
    QScrollArea, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from dual_dog.devices.position_serial import PositionSerial
from dual_dog.devices.positioning import (
    MIN_MULTI_TAG_STALE_SECONDS,
    SUPPORTED_TAGS,
    WAYPOINT_TAGS,
    PositionSettings,
    TagTrack,
    inside_stage,
    control_stale_seconds,
    unit_to_meters,
)


TAG_COLORS = dict(zip(SUPPORTED_TAGS, (
    "#287acc", "#d87a1a", "#198754", "#9b59b6", "#c0392b",
    "#008b8b", "#7f6000", "#e84393", "#5b6b7a", "#6c5ce7",
)))


class StageMap(QWidget):
    def __init__(self, owner):
        super().__init__(owner)
        self.owner = owner
        self.setMinimumSize(420, 320)

    def _geometry(self):
        settings = self.owner.settings
        xs, ys = zip(*settings.corners)
        xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
        area = QRectF(self.rect()).adjusted(65, 60, -55, -55)
        scale = min(area.width() / (xmax - xmin), area.height() / (ymax - ymin))
        width, height = (xmax - xmin) * scale, (ymax - ymin) * scale
        left, bottom = area.center().x() - width / 2, area.center().y() + height / 2
        plot = QRectF(left, bottom - height, width, height)
        return xmin, xmax, ymin, ymax, scale, left, bottom, plot

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.LeftButton or not self.owner.map_target_edit.isChecked():
            super().mousePressEvent(event)
            return
        xmin, xmax, ymin, ymax, scale, left, bottom, plot = self._geometry()
        point = event.position()
        if not plot.contains(point):
            self.owner.settings_status.setText("目标点必须点击在场地地图内部。")
            return
        stage_point = (
            xmin + (point.x() - left) / scale,
            ymin + (bottom - point.y()) / scale,
        )
        if not inside_stage(stage_point, self.owner.settings.corners):
            self.owner.settings_status.setText("目标点位于场地四角边界外。")
            return
        self.owner.set_target_point(self.owner.target_tag_choice.currentData(), stage_point)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f8fafc"))
        settings = self.owner.settings
        xmin, xmax, ymin, ymax, scale, left, bottom, plot = self._geometry()
        width, height = plot.width(), plot.height()

        def screen(point):
            return QPointF(left + (point[0] - xmin) * scale, bottom - (point[1] - ymin) * scale)

        painter.setPen(QColor("#607085"))
        title = "场地实时位置" if settings.xy_confirmed else "原始数值预览（X/Y 协议尚未确认）"
        if not settings.stage_configured:
            title += " · 示例边界，待设置四角"
        painter.drawText(16, 25, title)
        painter.setPen(QPen(QColor("#e1e8f0"), 1))
        for i in range(6):
            x = left + width * i / 5
            y = bottom - height * i / 5
            painter.drawLine(QPointF(x, bottom - height), QPointF(x, bottom))
            painter.drawLine(QPointF(left, y), QPointF(left + width, y))
        painter.setPen(QColor("#607085"))
        for i in range(6):
            painter.drawText(QRectF(left + width * i / 5 - 30, bottom + 8, 60, 20), Qt.AlignCenter, f"{xmin + (xmax - xmin) * i / 5:g}")
            painter.drawText(QRectF(left - 62, bottom - height * i / 5 - 10, 55, 20), Qt.AlignRight | Qt.AlignVCenter, f"{ymin + (ymax - ymin) * i / 5:g}")
        painter.drawText(int(left + width - 90), int(bottom + 44), f"X / {settings.unit_label}")
        painter.drawText(int(left), int(bottom - height - 25), f"Y / {settings.unit_label}")
        painter.setPen(QPen(QColor("#6c8d79"), 2))
        painter.setBrush(QColor(110, 175, 135, 22))
        painter.drawPolygon(QPolygonF([screen(p) for p in settings.corners]))
        painter.setBrush(Qt.NoBrush)
        for label, corner in zip("ABCD", settings.corners):
            pt = screen(corner)
            painter.drawEllipse(pt, 4, 4)
            painter.drawText(pt + QPointF(7, -8), label)

        for tag in WAYPOINT_TAGS:
            target = self.owner.target_points.get(tag)
            if target is None:
                continue
            point = screen(target)
            color = QColor(TAG_COLORS[tag])
            current = self.owner.tracks[tag].kalman.position
            if current is not None:
                painter.setPen(QPen(color, 2, Qt.DashLine))
                painter.drawLine(screen(current), point)
            painter.setPen(QPen(color, 2, Qt.DashLine))
            painter.setBrush(QColor(color.red(), color.green(), color.blue(), 35))
            painter.drawEllipse(point, 10, 10)
            painter.drawLine(point + QPointF(-14, 0), point + QPointF(14, 0))
            painter.drawLine(point + QPointF(0, -14), point + QPointF(0, 14))
            painter.drawText(point + QPointF(12, -12), f"{tag}目标")

        now = time.monotonic()
        for tag, track in self.owner.tracks.items():
            position = track.kalman.position
            if position is None:
                continue
            stale = track.stale(now) or not self.owner.reader.port.isOpen()
            color = QColor("#929ba8" if stale else TAG_COLORS[tag])
            painter.save()
            painter.setClipRect(plot.adjusted(-1, -1, 1, 1))
            if self.owner.show_trails.isChecked() and len(track.history) > 1:
                trail_color = QColor(color)
                trail_color.setAlpha(100)
                painter.setPen(QPen(trail_color, 2))
                painter.drawPolyline(QPolygonF([screen(p) for p in track.history]))
            if self.owner.show_raw.isChecked() and track.raw:
                point = screen(track.raw)
                painter.setPen(QPen(QColor("#a2aab5"), 2))
                painter.drawLine(point + QPointF(-5, -5), point + QPointF(5, 5))
                painter.drawLine(point + QPointF(-5, 5), point + QPointF(5, -5))
            painter.restore()
            point = screen(position)
            offscreen = not plot.contains(point)
            point = QPointF(max(plot.left(), min(plot.right(), point.x())), max(plot.top(), min(plot.bottom(), point.y())))
            painter.setPen(QPen(color, 2))
            painter.setBrush(Qt.NoBrush if stale else color)
            painter.drawEllipse(point, 7, 7)
            painter.setBrush(Qt.NoBrush)
            label = self.owner.tag_label(tag)
            if offscreen:
                label += "（场外，点位贴边提示）"
            elif stale:
                label += "（过期/断开）"
            index = SUPPORTED_TAGS.index(tag)
            place_right = index % 2 == 0
            label_x = point.x() + 12 if place_right else point.x() - 215
            label_x = max(8, min(label_x, self.width() - 210))
            label_y = point.y() + (10 if index % 4 in {1, 2} else -31)
            painter.drawText(QRectF(label_x, label_y, 205, 40), Qt.TextWordWrap, label)
        painter.end()


class PositionWindow(QWidget):
    sample_processed = Signal(object)
    zero_calibrated = Signal(object)
    target_move_requested = Signal(object)
    target_stop_requested = Signal()
    target_choreography_requested = Signal(object)
    closed = Signal()

    def __init__(self, configs, config_path: Path):
        super().__init__(None)
        self.config_path = Path(config_path)
        self.configs = configs
        self.robot_names = {c.robot_id: c.name for c in configs}
        load_error = ""
        try:
            self.settings = PositionSettings.load(self.config_path)
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
            self.settings = PositionSettings()
            load_error = f"定位配置未载入，使用未标定默认值：{exc}"
        self.tracks = {tag: TagTrack(tag, self.settings) for tag in SUPPORTED_TAGS}
        self.target_points = dict(self.settings.tag_targets)
        self.reader = PositionSerial(self)
        self.setWindowTitle("集群实时位置 · TAG1～TAG10")
        self.resize(1380, 900)
        self.setMinimumSize(1000, 660)
        layout = QVBoxLayout(self)
        warning = QLabel("定位默认只显示；TAG1～TAG10 默认对应 dog01～dog10。自动回位前必须确认单位、X/Y 以及标零时机头在舞台坐标中的朝向；否则 TAG 误差无法正确换算成前进/横移。滤波不能消除长期系统偏差。")
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#996323; padding:6px; background:#fff8e9;")
        layout.addWidget(warning)
        connection_row = QHBoxLayout()
        self.port_choice = QComboBox()
        self.port_choice.setMinimumWidth(235)
        self.refresh_ports_button = QPushButton("刷新端口")
        self.connect_button = QPushButton("连接定位串口")
        self.connect_button.setProperty("role", "primary")
        self.disconnect_button = QPushButton("断开")
        self.disconnect_button.setEnabled(False)
        connection_row.addWidget(QLabel("115200 / 8N1"))
        connection_row.addWidget(self.port_choice, 1)
        connection_row.addWidget(self.refresh_ports_button)
        connection_row.addWidget(self.connect_button)
        connection_row.addWidget(self.disconnect_button)
        layout.addLayout(connection_row)
        self.connection_status = QLabel(load_error or "未连接；自动识别唯一 USB 串口，多个串口时请手动选择。")
        self.connection_status.setWordWrap(True)
        layout.addWidget(self.connection_status)

        splitter = QSplitter(Qt.Horizontal)
        settings_content = QWidget()
        settings_layout = QVBoxLayout(settings_content)
        settings_layout.setContentsMargins(4, 4, 8, 4)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(settings_content)
        settings_panel = QWidget()
        panel_layout = QVBoxLayout(settings_panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        panel_layout.addWidget(scroll, 1)
        settings_panel.setMinimumWidth(380)
        splitter.addWidget(settings_panel)
        corners_box = QGroupBox("1. 四角坐标（沿边界 A→B→C→D）")
        corners_grid = QGridLayout(corners_box)
        self.unit_choice = QComboBox()
        for label, value in (("原始单位（未确认）", "raw"), ("厘米 cm", "cm"), ("米 m", "m"), ("毫米 mm", "mm")):
            self.unit_choice.addItem(label, value)
        self.unit_choice.setCurrentIndex(self.unit_choice.findData(self.settings.unit))
        corners_grid.addWidget(QLabel("数据单位"), 0, 0)
        corners_grid.addWidget(self.unit_choice, 0, 1, 1, 2)
        corners_grid.addWidget(QLabel("角点"), 1, 0)
        corners_grid.addWidget(QLabel("X"), 1, 1)
        corners_grid.addWidget(QLabel("Y"), 1, 2)
        self.corner_inputs = []
        for row, (name, point) in enumerate(zip("ABCD", self.settings.corners), start=2):
            editors = [self.spin(-1e7, 1e7, v, 3) for v in point]
            self.corner_inputs.append(editors)
            corners_grid.addWidget(QLabel(name), row, 0)
            for column, editor in enumerate(editors, start=1):
                corners_grid.addWidget(editor, row, column)
        self.xy_confirm = QCheckBox("已确认 X/Y，且与四角同坐标系")
        self.xy_confirm.setChecked(self.settings.xy_confirmed)
        corners_grid.addWidget(self.xy_confirm, 6, 0, 1, 3)
        self.zero_heading_choice = QComboBox()
        for label, degrees in (
            ("+X（地图向右）", 0.0),
            ("+Y（地图向上）", 90.0),
            ("-X（地图向左）", 180.0),
            ("-Y（地图向下）", -90.0),
        ):
            self.zero_heading_choice.addItem(label, degrees)
        heading_index = self.zero_heading_choice.findData(
            float(self.settings.zero_heading_deg)
        )
        self.zero_heading_choice.setCurrentIndex(max(0, heading_index))
        corners_grid.addWidget(QLabel("标零时机器狗机头朝向"), 7, 0)
        corners_grid.addWidget(self.zero_heading_choice, 7, 1, 1, 2)
        self.heading_confirm = QCheckBox(
            "已确认机头方向（这是 TAG 坐标转换成前进/横移的依据）"
        )
        self.heading_confirm.setChecked(self.settings.heading_confirmed)
        corners_grid.addWidget(self.heading_confirm, 8, 0, 1, 3)
        hint = QLabel("四角定义显示边界，不做基站解算或坐标校准。默认四角是示例。若数据其实是距离，请勿勾选 X/Y 确认。")
        hint.setWordWrap(True)
        corners_grid.addWidget(hint, 9, 0, 1, 3)
        settings_layout.addWidget(corners_box)

        binding_box = QGroupBox("2. 标签绑定机器狗")
        binding_grid = QGridLayout(binding_box)
        self.binding_choices = {}
        self.calibrate_zero_buttons = {}
        for row, tag in enumerate(self.tracks):
            choice = QComboBox()
            choice.addItem("未绑定（只显示标签）", "")
            for config in configs:
                choice.addItem(f"{config.name} ({config.robot_id})", config.robot_id)
            robot_id = self.settings.bindings[tag]
            if robot_id and robot_id not in self.robot_names:
                choice.addItem(f"{robot_id}（配置中不存在）", robot_id)
            choice.setCurrentIndex(max(0, choice.findData(robot_id)))
            self.binding_choices[tag] = choice
            calibrate = QPushButton("标定为零")
            calibrate.clicked.connect(
                lambda checked=False, tag=tag: self.calibrate_tag_zero(tag)
            )
            self.calibrate_zero_buttons[tag] = calibrate
            binding_grid.addWidget(QLabel(tag), row, 0)
            binding_grid.addWidget(choice, row, 1)
            binding_grid.addWidget(calibrate, row, 2)
        settings_layout.addWidget(binding_box)

        direction_box = QGroupBox("2b. 回位纠偏方向（每台狗独立）")
        direction_grid = QGridLayout(direction_box)
        direction_grid.addWidget(QLabel("狗"), 0, 0)
        direction_grid.addWidget(QLabel("标零后允许纠偏的方向"), 0, 1)
        self.direction_lock_choices = {}
        for row, config in enumerate(configs, start=1):
            choice = QComboBox()
            choice.addItem("X+Y 都允许", "xy")
            choice.addItem("仅 X（舞台左右/Y 保持）", "x")
            choice.addItem("仅 Y（舞台上下/X 保持）", "y")
            saved = self.settings.direction_locks.get(config.robot_id, "xy")
            choice.setCurrentIndex(max(0, choice.findData(saved)))
            self.direction_lock_choices[config.robot_id] = choice
            direction_grid.addWidget(
                QLabel(f"{config.name} ({config.robot_id})"), row, 0
            )
            direction_grid.addWidget(choice, row, 1)
        settings_layout.addWidget(direction_box)

        filter_box = QGroupBox("3. 卡尔曼滤波（每标签独立）")
        filter_grid = QGridLayout(filter_box)
        self.filter_mode_choice = QComboBox()
        self.filter_mode_choice.addItem("稳定优先（中值 + 卡尔曼）", "stable")
        self.filter_mode_choice.addItem("跟随优先（原匀速卡尔曼）", "tracking")
        self.filter_mode_choice.setCurrentIndex(self.filter_mode_choice.findData(self.settings.filter_mode))
        filter_grid.addWidget(self.filter_mode_choice, 0, 0, 1, 2)
        self.measurement_noise = self.spin(0.0001, 100000, self.settings.measurement_sigma, 4)
        self.acceleration_noise = self.spin(0.0001, 100000, self.settings.acceleration_sigma, 4)
        self.gate = self.spin(2, 10, self.settings.gate_sigma, 1)
        self.stale_timeout = self.spin(
            MIN_MULTI_TAG_STALE_SECONDS, 30, self.settings.stale_seconds, 1
        )
        for row, (label, widget) in enumerate((("测量噪声 σ（坐标单位）", self.measurement_noise), ("过程噪声强度", self.acceleration_noise), ("异常跳点门限（σ）", self.gate), ("过期时间（秒）", self.stale_timeout)), start=1):
            filter_grid.addWidget(QLabel(label), row, 0)
            filter_grid.addWidget(widget, row, 1)
        filter_hint = QLabel("稳定优先：先取三帧中值，不做速度外推，连续变化时加快跟随，约有 1～3 帧或更长的延迟。跟随优先保留原算法。滤波不能消除持续偏差；过期后冻结。")
        filter_hint.setWordWrap(True)
        filter_grid.addWidget(filter_hint, 5, 0, 1, 2)
        settings_layout.addWidget(filter_box)

        return_box = QGroupBox("4. 标签自动回位")
        return_grid = QGridLayout(return_box)
        self.return_speed = QDoubleSpinBox()
        self.return_speed.setRange(70.0, 200.0)
        self.return_speed.setDecimals(0)
        self.return_speed.setSingleStep(10.0)
        self.return_speed.setValue(float(self.settings.return_speed_percent))
        self.return_speed.setSuffix(" %")
        self.return_speed.setToolTip(
            "100% 是标准速度；提高后仍受 robots.json 全局速度上限、TAG 超时和距离保护"
        )
        return_grid.addWidget(QLabel("回位速度"), 0, 0)
        return_grid.addWidget(self.return_speed, 0, 1)
        self.continuous_return = QCheckBox("连续平滑回位（推荐）")
        self.continuous_return.setChecked(self.settings.continuous_return)
        self.continuous_return.setToolTip(
            "位置基准始终是 UWB；连续模式仅在短时间隔内平滑推算移动，UWB 过期后必须停车等待新帧"
        )
        return_grid.addWidget(self.continuous_return, 1, 0, 1, 2)
        self.return_latency = QDoubleSpinBox()
        self.return_latency.setRange(50.0, 6000.0)
        self.return_latency.setDecimals(0)
        self.return_latency.setSingleStep(50.0)
        self.return_latency.setValue(float(self.settings.return_latency_ms))
        self.return_latency.setSuffix(" ms")
        self.return_latency.setToolTip(
            "UWB 从真实运动到位置出现在程序中的估计总延迟；秒级延迟下固定零位使用分段保护，地图目标可用里程计连续导航"
        )
        return_grid.addWidget(QLabel("估计 UWB 反馈延迟"), 2, 0)
        return_grid.addWidget(self.return_latency, 2, 1)
        self.return_timeout = QDoubleSpinBox()
        self.return_timeout.setRange(0.5, 4.0)
        self.return_timeout.setDecimals(1)
        self.return_timeout.setSingleStep(0.5)
        self.return_timeout.setValue(float(self.settings.return_max_seconds))
        self.return_timeout.setSuffix(" s")
        self.return_timeout.setToolTip(
            "特技后的 UWB 回位默认 4 秒内完成；超时仍未回到零位时停车并提示，可在 0.5～4 秒内调整"
        )
        return_grid.addWidget(QLabel("单次回位最长时间"), 3, 0)
        return_grid.addWidget(self.return_timeout, 3, 1)
        return_hint = QLabel(
            "回位只以 UWB 绝对位置判断误差；前后/左右/旋转纠偏速度默认不低于 0.2。"
            "单次回位在 4 s 内完成；超时停车并提示。"
        )
        return_hint.setWordWrap(True)
        return_grid.addWidget(return_hint, 4, 0, 1, 2)
        settings_layout.addWidget(return_box)

        target_box = QGroupBox("5. TAG1～TAG10 地图目标点 / 编舞点位")
        target_grid = QGridLayout(target_box)
        self.target_tag_choice = QComboBox()
        for tag in WAYPOINT_TAGS:
            robot_id = self.settings.bindings.get(tag, "")
            self.target_tag_choice.addItem(
                f"{tag} / {self.robot_names.get(robot_id, robot_id or '未绑定')}", tag
            )
        self.target_x = self.spin(-100000, 100000, 0.0, 3)
        self.target_y = self.spin(-100000, 100000, 0.0, 3)
        self.target_x.setSuffix(f" {self.settings.unit_label}")
        self.target_y.setSuffix(f" {self.settings.unit_label}")
        self.map_target_edit = QCheckBox("允许点击右侧地图设置当前 TAG 的目标点")
        self.target_duration = self.spin(1.0, 60.0, 12.0, 1)
        self.target_duration.setSuffix(" s")
        self.set_target_button = QPushButton("使用输入坐标标点")
        self.clear_target_button = QPushButton("清除当前点")
        self.save_targets_button = QPushButton("保存当前工作点位")
        self.snapshot_choice = QComboBox()
        self.snapshot_name = QLineEdit()
        self.snapshot_name.setPlaceholderText("例：进场队形 / 退场队形")
        self.save_snapshot_button = QPushButton("保存为点位快照")
        self.load_snapshot_button = QPushButton("载入快照")
        self.delete_snapshot_button = QPushButton("删除快照")
        self.move_one_target_button = QPushButton("现场试走当前 TAG")
        self.move_all_targets_button = QPushButton("现场试走当前快照（1～10）")
        self.stop_target_button = QPushButton("停止现场试走")
        self.add_one_dance_button = QPushButton("当前点位加入编舞")
        self.add_all_dance_button = QPushButton("十个点位同时加入编舞")
        self.target_delta_status = QLabel("尚无可比较的实时位置")
        self.target_delta_status.setWordWrap(True)
        target_grid.addWidget(QLabel("当前标签"), 0, 0)
        target_grid.addWidget(self.target_tag_choice, 0, 1, 1, 2)
        target_grid.addWidget(QLabel("目标 X"), 1, 0)
        target_grid.addWidget(self.target_x, 1, 1, 1, 2)
        target_grid.addWidget(QLabel("目标 Y"), 2, 0)
        target_grid.addWidget(self.target_y, 2, 1, 1, 2)
        target_grid.addWidget(self.map_target_edit, 3, 0, 1, 3)
        target_grid.addWidget(self.set_target_button, 4, 0)
        target_grid.addWidget(self.clear_target_button, 4, 1)
        target_grid.addWidget(self.save_targets_button, 4, 2)
        target_grid.addWidget(QLabel("已保存快照"), 5, 0)
        target_grid.addWidget(self.snapshot_choice, 5, 1, 1, 2)
        target_grid.addWidget(QLabel("快照名称"), 6, 0)
        target_grid.addWidget(self.snapshot_name, 6, 1, 1, 2)
        target_grid.addWidget(self.save_snapshot_button, 7, 0)
        target_grid.addWidget(self.load_snapshot_button, 7, 1)
        target_grid.addWidget(self.delete_snapshot_button, 7, 2)
        target_grid.addWidget(self.target_delta_status, 8, 0, 1, 3)
        target_grid.addWidget(QLabel("编舞占用/最长到达时间"), 9, 0, 1, 2)
        target_grid.addWidget(self.target_duration, 9, 2)
        target_grid.addWidget(self.move_one_target_button, 10, 0)
        target_grid.addWidget(self.move_all_targets_button, 10, 1, 1, 2)
        target_grid.addWidget(self.stop_target_button, 11, 0, 1, 3)
        target_grid.addWidget(self.add_one_dance_button, 12, 0)
        target_grid.addWidget(self.add_all_dance_button, 12, 1, 1, 2)
        target_hint = QLabel(
            "先选择 TAG，再勾选地图标点并点击右侧场地。目标点保存为场地坐标；"
            "加入编舞后会生成各机器狗独立的“TAG目标点”时间线指令。"
            "先用小位移现场试走；若方向不对，请核对第1区的“标零时机头朝向”。"
        )
        target_hint.setWordWrap(True)
        target_grid.addWidget(target_hint, 13, 0, 1, 3)
        settings_layout.addWidget(target_box)
        self.apply_button = QPushButton("应用并保存设置")
        self.apply_button.setProperty("role", "primary")
        self.reset_button = QPushButton("重置滤波与轨迹（等待新数据）")
        self.settings_status = QLabel("设置未应用时，地图继续使用上次配置。")
        self.settings_status.setWordWrap(True)
        panel_layout.addWidget(self.apply_button)
        panel_layout.addWidget(self.reset_button)
        panel_layout.addWidget(self.settings_status)
        settings_layout.addStretch()

        display = QWidget()
        display_layout = QVBoxLayout(display)
        display_layout.setContentsMargins(3, 0, 0, 0)
        legend = QHBoxLayout()
        self.show_raw = QCheckBox("灰叉：原始位置")
        self.show_raw.setToolTip("未经滤波的原始测量会抖动；表格始终保留原始值供核对。")
        self.show_raw.setChecked(False)
        self.show_trails = QCheckBox("显示滤波轨迹")
        self.show_trails.setChecked(True)
        legend.addWidget(QLabel("彩色：TAG1～TAG10 实时位置   空心灰：过期"))
        legend.addStretch()
        legend.addWidget(self.show_raw)
        legend.addWidget(self.show_trails)
        display_layout.addLayout(legend)
        self.map = StageMap(self)
        display_layout.addWidget(self.map, 1)
        self.position_table = QTableWidget(len(SUPPORTED_TAGS), 7)
        self.position_table.setHorizontalHeaderLabels(["标签 / 狗", "原始 X, Y", "滤波 X, Y", "有效数据龄", "接收/拒绝", "位置状态", "单位"])
        self.position_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.position_table.verticalHeader().setVisible(False)
        self.position_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.position_table.horizontalHeader().setStretchLastSection(True)
        self.position_table.setMinimumHeight(250)
        self.position_table.setMaximumHeight(320)
        self.position_table.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        for row in range(len(SUPPORTED_TAGS)):
            for col in range(7):
                self.position_table.setItem(row, col, QTableWidgetItem("--"))
        display_layout.addWidget(self.position_table)
        self.sample_status = QLabel("尚未收到 TAG1～TAG10 数据")
        self.sample_status.setWordWrap(True)
        display_layout.addWidget(self.sample_status)
        splitter.addWidget(display)
        splitter.setSizes([390, 850])
        layout.addWidget(splitter, 1)
        self.refresh_ports()
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.connect_serial)
        self.disconnect_button.clicked.connect(self.disconnect_serial)
        self.reader.status.connect(self.connection_status.setText)
        self.reader.connection_changed.connect(self.connection_changed)
        self.reader.samples.connect(self.receive_samples)
        self.apply_button.clicked.connect(self.apply_settings)
        self.reset_button.clicked.connect(self.reset_tracks)
        self.target_tag_choice.currentIndexChanged.connect(self.load_target_editor)
        self.set_target_button.clicked.connect(self.set_target_from_editors)
        self.clear_target_button.clicked.connect(self.clear_current_target)
        self.save_targets_button.clicked.connect(self.save_target_points)
        self.save_snapshot_button.clicked.connect(self.save_target_snapshot)
        self.load_snapshot_button.clicked.connect(self.load_target_snapshot)
        self.delete_snapshot_button.clicked.connect(self.delete_target_snapshot)
        self.snapshot_choice.currentTextChanged.connect(
            lambda name: self.snapshot_name.setText(name)
        )
        self.move_one_target_button.clicked.connect(
            lambda: self.request_target_move((self.target_tag_choice.currentData(),))
        )
        self.move_all_targets_button.clicked.connect(
            lambda: self.request_target_move(WAYPOINT_TAGS)
        )
        self.stop_target_button.clicked.connect(
            lambda checked=False: self.target_stop_requested.emit()
        )
        self.add_one_dance_button.clicked.connect(
            lambda: self.request_target_choreography((self.target_tag_choice.currentData(),))
        )
        self.add_all_dance_button.clicked.connect(
            lambda: self.request_target_choreography(WAYPOINT_TAGS)
        )
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.refresh_display)
        self.timer.start()
        self.refresh_snapshot_choices()
        self.load_target_editor()
        self.refresh_display()

    @staticmethod
    def spin(minimum, maximum, value, decimals):
        widget = QDoubleSpinBox()
        widget.setDecimals(decimals)
        widget.setRange(minimum, maximum)
        widget.setValue(value)
        widget.setMinimumWidth(80)
        widget.setKeyboardTracking(False)
        return widget

    def tag_label(self, tag):
        robot_id = self.settings.bindings.get(tag, "")
        if not robot_id:
            return f"{tag}（未绑定）"
        return f"{tag} / {self.robot_names.get(robot_id, robot_id)}"

    def load_target_editor(self) -> None:
        tag = self.target_tag_choice.currentData()
        point = self.target_points.get(tag)
        self.target_x.setValue(0.0 if point is None else point[0])
        self.target_y.setValue(0.0 if point is None else point[1])
        self.settings_status.setText(
            f"{tag} 尚未设置目标点。" if point is None
            else f"{tag} 目标点：({point[0]:.3f}, {point[1]:.3f}) {self.settings.unit_label}"
        )

    def set_target_point(self, tag, point) -> None:
        if tag not in WAYPOINT_TAGS:
            return
        normalized = (float(point[0]), float(point[1]))
        if not inside_stage(normalized, self.settings.corners):
            self.settings_status.setText(f"{tag} 目标点位于场地边界外，未设置。")
            return
        self.target_points[tag] = normalized
        self.target_x.setValue(normalized[0])
        self.target_y.setValue(normalized[1])
        self.settings_status.setText(
            f"已标记 {tag} 目标点 ({normalized[0]:.3f}, {normalized[1]:.3f})；请点击保存点位。"
        )
        self.map.update()

    def set_target_from_editors(self) -> None:
        self.set_target_point(
            self.target_tag_choice.currentData(),
            (self.target_x.value(), self.target_y.value()),
        )

    def clear_current_target(self) -> None:
        tag = self.target_tag_choice.currentData()
        self.target_points[tag] = None
        self.load_target_editor()
        self.map.update()

    def save_target_points(self) -> bool:
        candidate = PositionSettings(**asdict(self.settings))
        candidate.tag_targets = dict(self.target_points)
        try:
            candidate.validate()
            candidate.save(self.config_path)
        except (OSError, ValueError) as exc:
            self.settings_status.setText(f"目标点未保存：{exc}")
            return False
        self.settings.tag_targets = dict(candidate.tag_targets)
        self.settings_status.setText("TAG1～TAG10 地图目标点已保存。")
        return True

    def refresh_snapshot_choices(self, selected: str = "") -> None:
        self.snapshot_choice.blockSignals(True)
        self.snapshot_choice.clear()
        for name in self.settings.waypoint_snapshots:
            self.snapshot_choice.addItem(name)
        if selected:
            index = self.snapshot_choice.findText(selected)
            if index >= 0:
                self.snapshot_choice.setCurrentIndex(index)
        self.snapshot_choice.blockSignals(False)
        current = self.snapshot_choice.currentText()
        if current:
            self.snapshot_name.setText(current)

    def save_target_snapshot(self) -> bool:
        name = self.snapshot_name.text().strip()
        if not name:
            self.settings_status.setText("请先输入点位快照名称。")
            return False
        candidate = PositionSettings(**asdict(self.settings))
        candidate.tag_targets = dict(self.target_points)
        candidate.waypoint_snapshots[name] = dict(self.target_points)
        try:
            candidate.validate()
            candidate.save(self.config_path)
        except (OSError, ValueError) as exc:
            self.settings_status.setText(f"点位快照未保存：{exc}")
            return False
        self.settings.tag_targets = dict(candidate.tag_targets)
        self.settings.waypoint_snapshots = {
            key: dict(value)
            for key, value in candidate.waypoint_snapshots.items()
        }
        self.refresh_snapshot_choices(name)
        self.settings_status.setText(
            f"已保存点位快照“{name}”，可反复载入、试走或加入编舞。"
        )
        return True

    def load_target_snapshot(self) -> bool:
        name = self.snapshot_choice.currentText()
        snapshot = self.settings.waypoint_snapshots.get(name)
        if snapshot is None:
            self.settings_status.setText("请先选择已保存的点位快照。")
            return False
        self.target_points = dict(snapshot)
        self.settings.tag_targets = dict(snapshot)
        self.load_target_editor()
        self.map.update()
        self.settings_status.setText(f"已载入点位快照“{name}”。")
        return True

    def delete_target_snapshot(self) -> bool:
        name = self.snapshot_choice.currentText()
        if not name or name not in self.settings.waypoint_snapshots:
            self.settings_status.setText("没有可删除的点位快照。")
            return False
        candidate = PositionSettings(**asdict(self.settings))
        candidate.waypoint_snapshots.pop(name, None)
        try:
            candidate.save(self.config_path)
        except (OSError, ValueError) as exc:
            self.settings_status.setText(f"点位快照未删除：{exc}")
            return False
        self.settings.waypoint_snapshots = {
            key: dict(value)
            for key, value in candidate.waypoint_snapshots.items()
        }
        self.refresh_snapshot_choices()
        self.settings_status.setText(f"已删除点位快照“{name}”。")
        return True

    def target_payload(self, tags):
        scale = unit_to_meters(self.settings.unit)
        if scale is None or not self.settings.xy_confirmed or not self.settings.heading_confirmed:
            self.settings_status.setText("请先确认坐标单位、X/Y 方向和标零机头方向并应用设置。")
            return None
        entries = []
        missing = []
        for tag in tags:
            point = self.target_points.get(tag)
            robot_id = self.settings.bindings.get(tag, "")
            if point is None or not robot_id:
                missing.append(tag)
                continue
            entries.append({
                "tag": tag,
                "robot_id": robot_id,
                "raw": point,
                "meters": (point[0] * scale, point[1] * scale),
                "direction_lock": self.direction_lock_for(robot_id),
            })
        if missing:
            self.settings_status.setText("以下标签未绑定或未设置目标点：" + ", ".join(missing))
            return None
        return {
            "entries": entries,
            "snapshot_name": self.snapshot_choice.currentText().strip(),
            "duration": float(self.target_duration.value()),
            "stale_seconds": control_stale_seconds(self.settings.stale_seconds),
            "stage_heading_deg": float(self.settings.zero_heading_deg),
            "return_speed_percent": float(self.settings.return_speed_percent),
            "continuous_return": bool(self.settings.continuous_return),
            "return_latency_ms": float(self.settings.return_latency_ms),
            "return_max_seconds": float(self.settings.return_max_seconds),
        }

    def request_target_move(self, tags) -> None:
        payload = self.target_payload(tags)
        if payload is not None:
            self.target_move_requested.emit(payload)

    def request_target_choreography(self, tags) -> None:
        payload = self.target_payload(tags)
        if payload is not None:
            self.target_choreography_requested.emit(payload)

    def direction_lock_for(self, robot_id: str) -> str:
        choice = self.direction_lock_choices.get(robot_id)
        value = choice.currentData() if choice is not None else "xy"
        return value if value in {"xy", "x", "y"} else "xy"

    def calibrate_tag_zero(self, tag):
        robot_id = self.binding_choices[tag].currentData()
        if not robot_id:
            self.settings_status.setText(f"{tag} 未绑定机器狗，不能标定零位。")
            return
        track = self.tracks[tag]
        position = track.raw
        if position is None:
            self.settings_status.setText(f"{tag} 尚无有效位置，不能标定零位。")
            return
        scale = unit_to_meters(self.settings.unit)
        if scale is None:
            self.settings_status.setText(
                f"{tag} 单位未确认（raw），无法换算成米用于控制。"
            )
            return
        if not self.settings.xy_confirmed:
            self.settings_status.setText(
                f"{tag} 的 X/Y 坐标方向尚未确认，禁止用于自动回位。"
            )
            return
        if not self.settings.heading_confirmed:
            self.settings_status.setText(
                "尚未确认标零时的机头朝向，禁止自动回位。请在第 1 区选择方向、勾选确认并应用设置。"
            )
            return
        stale_limit = control_stale_seconds(self.settings.stale_seconds)
        raw_time = track.last_raw_at
        if (
            not self.reader.port.isOpen()
            or raw_time is None
            or time.monotonic() - raw_time > stale_limit
        ):
            self.settings_status.setText(
                f"{tag} 数据过期或串口断开，不能标定零位。"
            )
            return
        x_m = float(position[0]) * scale
        y_m = float(position[1]) * scale
        received_time = raw_time if raw_time is not None else time.monotonic()
        self.zero_calibrated.emit(
            {
                "tag": tag,
                "robot_id": robot_id,
                "raw": tuple(position),
                "meters": (x_m, y_m),
                "received_at": float(received_time),
                "stale_seconds": stale_limit,
                "stage_heading_deg": float(self.settings.zero_heading_deg),
                "return_speed_percent": float(self.settings.return_speed_percent),
                "continuous_return": bool(self.settings.continuous_return),
                "return_latency_ms": float(self.settings.return_latency_ms),
                "return_max_seconds": float(self.settings.return_max_seconds),
                "direction_lock": self.direction_lock_for(robot_id),
            }
        )
        self.settings_status.setText(
            f"{tag} 当前位置 {position[0]:.2f}, {position[1]:.2f} "
            f"{self.settings.unit_label} 已标定为 "
            f"{self.robot_names.get(robot_id, robot_id)} 零位 "
            f"({x_m:.3f}, {y_m:.3f}) m"
        )

    def refresh_ports(self):
        selected = self.port_choice.currentData() or self.settings.port
        self.port_choice.clear()
        self.port_choice.addItem("自动检测 USB 串口", "")
        for name, description, is_usb in self.reader.available_ports():
            self.port_choice.addItem(f"{name} · {description}", name)
        index = self.port_choice.findData(selected)
        self.port_choice.setCurrentIndex(max(0, index))

    def connect_serial(self):
        self.reset_tracks()
        self.reader.start(str(self.port_choice.currentData() or ""))

    def disconnect_serial(self):
        self.reader.stop()
        self.connection_status.setText("定位串口已释放；地图只保留最后有效位置（灰色）。")
        self.refresh_display()

    def connection_changed(self, connected):
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)
        self.port_choice.setEnabled(not connected)
        self.refresh_ports_button.setEnabled(not connected)

    def receive_samples(self, samples):
        for sample in samples:
            track = self.tracks.get(sample.tag)
            if track:
                now = time.monotonic()
                track.update(sample, now)
                self.sample_processed.emit({"tag": sample.tag, "monotonic": now,
                                            "raw": (sample.x, sample.y), "filtered": track.kalman.position,
                                            "accepted": track.accepted_latest})

    def reset_tracks(self):
        self.tracks = {tag: TagTrack(tag, self.settings) for tag in SUPPORTED_TAGS}
        self.refresh_display()

    def apply_settings(self):
        candidate = PositionSettings(**asdict(self.settings))
        candidate.port = str(self.port_choice.currentData() or "")
        candidate.unit = self.unit_choice.currentData()
        candidate.xy_confirmed = self.xy_confirm.isChecked()
        candidate.heading_confirmed = self.heading_confirm.isChecked()
        candidate.zero_heading_deg = float(self.zero_heading_choice.currentData())
        candidate.return_speed_percent = self.return_speed.value()
        candidate.continuous_return = self.continuous_return.isChecked()
        candidate.return_latency_ms = self.return_latency.value()
        candidate.return_max_seconds = self.return_timeout.value()
        candidate.tag_targets = dict(self.target_points)
        candidate.corners = tuple(tuple(editor.value() for editor in row) for row in self.corner_inputs)
        candidate.stage_configured = True
        candidate.bindings = {tag: choice.currentData() for tag, choice in self.binding_choices.items()}
        candidate.direction_locks = {
            robot_id: choice.currentData()
            for robot_id, choice in self.direction_lock_choices.items()
        }
        candidate.measurement_sigma = self.measurement_noise.value()
        candidate.acceleration_sigma = self.acceleration_noise.value()
        candidate.gate_sigma = self.gate.value()
        candidate.stale_seconds = self.stale_timeout.value()
        candidate.filter_mode = self.filter_mode_choice.currentData()
        try:
            candidate.validate()
            missing = set(candidate.bindings.values()) - {""} - set(self.robot_names)
            if missing:
                raise ValueError(f"绑定的机器狗不存在：{', '.join(sorted(missing))}")
            candidate.save(self.config_path)
        except (OSError, ValueError) as exc:
            self.settings_status.setText(f"未应用：{exc}")
            return
        self.settings = candidate
        self.target_points = dict(candidate.tag_targets)
        self.refresh_snapshot_choices(self.snapshot_choice.currentText())
        self.reset_tracks()
        control_warning = ""
        if not candidate.xy_confirmed:
            control_warning += " X/Y 协议仍未确认。"
        if not candidate.heading_confirmed:
            control_warning += " 标零机头方向仍未确认。"
        self.settings_status.setText(
            "已保存；滤波与轨迹已重置。" + control_warning
            + (" 当前仅允许显示，禁止自动回位。" if control_warning else " 请重新接收数据并逐台标零。")
        )

    def refresh_display(self):
        now = time.monotonic()
        for row, (tag, track) in enumerate(self.tracks.items()):
            raw_pos = track.raw
            filtered = raw_pos
            if raw_pos is None:
                state = "等待数据"
            elif not self.reader.port.isOpen():
                state = "串口断开"
            elif track.raw_stale(now):
                state = "过期/失联"
            else:
                state = "原始实时"
            if raw_pos is not None and self.settings.stage_configured and not inside_stage(raw_pos, self.settings.corners):
                state += " · 场外"
            if raw_pos is not None and not self.settings.xy_confirmed:
                state += " · X/Y未确认"
            age = "--" if track.last_raw_at is None else f"{max(0, now - track.last_raw_at):.1f}s"
            values = [self.tag_label(tag), self.format_point(track.raw), self.format_point(raw_pos), age,
                      f"{track.received} / {track.kalman.rejected}", state, self.settings.unit_label]
            for col, value in enumerate(values):
                self.position_table.item(row, col).setText(value)
        count = sum(track.received for track in self.tracks.values())
        live = sum(
            1 for track in self.tracks.values()
            if track.raw is not None and not track.raw_stale(now)
        )
        self.sample_status.setText(f"收到 {self.reader.received_bytes} 字节 · TAG1～TAG10 {count} 帧 · 当前原始有效 {live}/10 · 无效行 {self.reader.parser.invalid_lines} · 控制使用原始坐标")
        tag = self.target_tag_choice.currentData()
        target = self.target_points.get(tag)
        current = self.tracks[tag].raw if tag in self.tracks else None
        if target is None:
            self.target_delta_status.setText(f"{tag} 尚未设置目标点。")
        elif current is None:
            self.target_delta_status.setText(
                f"{tag} 目标=({target[0]:.3f}, {target[1]:.3f})；等待实时位置。"
            )
        else:
            dx, dy = target[0] - current[0], target[1] - current[1]
            heading = self.zero_heading_choice.currentText()
            self.target_delta_status.setText(
                f"{tag} 现场预览：舞台 ΔX={dx:+.3f}、ΔY={dy:+.3f} "
                f"{self.settings.unit_label}；标零机头基准={heading}。"
            )
        self.map.update()

    @staticmethod
    def format_point(point):
        return "--" if point is None else f"{point[0]:.2f}, {point[1]:.2f}"

    def showEvent(self, event):
        self.timer.start()
        super().showEvent(event)

    def closeEvent(self, event):
        self.reader.stop()
        self.connection_status.setText("定位窗口已停止接收；重新打开后请点击连接。")
        self.timer.stop()
        self.closed.emit()
        event.accept()
