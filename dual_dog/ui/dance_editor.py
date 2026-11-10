from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMenu, QPushButton, QTableWidget,
    QFileDialog, QMessageBox, QToolButton, QVBoxLayout, QWidget,
)

from dual_dog.paths import DEMO_CONFIG
from dual_dog.control.choreography import (
    MINIMUM_FLIP_TRACK_SECONDS, STUNT_RETURN_TRACK_SECONDS,
    DanceStep, MoveDirection, PhaseKind, resolve_dance_time_conflicts,
    build_multitrack_dance, TRICK_POST_ACTION_CORRECTION_SECONDS,
    merge_dance_documents, overlapping_dance_cue_messages,
)
from dual_dog.models import Command, RobotConfig


# Real BPX flips spend about 0.5 s flushing zero velocity, 1.0--2.0 s
# establishing Walk and 0.5 s disabling velocity control before the firmware
# receives the flip command.  This is scheduling guidance only: the operator
# can still override every start time.
FLIP_PREPARATION_ESTIMATE_SECONDS = 3.0


class RobotMultiSelector(QWidget):
    """Compact table-cell selector for one or more choreography tracks."""

    def __init__(
        self, robots: list[RobotConfig], targets: tuple[str, ...] = (), parent=None
    ) -> None:
        super().__init__(parent)
        self._robots = robots
        self._actions: dict[str, QAction] = {}

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.button = QToolButton(self)
        self.button.setPopupMode(QToolButton.InstantPopup)
        self.menu = QMenu(self.button)

        use_current = QAction("所有当前选中设备", self.menu)
        use_current.triggered.connect(lambda: self.set_targets(()))
        self.menu.addAction(use_current)
        self.menu.addSeparator()
        for robot in robots:
            action = QAction(f"{robot.name}（{robot.robot_id}）", self.menu)
            action.setCheckable(True)
            action.toggled.connect(self._update_text)
            self.menu.addAction(action)
            self._actions[robot.robot_id] = action
        self.menu.addSeparator()
        select_all = QAction("全选全部机器人", self.menu)
        select_all.triggered.connect(
            lambda: self.set_targets(tuple(self._actions))
        )
        self.menu.addAction(select_all)

        self.button.setMenu(self.menu)
        layout.addWidget(self.button)
        self.set_targets(targets)

    def set_targets(self, targets: tuple[str, ...]) -> None:
        selected = set(targets)
        for robot_id, action in self._actions.items():
            action.blockSignals(True)
            action.setChecked(robot_id in selected)
            action.blockSignals(False)
        self._update_text()

    def targets(self) -> tuple[str, ...]:
        return tuple(
            robot.robot_id
            for robot in self._robots
            if self._actions[robot.robot_id].isChecked()
        )

    def _update_text(self, *_args) -> None:
        targets = self.targets()
        if not targets:
            text = "所有当前选中设备"
        elif len(targets) <= 3:
            text = "、".join(target.replace("dog", "狗") for target in targets)
        else:
            text = f"已选 {len(targets)} 台机器人"
        self.button.setText(text)
        self.button.setToolTip(
            "开始编舞时勾选的全部设备" if not targets else "、".join(targets)
        )


class ChoreographyEditorDialog(QDialog):
    ACTIONS = [
        ("左翻", Command.LEFT_FLIP), ("右翻", Command.RIGHT_FLIP),
        ("Pronk", Command.PRONK), ("Pace", Command.PACE),
        ("Bound", Command.BOUND), ("双足正立", Command.BIPEDAL),
        ("双足倒立", Command.INV_BIPEDAL), ("Run", Command.RUN),
        ("Up Jump", Command.UP_JUMP), ("Front Jump", Command.FRONT_JUMP),
        ("Back Jump", Command.BACK_JUMP), ("Left Jump", Command.LEFT_JUMP),
        ("Right Jump", Command.RIGHT_JUMP), ("Upright", Command.UPRIGHT),
    ]
    POSTURES = [("起立", Command.STAND), ("坐下", Command.SIT)]
    DIRECTIONS = [
        ("前进", MoveDirection.FORWARD), ("后退", MoveDirection.BACKWARD),
        ("左移", MoveDirection.LEFT), ("右移", MoveDirection.RIGHT),
        ("左转", MoveDirection.TURN_LEFT), ("右转", MoveDirection.TURN_RIGHT),
    ]

    def __init__(
        self, steps: list[DanceStep], robots: list[RobotConfig],
        max_x: float, max_y: float, max_yaw: float,
        default_non_flip_seconds: float, trick_lock_seconds: float, parent=None,
    ) -> None:
        super().__init__(parent)
        self.robots = robots
        self.max_x, self.max_y, self.max_yaw = max_x, max_y, max_yaw
        self.default_non_flip_seconds = default_non_flip_seconds
        self.trick_lock_seconds = trick_lock_seconds
        self.setWindowTitle("多轨编舞时间线编辑器")
        self.resize(1520, 720)
        layout = QVBoxLayout(self)
        help_label = QLabel(
            "每行直接选择哪些机器人、执行什么动作以及持续多久；目标机器人支持任意多选。"
            "相同开始时间可让不同机器狗同时执行不同动作；同一机器狗的指令时间重叠时，"
            "启动前会弹确认并放行执行。"
            "“所有当前选中设备”包含开始编舞时勾选的每台设备。"
            "非空翻动作按填写时长占轨；空翻时长栏显示固定的安全占用时间。"
        )
        help_label.setWordWrap(True)
        layout.addWidget(help_label)

        imports = QHBoxLayout()
        self.import_dances_button = QPushButton("导入并合并舞蹈（可多选）")
        self.import_dances_button.setToolTip(
            "保留当前指令，将多个 JSON 方案按各自原开始时间合并。音乐使用主界面当前选择。"
        )
        self.import_dances_button.clicked.connect(self.import_dances)
        imports.addWidget(self.import_dances_button)
        imports.addWidget(QLabel("分别编排各台机器狗后，可一次选择多个方案合并，再统一保存。"))
        imports.addStretch()
        layout.addLayout(imports)
        self.import_directory = str(DEMO_CONFIG.parent)

        self.table = QTableWidget(0, 10)
        self.table.setHorizontalHeaderLabels(
            ["预计开始时间（可修改）", "选择机器人（可多选）", "指令类型", "动作 / 方向", "持续时间 / 安全占用", "速度", "特技后回标零", "点位名称", "目标 X(m)", "目标 Y(m)"]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.verticalHeader().setDefaultSectionSize(44)
        layout.addWidget(self.table, 1)

        presets = QHBoxLayout()
        wave = QPushButton("生成波浪动作示例")
        formation = QPushButton("生成交错队形示例")
        wave.clicked.connect(self.add_wave_preset)
        formation.clicked.connect(self.add_formation_preset)
        auto_timing = QPushButton("重新自动估算开始时间")
        auto_timing.setToolTip("按当前表格顺序依次排列；计算后每个开始时间仍可手动修改")
        auto_timing.clicked.connect(self.auto_arrange_start_times)
        presets.addWidget(QLabel("快速模板："))
        presets.addWidget(wave)
        presets.addWidget(formation)
        presets.addWidget(auto_timing)
        self.resolve_conflicts_button = QPushButton("一键解决时间冲突")
        self.resolve_conflicts_button.setToolTip(
            "计入特技安全占用、回位和自动纠偏；只向后调整开始时间，保留原动作顺序及其他字段"
        )
        self.resolve_conflicts_button.clicked.connect(self.resolve_time_conflicts)
        presets.addWidget(self.resolve_conflicts_button)
        presets.addStretch()
        layout.addLayout(presets)
        self.conflict_status = QLabel("")
        self.conflict_status.setWordWrap(True)
        layout.addWidget(self.conflict_status)

        tools = QHBoxLayout()
        for text, kind in (
            ("+ 添加动作", PhaseKind.TRICK), ("+ 添加起立/坐下", PhaseKind.POSTURE),
            ("+ 添加队形移动", PhaseKind.MOVE),
            ("+ 添加TAG目标点", PhaseKind.TAG_TARGET),
            ("+ 添加纠偏", PhaseKind.CORRECT_YAW), ("+ 添加停止", PhaseKind.STOP),
        ):
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, k=kind: self.add_step(k))
            tools.addWidget(button)
        up, down, remove = QPushButton("↑ 上移"), QPushButton("↓ 下移"), QPushButton("删除选中指令")
        up.clicked.connect(lambda: self.move_step(-1))
        down.clicked.connect(lambda: self.move_step(1))
        remove.clicked.connect(self.remove_step)
        tools.addSpacing(20)
        for button in (up, down, remove):
            tools.addWidget(button)
        tools.addStretch()
        layout.addLayout(tools)

        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Save).setText("保存多轨编舞")
        buttons.button(QDialogButtonBox.Cancel).setText("取消")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self.rebuild(steps, 0)

    def import_dances(self, checked: bool = False) -> None:
        filenames, _ = QFileDialog.getOpenFileNames(
            self, "选择要合并的舞蹈方案（可多选）", self.import_directory,
            "编舞方案 (*.json);;所有文件 (*)",
        )
        if not filenames:
            return
        # The dialog can return duplicates; do not append the same file twice
        # within one selection. Re-importing in a later operation is deliberate.
        paths = list(dict.fromkeys(Path(name).resolve() for name in filenames))
        current = self.steps()
        first_row = self.table.rowCount()
        try:
            documents = []
            for path in paths:
                try:
                    payload = json.loads(path.read_text(encoding="utf-8-sig"))
                except (OSError, ValueError) as exc:
                    raise ValueError(f"{path.name}：{exc}") from exc
                documents.append((path.name, payload))
            targets = [robot.robot_id for robot in self.robots]
            merged = merge_dance_documents(current, documents, self.trick_lock_seconds, targets)
            added = merged[len(current):]
            phases = build_multitrack_dance(
                merged, self.trick_lock_seconds, targets, allow_overlap=True,
                post_action_correction_seconds=TRICK_POST_ACTION_CORRECTION_SECONDS,
            )
            conflicts = overlapping_dance_cue_messages(phases)
            # Append rather than rebuild: existing unsaved cell values stay intact.
            self.table.setUpdatesEnabled(False)
            for step in added:
                self.add_step(
                    step.kind, step.command, step.duration, step.speed,
                    step.start_time, step.targets, step.direction,
                    step.target_x, step.target_y, step.target_name, step.return_to_start,
                )
                row = self.table.rowCount() - 1
                # Legacy FORWARD uses the forward movement control in this UI.
                self.table.cellWidget(row, 0).setProperty(
                    "legacy_forward", step.kind is PhaseKind.FORWARD,
                )
                duration = self.table.cellWidget(row, 4)
                duration.setValue(step.duration)
                if step.kind is PhaseKind.TRICK and step.command in {Command.LEFT_FLIP, Command.RIGHT_FLIP}:
                    duration.setToolTip(
                        f"保留源方案时长；空翻实际安全占用按当前配置 {self.trick_lock_seconds:.2f} 秒计算"
                    )
            # Refuse silent clamping/rounding of imported values outside the
            # editor's supported ranges. Roll back the complete batch on error.
            if any(
                set(actual.targets) != set(expected.targets)
                or replace(actual, targets=expected.targets) != expected
                for actual, expected in zip(self.steps(), merged)
            ):
                raise ValueError("源方案存在超出编辑器范围或精度的参数，无法原样导入；本次未合并任何指令")
        except (OSError, TypeError, ValueError) as exc:
            self.table.setRowCount(first_row)
            QMessageBox.warning(self, "导入舞蹈失败", str(exc))
            return
        finally:
            self.table.setUpdatesEnabled(True)
        self.import_directory = str(paths[0].parent)
        self.table.selectRow(first_row)
        wildcard_count = sum(not step.targets for step in added)
        self.conflict_status.setText(
            f"已合并 {len(paths)} 个方案、{len(added)} 条指令，当前共 {len(merged)} 条；保留各方案原开始时间。"
            + (f"检测到 {len(conflicts)} 处时间冲突，可点击“一键解决时间冲突”。" if conflicts else "未发现时间冲突。")
            + (f"其中 {wildcard_count} 条使用“所有当前选中设备”，请核对目标机器人。" if wildcard_count else "")
            + "音乐使用主界面当前选择；请保存多轨编舞，再保存方案 JSON。"
        )

    def _populate_action(
        self, combo: QComboBox, kind: PhaseKind,
        command: Command | None = None,
        direction: MoveDirection = MoveDirection.FORWARD,
    ) -> None:
        combo.blockSignals(True)
        combo.clear()
        if kind is PhaseKind.TRICK:
            for label, value in self.ACTIONS:
                combo.addItem(label, value.value)
            if command is not None:
                combo.setCurrentIndex(max(0, combo.findData(command.value)))
        elif kind is PhaseKind.POSTURE:
            for label, value in self.POSTURES:
                combo.addItem(label, value.value)
            if command is not None:
                combo.setCurrentIndex(max(0, combo.findData(command.value)))
        elif kind in {PhaseKind.MOVE, PhaseKind.FORWARD}:
            for label, value in self.DIRECTIONS:
                combo.addItem(label, value.value)
            combo.setCurrentIndex(max(0, combo.findData(direction.value)))
        elif kind is PhaseKind.TAG_TARGET:
            combo.addItem("UWB/TAG 绝对目标点", "tag_target")
        else:
            combo.addItem("—", "")
        combo.blockSignals(False)

    def _phase_end(self, row: int) -> float:
        start = self.table.cellWidget(row, 0).value()
        kind = PhaseKind(self.table.cellWidget(row, 2).currentData())
        duration = self.table.cellWidget(row, 4).value()
        if kind is PhaseKind.TRICK:
            action = self.table.cellWidget(row, 3).currentData()
            if action in {Command.LEFT_FLIP.value, Command.RIGHT_FLIP.value}:
                duration = max(self.trick_lock_seconds, MINIMUM_FLIP_TRACK_SECONDS)
                duration += FLIP_PREPARATION_ESTIMATE_SECONDS
                if self.table.cellWidget(row, 6).isChecked():
                    duration += STUNT_RETURN_TRACK_SECONDS
            elif self.table.cellWidget(row, 6).isChecked():
                duration += self.trick_lock_seconds + STUNT_RETURN_TRACK_SECONDS
        return start + duration

    def next_start_time(self) -> float:
        return max((self._phase_end(row) for row in range(self.table.rowCount())), default=0.0)

    def auto_arrange_start_times(self, checked: bool = False) -> None:
        """Fill editable start times using conservative end-to-end estimates."""
        cursor = 0.0
        for row in range(self.table.rowCount()):
            start_spin = self.table.cellWidget(row, 0)
            start_spin.setValue(cursor)
            cursor = self._phase_end(row)

    def resolve_time_conflicts(self, checked: bool = False) -> None:
        source = self.steps()
        if not source:
            self.conflict_status.setText("请先添加编舞指令。")
            return
        targets = [robot.robot_id for robot in self.robots]
        try:
            resolved = resolve_dance_time_conflicts(source, self.trick_lock_seconds, targets)
            phases = build_multitrack_dance(
                resolved, self.trick_lock_seconds, targets,
                post_action_correction_seconds=TRICK_POST_ACTION_CORRECTION_SECONDS,
            )
        except ValueError as exc:
            QMessageBox.warning(self, "解决时间冲突失败", str(exc))
            return
        changed = 0
        for row, (before, after) in enumerate(zip(source, resolved)):
            if before.start_time == after.start_time:
                continue
            start_spin = self.table.cellWidget(row, 0)
            start_spin.setMaximum(max(start_spin.maximum(), after.start_time))
            start_spin.setValue(after.start_time)
            changed += 1
        end = max(phase.start_time + phase.duration for phase in phases)
        self.conflict_status.setText(
            f"已调整 {changed} 条指令的开始时间，含自动纠偏的总时长为 {end:.2f} 秒，时间冲突已消除。"
            "机器人、指令类型、动作、持续时间及其他参数均未修改。请保存编舞；音乐卡点需按新时间核对。"
            if changed else "当前编舞无时间冲突（已计入自动纠偏和安全占用），无需调整。"
        )

    def add_step(
        self, kind: PhaseKind, command: Command | None = None,
        duration: float = 0.0, speed: float = 0.0,
        start_time: float | None = None, targets: tuple[str, ...] | None = None,
        direction: MoveDirection = MoveDirection.FORWARD,
        target_x: float = 0.0, target_y: float = 0.0,
        target_name: str = "",
        return_to_start: bool = False,
    ) -> None:
        row = self.table.rowCount()
        default_start = self.next_start_time() if start_time is None else start_time
        self.table.insertRow(row)
        start_spin = QDoubleSpinBox()
        start_spin.setRange(0.0, max(600.0, default_start))
        start_spin.setDecimals(2)
        start_spin.setSingleStep(0.25)
        start_spin.setSuffix(" s")
        start_spin.setValue(default_start)
        start_spin.setToolTip("系统根据前面动作预估；可直接输入需要的实际开始时间")

        default_targets = (
            tuple(robot.robot_id for robot in self.robots[:10])
            if targets is None else targets
        )
        target_selector = RobotMultiSelector(self.robots, default_targets)

        kind_combo = QComboBox()
        for label, value in (
            ("动作", PhaseKind.TRICK), ("起立/坐下", PhaseKind.POSTURE),
            ("队形移动", PhaseKind.MOVE),
            ("TAG目标点", PhaseKind.TAG_TARGET),
            ("纠正方向", PhaseKind.CORRECT_YAW), ("停止", PhaseKind.STOP),
        ):
            kind_combo.addItem(label, value.value)
        normalized_kind = PhaseKind.MOVE if kind is PhaseKind.FORWARD else kind
        kind_combo.setCurrentIndex(kind_combo.findData(normalized_kind.value))
        action_combo = QComboBox()
        self._populate_action(action_combo, normalized_kind, command, direction)

        duration_spin = QDoubleSpinBox()
        duration_spin.setRange(0.0, 60.0)
        duration_spin.setDecimals(2)
        duration_spin.setSingleStep(0.25)
        duration_spin.setSuffix(" s")
        defaults = {
            PhaseKind.TRICK: self.default_non_flip_seconds,
            PhaseKind.POSTURE: 3.0,
            PhaseKind.MOVE: 2.0, PhaseKind.CORRECT_YAW: 3.0, PhaseKind.STOP: 0.1,
            PhaseKind.TAG_TARGET: 12.0,
        }
        duration_spin.setValue(duration if duration > 0 else defaults[normalized_kind])
        speed_spin = QDoubleSpinBox()
        speed_spin.setDecimals(2)
        speed_spin.setSingleStep(0.05)
        return_to_start_check = QCheckBox("启用")
        return_to_start_check.setChecked(return_to_start)
        return_to_start_check.setToolTip(
            "特技结束后立即使用 UWB 回到已标零位置，默认 4 秒内完成；未标零或 UWB 不可用时不回位"
        )
        target_name_edit = QLineEdit(target_name)
        target_name_edit.setPlaceholderText("进场队形/TAG1")
        target_x_spin = QDoubleSpinBox()
        target_y_spin = QDoubleSpinBox()
        for target_spin, target_value in (
            (target_x_spin, target_x), (target_y_spin, target_y)
        ):
            target_spin.setRange(-10000.0, 10000.0)
            target_spin.setDecimals(3)
            target_spin.setSingleStep(0.05)
            target_spin.setValue(target_value)
        for column, widget in enumerate(
            (
                start_spin, target_selector, kind_combo, action_combo,
                duration_spin, speed_spin, return_to_start_check, target_name_edit,
                target_x_spin, target_y_spin,
            )
        ):
            self.table.setCellWidget(row, column, widget)

        action_values = {item.value for _, item in self.ACTIONS}
        posture_values = {item.value for _, item in self.POSTURES}
        direction_values = {item.value for _, item in self.DIRECTIONS}
        flip_values = {Command.LEFT_FLIP.value, Command.RIGHT_FLIP.value}
        initial_is_flip = (
            normalized_kind is PhaseKind.TRICK
            and action_combo.currentData() in flip_values
        )
        stored_duration = [
            self.default_non_flip_seconds if initial_is_flip else duration_spin.value()
        ]
        was_flip = [False]

        def update_duration_mode(*_args) -> None:
            current_kind = PhaseKind(kind_combo.currentData())
            is_flip = (
                current_kind is PhaseKind.TRICK
                and action_combo.currentData() in flip_values
            )
            if is_flip:
                if not was_flip[0]:
                    stored_duration[0] = duration_spin.value()
                duration_spin.setValue(
                    max(self.trick_lock_seconds, MINIMUM_FLIP_TRACK_SECONDS)
                )
                duration_spin.setEnabled(False)
                duration_spin.setToolTip("空翻按特技安全锁定时间占用轨道，不使用自定义持续时间")
            else:
                duration_spin.setEnabled(True)
                duration_spin.setToolTip("")
                if was_flip[0]:
                    duration_spin.setValue(stored_duration[0])
            was_flip[0] = is_flip

        def update_speed_limit(*_args) -> None:
            current_kind = PhaseKind(kind_combo.currentData())
            if current_kind is PhaseKind.TAG_TARGET:
                speed_spin.setRange(70.0, 200.0)
                speed_spin.setSingleStep(10.0)
                speed_spin.setSuffix(" %")
                speed_spin.setValue(min(200.0, max(70.0, speed if speed > 0 else 100.0)))
                return
            if current_kind is not PhaseKind.MOVE:
                speed_spin.setRange(0.0, max(self.max_x, self.max_y, self.max_yaw))
                speed_spin.setValue(0.0)
                speed_spin.setSuffix("")
                return
            move_direction = MoveDirection(action_combo.currentData())
            if move_direction in {MoveDirection.LEFT, MoveDirection.RIGHT}:
                limit, suffix = self.max_y, " m/s"
            elif move_direction in {MoveDirection.TURN_LEFT, MoveDirection.TURN_RIGHT}:
                limit, suffix = self.max_yaw, " rad/s"
            else:
                limit, suffix = self.max_x, " m/s"
            current = speed_spin.value()
            speed_spin.setRange(min(0.05, limit), limit)
            speed_spin.setSuffix(suffix)
            desired = speed if speed > 0 else min(0.25, limit)
            speed_spin.setValue(min(current if current > 0 else desired, limit))

        def update_enabled(*_args) -> None:
            current_kind = PhaseKind(kind_combo.currentData())
            old_data = action_combo.currentData()
            if current_kind is PhaseKind.TRICK:
                old_command = Command(old_data) if old_data in action_values else None
                self._populate_action(action_combo, current_kind, old_command)
            elif current_kind is PhaseKind.POSTURE:
                old_command = Command(old_data) if old_data in posture_values else None
                self._populate_action(action_combo, current_kind, old_command)
            elif current_kind is PhaseKind.MOVE:
                old_direction = MoveDirection(old_data) if old_data in direction_values else MoveDirection.FORWARD
                self._populate_action(action_combo, current_kind, direction=old_direction)
            elif current_kind is PhaseKind.TAG_TARGET:
                self._populate_action(action_combo, current_kind)
            else:
                self._populate_action(action_combo, current_kind)
            action_combo.setEnabled(
                current_kind in {PhaseKind.TRICK, PhaseKind.POSTURE, PhaseKind.MOVE}
            )
            speed_spin.setEnabled(current_kind in {PhaseKind.MOVE, PhaseKind.TAG_TARGET})
            return_to_start_check.setEnabled(current_kind is PhaseKind.TRICK)
            target_name_edit.setEnabled(current_kind is PhaseKind.TAG_TARGET)
            target_x_spin.setEnabled(current_kind is PhaseKind.TAG_TARGET)
            target_y_spin.setEnabled(current_kind is PhaseKind.TAG_TARGET)
            update_speed_limit()
            update_duration_mode()

        kind_combo.currentIndexChanged.connect(update_enabled)
        action_combo.currentIndexChanged.connect(update_speed_limit)
        action_combo.currentIndexChanged.connect(update_duration_mode)
        update_enabled()
        if normalized_kind is PhaseKind.MOVE:
            speed_spin.setValue(min(speed if speed > 0 else 0.25, speed_spin.maximum()))
        elif normalized_kind is PhaseKind.TAG_TARGET:
            speed_spin.setValue(min(200.0, max(70.0, speed if speed > 0 else 100.0)))
        self.table.selectRow(row)

    def steps(self) -> list[DanceStep]:
        result: list[DanceStep] = []
        for row in range(self.table.rowCount()):
            kind = PhaseKind(self.table.cellWidget(row, 2).currentData())
            action_data = self.table.cellWidget(row, 3).currentData()
            if (kind is PhaseKind.MOVE and action_data == MoveDirection.FORWARD.value
                    and self.table.cellWidget(row, 0).property("legacy_forward")):
                kind = PhaseKind.FORWARD
            targets = self.table.cellWidget(row, 1).targets()
            result.append(DanceStep(
                kind=kind,
                command=(
                    Command(action_data)
                    if kind in {PhaseKind.TRICK, PhaseKind.POSTURE}
                    else None
                ),
                duration=self.table.cellWidget(row, 4).value(),
                speed=(
                    self.table.cellWidget(row, 5).value()
                    if kind in {PhaseKind.MOVE, PhaseKind.FORWARD, PhaseKind.TAG_TARGET}
                    else 0.0
                ),
                start_time=self.table.cellWidget(row, 0).value(),
                targets=targets,
                direction=MoveDirection(action_data) if kind is PhaseKind.MOVE else MoveDirection.FORWARD,
                return_to_start=self.table.cellWidget(row, 6).isChecked(),
                target_name=self.table.cellWidget(row, 7).text().strip(),
                target_x=self.table.cellWidget(row, 8).value(),
                target_y=self.table.cellWidget(row, 9).value(),
            ))
        return result

    def rebuild(self, steps: list[DanceStep], selected_row: int) -> None:
        self.table.setRowCount(0)
        for step in steps:
            self.add_step(
                step.kind, step.command, step.duration, step.speed,
                step.start_time, step.targets, step.direction,
                step.target_x, step.target_y, step.target_name,
                step.return_to_start,
            )
        if steps:
            self.table.selectRow(max(0, min(selected_row, len(steps) - 1)))

    def move_step(self, direction: int) -> None:
        row, target = self.table.currentRow(), self.table.currentRow() + direction
        if row < 0 or target < 0 or target >= self.table.rowCount():
            return
        steps = self.steps()
        steps[row], steps[target] = steps[target], steps[row]
        self.rebuild(steps, target)

    def remove_step(self, checked: bool = False) -> None:
        row = self.table.currentRow()
        if row >= 0:
            self.table.removeRow(row)

    def add_wave_preset(self) -> None:
        base = self.next_start_time()
        for index, robot in enumerate(self.robots):
            self.add_step(
                PhaseKind.TRICK, command=Command.PRONK, duration=0.8,
                start_time=base + index * 0.45, targets=(robot.robot_id,),
            )

    def add_formation_preset(self) -> None:
        base = self.next_start_time()
        lateral_speed, forward_speed = min(0.12, self.max_y), min(0.25, self.max_x)
        for index, robot in enumerate(self.robots):
            direction = MoveDirection.LEFT if index % 2 == 0 else MoveDirection.RIGHT
            self.add_step(
                PhaseKind.MOVE, duration=1.2, speed=lateral_speed,
                start_time=base, targets=(robot.robot_id,), direction=direction,
            )
            self.add_step(
                PhaseKind.MOVE, duration=1.5, speed=forward_speed,
                start_time=base + 1.3, targets=(robot.robot_id,),
                direction=MoveDirection.FORWARD,
            )
