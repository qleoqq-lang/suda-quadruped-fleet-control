from __future__ import annotations
from typing import TYPE_CHECKING
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
ODOMETRY_SCALE_SPECS = (("forward","forward"),("backward","backward"),("left","left"),("right","right"),("turn_left","turn_left"),("turn_right","turn_right"))
if TYPE_CHECKING:
    from dual_dog.ui.gui import MainWindow
class OdometryCalibrationWindow(QWidget):
    def __init__(self, owner):
        super().__init__(None)
        self.owner = owner
        self.setWindowTitle("Odometry")
        self.resize(980, 720)
        self.setMinimumSize(860, 640)
        layout = QVBoxLayout(self)
        status_box = QGroupBox("Fused odometry")
        status_layout = QGridLayout(status_box)
        self.odometry_robot = QComboBox()
        for config in owner.configs:
            self.odometry_robot.addItem(f"{config.name} ({config.robot_id})", config.robot_id)
        self.odometry_pose = QLabel("x=-- m    y=-- m    yaw=--")
        self.odometry_fusion = QLabel("EKF waiting")
        status_layout.addWidget(QLabel("Robot"), 0, 0)
        status_layout.addWidget(self.odometry_robot, 0, 1)
        status_layout.addWidget(self.odometry_pose, 0, 2, 1, 3)
        status_layout.addWidget(self.odometry_fusion, 1, 0, 1, 5)
        layout.addWidget(status_box)
        middle = QHBoxLayout()
        correction_box = QGroupBox("Pose correction")
        correction_layout = QGridLayout(correction_box)
        self.odometry_pose_inputs = []
        self.odometry_delta_inputs = []
        correction_layout.addWidget(QLabel("axis"), 0, 0)
        correction_layout.addWidget(QLabel("pose"), 0, 1)
        correction_layout.addWidget(QLabel("delta"), 0, 2)
        specs = (("x", -10000.0, 10000.0, 3), ("y", -10000.0, 10000.0, 3), ("yaw", -36000.0, 36000.0, 2))
        for row, (label, minimum, maximum, decimals) in enumerate(specs, start=1):
            pose_input = QDoubleSpinBox(); pose_input.setRange(minimum, maximum); pose_input.setDecimals(decimals)
            delta_input = QDoubleSpinBox(); delta_input.setRange(minimum, maximum); delta_input.setDecimals(decimals)
            self.odometry_pose_inputs.append(pose_input)
            self.odometry_delta_inputs.append(delta_input)
            correction_layout.addWidget(QLabel(label), row, 0)
            correction_layout.addWidget(pose_input, row, 1)
            correction_layout.addWidget(delta_input, row, 2)
        set_pose_button = QPushButton("Set pose"); set_pose_button.setProperty("role", "primary")
        add_correction_button = QPushButton("Add correction")
        correction_layout.addWidget(set_pose_button, 4, 1)
        correction_layout.addWidget(add_correction_button, 4, 2)
        middle.addWidget(correction_box, 1)
        scale_box = QGroupBox("Scale factors")
        scale_layout = QGridLayout(scale_box)
        self.odometry_scale_inputs = {}
        for index, (key, label) in enumerate(ODOMETRY_SCALE_SPECS):
            editor = QDoubleSpinBox(); editor.setRange(0.5, 1.5); editor.setDecimals(4); editor.setSingleStep(0.001); editor.setValue(1.0)
            self.odometry_scale_inputs[key] = editor
            scale_layout.addWidget(QLabel(label), index // 3, (index % 3) * 2)
            scale_layout.addWidget(editor, index // 3, (index % 3) * 2 + 1)
        apply_scales_button = QPushButton("Save scales"); apply_scales_button.setProperty("role", "primary")
        scale_layout.addWidget(apply_scales_button, 2, 0, 1, 6)
        middle.addWidget(scale_box, 1)
        layout.addLayout(middle)
        lateral_box = QGroupBox("二维直线轨迹 PID")
        lateral_layout = QGridLayout(lateral_box)
        self.lateral_feedback_inputs = {}
        specs2 = (("kp", "Kp(位置)", 0.0, 5.0, 3), ("ki", "Ki", 0.0, 2.0, 3), ("kd", "Kv(横速)", 0.0, 2.0, 3), ("max_correction", "补偿限幅", 0.01, min(owner.safety.max_x, owner.safety.max_y), 3))
        for column, (key, label, minimum, maximum, decimals) in enumerate(specs2):
            editor = QDoubleSpinBox(); editor.setRange(minimum, maximum); editor.setDecimals(decimals); editor.setSingleStep(0.01)
            if key == "max_correction":
                editor.setSuffix(" m/s")
            self.lateral_feedback_inputs[key] = editor
            lateral_layout.addWidget(QLabel(label), 0, column * 2)
            lateral_layout.addWidget(editor, 0, column * 2 + 1)
        apply_lateral_button = QPushButton("保存轨迹 PID"); apply_lateral_button.setProperty("role", "primary")
        lateral_layout.addWidget(apply_lateral_button, 0, 8)
        self.odometry_lateral_status = QLabel("轨迹纠偏：待命")
        self.odometry_lateral_status.setWordWrap(True)
        lateral_layout.addWidget(self.odometry_lateral_status, 1, 0, 1, 9)
        layout.addWidget(lateral_box)
        help_label = QLabel("锁定二维直线：前后走修正左右偏移，横移修正前后偏移；误差在 ±2 cm 内停止补偿。")
        help_label.setWordWrap(True)
        layout.addWidget(help_label)
        layout.addStretch()
        self.odometry_robot.currentIndexChanged.connect(owner.odometry_robot_changed)
        set_pose_button.clicked.connect(owner.set_odometry_pose)
        add_correction_button.clicked.connect(owner.add_odometry_correction)
        apply_scales_button.clicked.connect(owner.apply_odometry_scales)
        apply_lateral_button.clicked.connect(owner.apply_lateral_feedback_config)
    def closeEvent(self, event: QCloseEvent) -> None:
        event.accept()
