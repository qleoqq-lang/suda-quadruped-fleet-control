from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
import time


class Command(StrEnum):
    STAND = "stand"
    SIT = "sit"
    WALK = "walk"
    VELOCITY = "velocity"
    STOP = "stop"
    DAMPING = "damping"
    LEFT_FLIP = "left_flip"
    RIGHT_FLIP = "right_flip"
    PRONK = "pronk"
    PACE = "pace"
    BOUND = "bound"
    BIPEDAL = "bipedal"
    INV_BIPEDAL = "inv_bipedal"
    RUN = "run"
    UP_JUMP = "up_jump"
    FRONT_JUMP = "front_jump"
    BACK_JUMP = "back_jump"
    LEFT_JUMP = "left_jump"
    RIGHT_JUMP = "right_jump"
    UPRIGHT = "upright"


@dataclass(frozen=True)
class RobotConfig:
    robot_id: str
    name: str
    ip: str
    mac: str
    state_port: int
    session_id: int


@dataclass(frozen=True)
class SafetyConfig:
    max_x: float = 0.30
    max_y: float = 0.30
    max_yaw: float = 0.50
    min_lateral_actuation: float = 0.25
    command_timeout_ms: int = 350
    telemetry_timeout_ms: int = 1500
    low_battery_percent: int = 20
    critical_battery_percent: int = 10
    warn_temperature_c: float = 65.0
    stop_temperature_c: float = 75.0
    max_roll_deg: float = 35.0
    max_pitch_deg: float = 35.0
    feedback_enabled: bool = True
    yaw_kp: float = 1.60
    yaw_ki: float = 0.08
    yaw_kd: float = 0.12
    yaw_integral_limit: float = 0.40
    yaw_derivative_alpha: float = 0.25
    max_yaw_correction: float = 0.30
    min_yaw_actuation: float = 0.22
    feedback_deadband_rad: float = 0.017453292519943295
    lateral_kp: float = 0.70
    lateral_ki: float = 0.03
    lateral_kd: float = 0.45
    lateral_integral_limit: float = 0.04
    lateral_derivative_alpha: float = 0.30
    lateral_deadband_m: float = 0.02
    max_lateral_correction: float = 0.05
    max_lateral_error_m: float = 0.12
    lateral_uncertainty_limit_m: float = 0.08
    lateral_slew_rate: float = 0.45
    adaptive_slip_learning_rate: float = 0.40
    adaptive_slip_filter_alpha: float = 0.25
    adaptive_slip_deadband_m_s: float = 0.01
    adaptive_slip_min_speed_m_s: float = 0.08
    adaptive_slip_warmup_s: float = 0.35
    adaptive_slip_hard_limit: float = 0.24
    adaptive_slip_slew_rate: float = 0.35
    one_shot_trigger_ms: int = 100
    non_flip_action_seconds: float = 5.0
    one_shot_return_ms: int = 3000


@dataclass
class Telemetry:
    robot_id: str
    connected: bool = False
    action: str = "离线"
    battery: int | None = None
    rpy: tuple[float, float, float] | None = None
    odom_position: tuple[float, float, float] | None = None
    leg_velocity_body: tuple[float, float, float] | None = None
    leg_orientation: tuple[float, float, float, float] | None = None
    imu_acceleration: tuple[float, float, float] | None = None
    imu_angular_velocity: tuple[float, float, float] | None = None
    imu_quaternion: tuple[float, float, float, float] | None = None
    joint_position: tuple[float, ...] | None = None
    joint_velocity: tuple[float, ...] | None = None
    odometry_token: tuple[object, ...] | None = None
    corrected_pose: tuple[float, float, float] | None = None
    corrected_velocity_world: tuple[float, float] | None = None
    corrected_yaw_rate: float | None = None
    odom_uncertainty: tuple[float, float, float] | None = None
    odom_support_legs: int = 0
    leg_velocity_candidates_body: tuple[tuple[float, float], ...] | None = None
    leg_velocity_inliers: tuple[bool, ...] | None = None
    commanded_velocity: tuple[float, float, float] | None = None
    adaptive_slip_direction: str = ""
    adaptive_slip_cross_velocity: float | None = None
    adaptive_slip_bias: float = 0.0
    adaptive_slip_correction: tuple[float, float] | None = None
    motion_state: int | None = None
    gait: int | None = None
    sub_gait: int | None = None
    max_temperature: float | None = None
    updated_at: float = field(default_factory=time.monotonic)
    error: str | None = None
    protection: str = "正常"
    feedback: str = "待命"

    def copy(self) -> "Telemetry":
        return Telemetry(**vars(self))
