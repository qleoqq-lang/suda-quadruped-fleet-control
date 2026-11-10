from __future__ import annotations

from dataclasses import dataclass
import math
import time

from dual_dog.models import SafetyConfig, Telemetry


def wrap_angle(angle: float) -> float:
    """Return an angle in [-pi, pi), safe across the IMU yaw boundary."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def yaw_calibration_offset(raw_yaw: float, common_yaw: float = 0.0) -> float:
    """Offset that maps one robot's raw IMU yaw into a shared heading frame."""
    return wrap_angle(common_yaw - raw_yaw)


def raw_target_for_common_yaw(common_target: float, offset: float) -> float:
    """Convert a shared-frame target into one robot's raw IMU yaw target."""
    return wrap_angle(common_target - offset)


def common_yaw_from_raw(raw_yaw: float, offset: float) -> float:
    return wrap_angle(raw_yaw + offset)


@dataclass(frozen=True)
class FeedbackResult:
    x: float
    y: float
    yaw: float
    status: str
    heading_error: float = 0.0


class YawPid:
    """PID for wrapped yaw angles with anti-windup and filtered derivative."""

    def __init__(self, safety: SafetyConfig) -> None:
        self.safety = safety
        self.target: float | None = None
        self.integral = 0.0
        self.previous_error: float | None = None
        self.filtered_derivative = 0.0
        self.previous_time: float | None = None

    def reset(self) -> None:
        self.target = None
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0
        self.previous_time = None

    def lock(self, yaw: float, now: float) -> None:
        self.reset()
        self.target = wrap_angle(yaw)
        self.previous_time = now

    def set_target(self, yaw: float, now: float) -> None:
        target = wrap_angle(yaw)
        if self.target is None or abs(wrap_angle(target - self.target)) > 1e-6:
            self.lock(target, now)

    def update(
        self,
        current_yaw: float,
        now: float,
        measured_yaw_rate: float | None = None,
    ) -> tuple[float, float]:
        if self.target is None:
            self.lock(current_yaw, now)
        error = wrap_angle(self.target - current_yaw)
        if abs(error) < self.safety.feedback_deadband_rad:
            error = 0.0
            self.integral *= 0.9

        dt = 0.1 if self.previous_time is None else max(
            0.02, min(0.2, now - self.previous_time)
        )
        raw_derivative = (
            -float(measured_yaw_rate)
            if measured_yaw_rate is not None and math.isfinite(measured_yaw_rate)
            else (
                0.0
                if self.previous_error is None
                else wrap_angle(error - self.previous_error) / dt
            )
        )
        alpha = self.safety.yaw_derivative_alpha
        self.filtered_derivative = (
            alpha * raw_derivative + (1.0 - alpha) * self.filtered_derivative
        )

        candidate_integral = clamp(
            self.integral + error * dt,
            self.safety.yaw_integral_limit,
        )
        candidate_output = (
            self.safety.yaw_kp * error
            + self.safety.yaw_ki * candidate_integral
            + self.safety.yaw_kd * self.filtered_derivative
        )
        output = clamp(candidate_output, self.safety.max_yaw_correction)
        saturated = abs(candidate_output) > self.safety.max_yaw_correction
        drives_further = candidate_output * error > 0.0
        if not (saturated and drives_further):
            self.integral = candidate_integral

        self.previous_error = error
        self.previous_time = now
        return output, error


class StraightLineTracker:
    """Hold yaw and optionally use fused odometry to stay on a straight line."""

    def __init__(self, safety: SafetyConfig) -> None:
        self.safety = safety
        self.pid = YawPid(safety)
        self.lateral_kp = safety.lateral_kp
        self.lateral_ki = safety.lateral_ki
        self.lateral_kd = safety.lateral_kd
        self.max_lateral_correction = safety.max_lateral_correction
        self.line_origin: tuple[float, float] | None = None
        self.line_direction: tuple[float, float] | None = None
        self.motion_direction_body: tuple[float, float] | None = None
        self.lateral_integral = 0.0
        self.previous_lateral_error: float | None = None
        self.filtered_lateral_derivative = 0.0
        self.previous_lateral_time: float | None = None
        self.previous_lateral_output = 0.0
        self.line_locked_at: float | None = None
        self.filtered_path_error: float | None = None
        self.filtered_cross_velocity = 0.0
        self.lateral_fault = False
        self._yaw_pulse_credit = 0.0
        self._yaw_pulse_direction = 0

    def reset(self) -> None:
        self.pid.reset()
        self._yaw_pulse_credit = 0.0
        self._yaw_pulse_direction = 0
        self._reset_lateral(clear_fault=True)

    def _stationary_yaw(self, error: float, rate: float) -> float:
        """Predict coast-down, then quantize low rates into effective pulses."""
        if abs(error) <= self.safety.feedback_deadband_rad:
            self._yaw_pulse_credit = 0.0
            return 0.0
        predicted = error - rate * 0.16
        if predicted * error <= 0.0:
            self._yaw_pulse_credit = 0.0
            return 0.0
        desired = math.copysign(min(
            self.safety.max_yaw,
            3.0 * abs(predicted),
            math.sqrt(2.0 * 0.8 * abs(predicted)),
        ), predicted)
        direction = 1 if desired > 0.0 else -1
        if direction != self._yaw_pulse_direction:
            self._yaw_pulse_credit = 0.0
            self._yaw_pulse_direction = direction
        minimum = self.safety.min_yaw_actuation
        if abs(desired) >= minimum:
            self._yaw_pulse_credit = 0.0
            return desired
        self._yaw_pulse_credit += abs(desired) / minimum
        if self._yaw_pulse_credit >= 1.0:
            self._yaw_pulse_credit -= 1.0
            return math.copysign(minimum, desired)
        return 0.0

    def _reset_lateral(self, clear_fault: bool = False) -> None:
        self.line_origin = None
        self.line_direction = None
        self.motion_direction_body = None
        self.lateral_integral = 0.0
        self.previous_lateral_error = None
        self.filtered_lateral_derivative = 0.0
        self.previous_lateral_time = None
        self.previous_lateral_output = 0.0
        self.line_locked_at = None
        self.filtered_path_error = None
        self.filtered_cross_velocity = 0.0
        if clear_fault:
            self.lateral_fault = False

    def configure_lateral(
        self, kp: float, ki: float, kd: float, max_correction: float
    ) -> None:
        values = (float(kp), float(ki), float(kd), float(max_correction))
        if any(not math.isfinite(value) for value in values):
            raise ValueError("轨迹纠偏参数必须是有限数字")
        if min(values[:3]) < 0:
            raise ValueError("轨迹纠偏 PID 增益不能为负数")
        if not 0 < values[3] <= min(self.safety.max_x, self.safety.max_y):
            raise ValueError("轨迹补偿限幅必须大于 0 且不超过平移速度硬限幅")
        self.lateral_kp, self.lateral_ki, self.lateral_kd = values[:3]
        self.max_lateral_correction = values[3]
        self._reset_lateral(clear_fault=True)

    def _path_correction(
        self,
        x_command: float,
        y_command: float,
        telemetry: Telemetry,
        current_time: float,
        target_yaw: float | None,
    ) -> tuple[float, float, str]:
        if telemetry.corrected_pose is None:
            self._reset_lateral(clear_fault=False)
            return 0.0, 0.0, "轨迹[等待里程计]"
        if telemetry.odom_uncertainty is not None and max(
            telemetry.odom_uncertainty[:2]
        ) > self.safety.lateral_uncertainty_limit_m:
            self.lateral_integral = 0.0
            self.previous_lateral_output = 0.0
            self.previous_lateral_time = current_time
            return 0.0, 0.0, "轨迹[里程计置信度不足]"
        if self.lateral_fault:
            return 0.0, 0.0, "轨迹[偏差过大，松开运动键后重试]"

        position_x, position_y, pose_yaw_degrees = telemetry.corrected_pose
        pose_yaw = math.radians(pose_yaw_degrees)
        command_magnitude = math.hypot(x_command, y_command)
        if command_magnitude < 1e-6:
            self._reset_lateral(clear_fault=True)
            return 0.0, 0.0, "轨迹[无平移指令]"
        direction_body = (
            x_command / command_magnitude,
            y_command / command_magnitude,
        )
        if (
            self.motion_direction_body is not None
            and direction_body[0] * self.motion_direction_body[0]
            + direction_body[1] * self.motion_direction_body[1]
            < 0.999
        ):
            self._reset_lateral(clear_fault=True)
        if self.line_origin is None:
            body_heading = pose_yaw
            if target_yaw is not None and telemetry.rpy is not None:
                body_heading = wrap_angle(
                    pose_yaw + wrap_angle(target_yaw - telemetry.rpy[2])
                )
            cos_heading = math.cos(body_heading)
            sin_heading = math.sin(body_heading)
            line_direction = (
                cos_heading * direction_body[0]
                - sin_heading * direction_body[1],
                sin_heading * direction_body[0]
                + cos_heading * direction_body[1],
            )
            self.line_origin = (position_x, position_y)
            self.line_direction = line_direction
            self.motion_direction_body = direction_body
            self.previous_lateral_time = current_time
            self.line_locked_at = current_time
            return 0.0, 0.0, "轨迹[直线已锁定] 横差=+0.000m"

        assert self.line_direction is not None
        delta_x = position_x - self.line_origin[0]
        delta_y = position_y - self.line_origin[1]
        normal_x = -self.line_direction[1]
        normal_y = self.line_direction[0]
        error = normal_x * delta_x + normal_y * delta_y
        if abs(error) > self.safety.max_lateral_error_m:
            self._reset_lateral(clear_fault=False)
            self.lateral_fault = True
            return 0.0, 0.0, f"轨迹[偏差过大，已停用] 横差={error:+.3f}m"
        if self.line_locked_at is not None and current_time - self.line_locked_at < 0.30:
            self.previous_lateral_time = current_time
            return 0.0, 0.0, f"轨迹[观测稳定中] 横差={error:+.3f}m"

        dt = 0.05 if self.previous_lateral_time is None else max(
            0.02, min(0.2, current_time - self.previous_lateral_time)
        )
        filter_alpha = 0.25
        self.filtered_path_error = (
            error
            if self.filtered_path_error is None
            else filter_alpha * error + (1.0 - filter_alpha) * self.filtered_path_error
        )
        pid_error = self.filtered_path_error
        cross_velocity = 0.0
        if telemetry.corrected_velocity_world is not None:
            velocity_x, velocity_y = telemetry.corrected_velocity_world
            cross_velocity = normal_x * velocity_x + normal_y * velocity_y
        velocity_alpha = self.safety.lateral_derivative_alpha
        self.filtered_cross_velocity = (
            velocity_alpha * cross_velocity
            + (1.0 - velocity_alpha) * self.filtered_cross_velocity
        )
        if abs(error) <= self.safety.lateral_deadband_m:
            pid_error = 0.0
            self.lateral_integral *= 0.9
        candidate_integral = clamp(
            self.lateral_integral + pid_error * dt,
            self.safety.lateral_integral_limit,
        )
        raw_output = -(
            self.lateral_kp * pid_error
            + self.lateral_ki * candidate_integral
            + self.lateral_kd * self.filtered_cross_velocity
        )
        limited_output = clamp(raw_output, self.max_lateral_correction)
        if abs(raw_output) <= self.max_lateral_correction or raw_output * pid_error > 0:
            self.lateral_integral = candidate_integral

        max_change = self.safety.lateral_slew_rate * dt
        limited_output = max(
            self.previous_lateral_output - max_change,
            min(self.previous_lateral_output + max_change, limited_output),
        )
        limited_output = clamp(limited_output, self.max_lateral_correction)
        correction_world_x = limited_output * normal_x
        correction_world_y = limited_output * normal_y
        cos_pose = math.cos(pose_yaw)
        sin_pose = math.sin(pose_yaw)
        correction_body_x = (
            cos_pose * correction_world_x + sin_pose * correction_world_y
        )
        correction_body_y = (
            -sin_pose * correction_world_x + cos_pose * correction_world_y
        )
        self.previous_lateral_error = pid_error
        self.previous_lateral_time = current_time
        self.previous_lateral_output = limited_output
        state = (
            "到位"
            if pid_error == 0.0 and abs(self.filtered_cross_velocity) <= 0.01
            else "补偿"
        )
        return correction_body_x, correction_body_y, (
            f"轨迹[{state}] 横差={error:+.3f}m "
            f"横速={self.filtered_cross_velocity:+.3f}m/s "
            f"修正=({correction_body_x:+.3f},{correction_body_y:+.3f})m/s"
        )

    def apply(
        self,
        x: float,
        y: float,
        yaw: float,
        telemetry: Telemetry,
        now: float | None = None,
        target_yaw: float | None = None,
        lateral_correction: bool = False,
        path_correction: bool = False,
    ) -> FeedbackResult:
        current_time = time.monotonic() if now is None else now
        if (
            current_time - telemetry.updated_at > self.safety.telemetry_timeout_ms / 1000.0
            or (telemetry.rpy is not None and not all(math.isfinite(v) for v in telemetry.rpy))
        ):
            self.reset()
            return FeedbackResult(0.0, 0.0, 0.0, "遥测过期或无效，停止")
        has_explicit_target = target_yaw is not None
        correction_enabled = lateral_correction or path_correction
        if not self.safety.feedback_enabled or (
            abs(x) < 1e-6 and abs(y) < 1e-6 and not has_explicit_target
        ):
            self.reset()
            return FeedbackResult(x, y, yaw, "待命")
        manual_lateral = abs(y) > 1e-6
        if manual_lateral and not has_explicit_target and not correction_enabled:
            self.reset()
            return FeedbackResult(x, y, yaw, "人工横移")
        if abs(yaw) > 1e-6:
            self.reset()
            return FeedbackResult(x, y, yaw, "人工转向")
        if telemetry.rpy is None:
            self.reset()
            return FeedbackResult(x, y, 0.0, "缺少 IMU，未纠偏")

        if has_explicit_target:
            if not math.isfinite(target_yaw):
                self.reset()
                return FeedbackResult(x, y, 0.0, "目标 yaw 无效")
            if self.pid.target is not None and abs(
                wrap_angle(target_yaw - self.pid.target)
            ) > 1e-6:
                self._reset_lateral(clear_fault=True)
                self._yaw_pulse_credit = 0.0
            self.pid.set_target(target_yaw, current_time)

        if not correction_enabled:
            self._reset_lateral(clear_fault=True)

        measured_yaw_rate = telemetry.corrected_yaw_rate
        if measured_yaw_rate is None and telemetry.imu_angular_velocity is not None:
            measured_yaw_rate = telemetry.imu_angular_velocity[2]
        if measured_yaw_rate is not None and not math.isfinite(measured_yaw_rate):
            self.reset()
            return FeedbackResult(0.0, 0.0, 0.0, "角速度无效，停止")
        output, error = self.pid.update(
            telemetry.rpy[2], current_time, measured_yaw_rate
        )
        # Preserve the actual error: PID's internal deadband is not a measurement.
        error = wrap_angle(self.pid.target - telemetry.rpy[2])
        yaw_rate_ok = (
            measured_yaw_rate is None or abs(measured_yaw_rate) <= 0.04
        )
        stationary = has_explicit_target and abs(x) < 1e-6 and abs(y) < 1e-6
        tolerance = self.safety.feedback_deadband_rad
        angle_within_tolerance = abs(error) <= tolerance
        within_tolerance = angle_within_tolerance and yaw_rate_ok
        if within_tolerance:
            output = 0.0
        elif 0.0 < abs(output) < self.safety.min_yaw_actuation:
            # The BPX Walk controller ignores very small isolated yaw commands.
            # Keep the direction from the wrapped error and cross its actuation threshold.
            output = math.copysign(
                self.safety.min_yaw_actuation,
                error if abs(error) > 1e-9 else output,
            )
        if has_explicit_target and abs(x) < 1e-6 and abs(y) < 1e-6:
            output = self._stationary_yaw(error, measured_yaw_rate or 0.0)
        target = self.pid.target if self.pid.target is not None else telemetry.rpy[2]
        tracking_state = (
            "到位"
            if within_tolerance
            else ("角速度阻尼" if angle_within_tolerance else "微调")
        )
        status = (
            f"Yaw PID[{tracking_state}] 目标={math.degrees(target):+.1f}° "
            f"当前={math.degrees(telemetry.rpy[2]):+.1f}° "
            f"误差={math.degrees(error):+.1f}° 输出={output:+.3f}rad/s"
        )
        x_output = x
        y_output = y
        if correction_enabled and math.hypot(x, y) >= 1e-6:
            if abs(error) > math.radians(10.0):
                self._reset_lateral(clear_fault=False)
                path_status = "轨迹[等待航向误差进入 ±10°]"
            else:
                correction_x, correction_y, path_status = self._path_correction(
                    x, y, telemetry, current_time, target_yaw
                )
                x_output += correction_x
                y_output += correction_y
            status = f"{status}；{path_status}"
        return FeedbackResult(
            x=x_output,
            y=y_output,
            yaw=output,
            status=status,
            heading_error=error,
        )
