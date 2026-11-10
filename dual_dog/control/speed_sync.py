from __future__ import annotations

from dataclasses import dataclass, field
import math
from statistics import median
from typing import Hashable, Mapping, Sequence

from dual_dog.models import Telemetry


class ActuationPulse:
    """Represent sub-threshold commands by effective pulses and zero commands."""

    def __init__(self):
        self.credit = 0.0
        self.direction = 0

    def reset(self):
        self.credit = 0.0
        self.direction = 0

    def apply(self, value: float, minimum: float) -> float:
        direction = 1 if value > 0.0 else -1 if value < 0.0 else 0
        if direction != self.direction:
            self.reset()
            self.direction = direction
        if abs(value) >= minimum or direction == 0:
            self.credit = 0.0
            return value
        self.credit += abs(value) / minimum
        if self.credit >= 1.0:
            self.credit -= 1.0
            return math.copysign(minimum, value)
        return 0.0


class VelocitySynchronizer:
    """Per-robot PI loop that makes measured body velocity follow one target."""

    KP = 0.45
    KI = 0.80
    INTEGRAL_LIMIT = 0.25
    MAX_TRIM = 0.20
    FILTER_ALPHA = 0.30

    def __init__(self) -> None:
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.previous_time: float | None = None
        self.previous_targets = (0.0, 0.0)
        self.sample_token = None
        self.last_output = (0.0, 0.0, "速度同步：待命")

    def reset(self) -> None:
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.previous_time = None
        self.previous_targets = (0.0, 0.0)
        self.sample_token = None

    def apply(
        self,
        target_x: float,
        target_y: float,
        measured_velocity,
        now: float,
        *,
        limits: tuple[float, float] = (float("inf"), float("inf")),
        sample_token=None,
    ) -> tuple[float, float, str]:
        if measured_velocity is None:
            self.reset()
            return target_x, target_y, "速度同步：等待腿部里程计"
        measured_x, measured_y = float(measured_velocity[0]), float(measured_velocity[1])
        if not all(math.isfinite(value) for value in (measured_x, measured_y)):
            self.reset()
            return target_x, target_y, "速度同步：测量无效"

        targets = (target_x, target_y)
        if targets != self.previous_targets:
            for index, (old, new) in enumerate(zip(self.previous_targets, targets)):
                if old * new <= 0.0 or abs(new) < abs(old) * 0.5:
                    if index == 0:
                        self.integral_x = 0.0
                    else:
                        self.integral_y = 0.0
        elif sample_token is not None and sample_token == self.sample_token:
            return self.last_output
        fresh = sample_token is None or sample_token != self.sample_token
        self.sample_token = sample_token
        self.previous_targets = targets
        dt = 0.05 if self.previous_time is None else max(0.0, min(0.2, now - self.previous_time))
        # A command refresh is not a new sensor measurement.
        if not fresh:
            dt = 0.0
        alpha = 1.0 - math.exp(-dt / 0.10)
        if self.previous_time is None:
            # Do not treat the first real sample as a transition from zero;
            # that creates the same artificial acceleration on every robot.
            self.filtered_x = measured_x
            self.filtered_y = measured_y
        else:
            self.filtered_x = alpha * measured_x + (1.0 - alpha) * self.filtered_x
            self.filtered_y = alpha * measured_y + (1.0 - alpha) * self.filtered_y
        if fresh:
            self.previous_time = now

        def axis(
            target: float, measured: float, integral: float, limit: float
        ) -> tuple[float, float, float]:
            if abs(target) < 1e-6:
                return 0.0, 0.0, 0.0
            error = target - measured
            candidate = max(
                -self.INTEGRAL_LIMIT,
                min(self.INTEGRAL_LIMIT, integral + error * dt),
            )
            raw_trim = self.KP * error + self.KI * candidate
            trim = max(
                -self.MAX_TRIM,
                min(self.MAX_TRIM, raw_trim),
            )
            output = max(-limit, min(limit, target + trim))
            # Never reverse the requested direction to reduce a speed error.
            if output * target < 0.0:
                output = 0.0
            if (target + raw_trim - output) * error <= 0.0:
                integral = candidate
            return output, integral, error

        command_x, self.integral_x, error_x = axis(
            target_x, self.filtered_x, self.integral_x, limits[0]
        )
        command_y, self.integral_y, error_y = axis(
            target_y, self.filtered_y, self.integral_y, limits[1]
        )
        status = (
            f"速度同步 目标=({target_x:+.2f},{target_y:+.2f}) "
            f"实测=({self.filtered_x:+.2f},{self.filtered_y:+.2f}) "
            f"误差=({error_x:+.2f},{error_y:+.2f})"
        )
        self.last_output = (command_x, command_y, status)
        return self.last_output


@dataclass
class _ProgressSession:
    origins: dict[str, tuple[float, float]] = field(default_factory=dict)
    directions_world: dict[str, tuple[float, float]] = field(default_factory=dict)
    integrals: dict[str, float] = field(default_factory=dict)
    previous_trims: dict[str, float] = field(default_factory=dict)
    previous_time: float | None = None
    saturated_since: dict[str, float] = field(default_factory=dict)
    speed_ceiling: float | None = None


class FormationProgressSynchronizer:
    """Keep robots at the same relative travel progress within one movement cue.

    The per-robot :class:`VelocitySynchronizer` closes the fast local velocity
    loop.  This slower outer loop compares projected distance travelled against
    the group median and trims each robot's requested speed accordingly.
    """

    KP = 0.80
    KI = 0.20
    INTEGRAL_LIMIT = 0.25
    MAX_TRIM = 0.12
    TRIM_SLEW_RATE = 0.35
    ERROR_DEADBAND_M = 0.008
    UNCERTAINTY_LIMIT_M = 0.08

    def __init__(self) -> None:
        self._sessions: dict[Hashable, _ProgressSession] = {}

    def reset(self, session_key: Hashable | None = None) -> None:
        if session_key is None:
            self._sessions.clear()
        else:
            self._sessions.pop(session_key, None)

    @staticmethod
    def _unchanged(
        targets: Sequence[str], target_x: float, target_y: float, status: str
    ) -> dict[str, tuple[float, float, str]]:
        return {
            robot_id: (target_x, target_y, status)
            for robot_id in targets
        }

    def apply(
        self,
        targets: Sequence[str],
        target_x: float,
        target_y: float,
        snapshots: Mapping[str, Telemetry],
        now: float,
        session_key: Hashable = "default",
        limits: tuple[float, float] | None = None,
    ) -> dict[str, tuple[float, float, str]]:
        robot_ids = tuple(dict.fromkeys(targets))
        target_speed = math.hypot(target_x, target_y)
        if len(robot_ids) < 2 or target_speed < 1e-6:
            self.reset(session_key)
            return self._unchanged(
                robot_ids, target_x, target_y, "队形同步：至少需要两台平移设备"
            )
        if not all(math.isfinite(value) for value in (target_x, target_y, now)):
            self.reset(session_key)
            return self._unchanged(robot_ids, target_x, target_y, "队形同步：输入无效")

        valid: dict[str, Telemetry] = {}
        for robot_id in robot_ids:
            data = snapshots.get(robot_id)
            if data is None or not data.connected or data.corrected_pose is None:
                continue
            if now - data.updated_at > 0.5:
                continue
            if not all(math.isfinite(float(value)) for value in data.corrected_pose):
                continue
            if (
                data.odom_uncertainty is not None
                and max(data.odom_uncertainty[:2]) > self.UNCERTAINTY_LIMIT_M
            ):
                continue
            valid[robot_id] = data
        if len(valid) < 2:
            self.reset(session_key)
            return self._unchanged(
                robot_ids, target_x, target_y, "队形同步：等待至少两台有效里程计"
            )

        direction_body = (target_x / target_speed, target_y / target_speed)
        session = self._sessions.setdefault(session_key, _ProgressSession())
        if set(session.origins) != set(valid):
            session = _ProgressSession()
            self._sessions[session_key] = session
        if not session.origins:
            for robot_id, data in valid.items():
                position_x, position_y, yaw_degrees = data.corrected_pose
                yaw = math.radians(yaw_degrees)
                cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
                session.origins[robot_id] = (position_x, position_y)
                session.directions_world[robot_id] = (
                    cos_yaw * direction_body[0] - sin_yaw * direction_body[1],
                    sin_yaw * direction_body[0] + cos_yaw * direction_body[1],
                )
                session.integrals[robot_id] = 0.0
                session.previous_trims[robot_id] = 0.0
            session.previous_time = now
            return self._unchanged(
                robot_ids, target_x, target_y, "队形同步：相对进度已锁定"
            )

        dt = 0.05 if session.previous_time is None else max(
            0.02, min(0.2, now - session.previous_time)
        )
        session.previous_time = now
        requested_speed = target_speed
        if limits is not None:
            capacities = [target_speed]
            for robot_id, data in valid.items():
                command = data.commanded_velocity
                velocity = data.corrected_velocity_world
                saturated = command is not None and any(
                    abs(direction_body[i]) > 1e-6
                    and abs(command[i]) >= limits[i] * 0.98 for i in (0, 1)
                )
                if not saturated or velocity is None or not all(math.isfinite(v) for v in velocity):
                    session.saturated_since.pop(robot_id, None)
                    continue
                since = session.saturated_since.setdefault(robot_id, now)
                direction = session.directions_world[robot_id]
                measured = sum(velocity[i] * direction[i] for i in (0, 1))
                if now - since >= 0.6 and measured > 0.03:
                    capacities.append(measured)
            desired_ceiling = min(capacities)
            previous = session.speed_ceiling if session.speed_ceiling is not None else target_speed
            session.speed_ceiling = min(target_speed, max(
                previous - 0.15 * dt, min(previous + 0.02 * dt, desired_ceiling)
            ))
            target_speed = session.speed_ceiling
        progress: dict[str, float] = {}
        for robot_id, data in valid.items():
            position_x, position_y, _ = data.corrected_pose
            origin_x, origin_y = session.origins[robot_id]
            direction_x, direction_y = session.directions_world[robot_id]
            progress[robot_id] = (
                (position_x - origin_x) * direction_x
                + (position_y - origin_y) * direction_y
            )
        reference = median(progress.values())

        results = self._unchanged(
            robot_ids, target_x, target_y, "队形同步：该设备里程计不可用"
        )
        for robot_id, travelled in progress.items():
            error = reference - travelled
            pid_error = 0.0 if abs(error) <= self.ERROR_DEADBAND_M else error
            if pid_error == 0.0:
                session.integrals[robot_id] *= 0.9
            else:
                session.integrals[robot_id] = max(
                    -self.INTEGRAL_LIMIT,
                    min(
                        self.INTEGRAL_LIMIT,
                        session.integrals[robot_id] + pid_error * dt,
                    ),
                )
            requested_trim = max(
                -self.MAX_TRIM,
                min(
                    self.MAX_TRIM,
                    self.KP * pid_error + self.KI * session.integrals[robot_id],
                ),
            )
            previous_trim = session.previous_trims[robot_id]
            max_change = self.TRIM_SLEW_RATE * dt
            trim = max(
                previous_trim - max_change,
                min(previous_trim + max_change, requested_trim),
            )
            # Never reverse a robot to fix a progress error.  Slowing the leader
            # and accelerating the laggard is safer and visually smoother.
            adjusted_speed = max(0.0, target_speed + trim)
            session.previous_trims[robot_id] = trim
            results[robot_id] = (
                direction_body[0] * adjusted_speed,
                direction_body[1] * adjusted_speed,
                f"队形同步 进度={travelled:+.3f}m 中位={reference:+.3f}m "
                f"差={error:+.3f}m 调速={trim:+.3f}m/s "
                f"公共速度={target_speed:.2f}/{requested_speed:.2f}m/s",
            )
        return results
