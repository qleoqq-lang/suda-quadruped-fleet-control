from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
import logging
import math
import queue
import threading
import time

from dual_dog.control.adaptive_slip import AdaptiveSlipCompensator
from dual_dog.devices.drivers import RobotDriver, degrees
from dual_dog.control.feedback import StraightLineTracker
from dual_dog.models import Command, RobotConfig, SafetyConfig, Telemetry
from dual_dog.control.odometry import CorrectedOdometry
from dual_dog.control.speed_sync import ActuationPulse, VelocitySynchronizer


LOG = logging.getLogger(__name__)
RECONNECT_WATCHDOG_INTERVAL_SECONDS = 30.0
RECONNECT_WATCHDOG_MAX_INTERVAL_SECONDS = 120.0
RECONNECT_WATCHDOG_JITTER_SECONDS_PER_ROBOT = 0.75

ONE_SHOT_COMMANDS = {
    Command.LEFT_FLIP,
    Command.RIGHT_FLIP,
    Command.PRONK,
    Command.PACE,
    Command.BOUND,
    Command.BIPEDAL,
    Command.INV_BIPEDAL,
    Command.RUN,
    Command.UP_JUMP,
    Command.FRONT_JUMP,
    Command.BACK_JUMP,
    Command.LEFT_JUMP,
    Command.RIGHT_JUMP,
    Command.UPRIGHT,
}
FLIP_COMMANDS = {Command.LEFT_FLIP, Command.RIGHT_FLIP}
ONE_SHOT_GAIT_COMMANDS = {
    Command.PRONK,
    Command.PACE,
    Command.BOUND,
    Command.BIPEDAL,
    Command.INV_BIPEDAL,
    Command.RUN,
}
ONE_SHOT_DISCRETE_COMMANDS = {
    Command.UP_JUMP,
    Command.FRONT_JUMP,
    Command.BACK_JUMP,
    Command.LEFT_JUMP,
    Command.RIGHT_JUMP,
    Command.UPRIGHT,
}


def one_shot_recovery_seconds(command: Command, configured_ms: float) -> float:
    """Gait commands return directly to Walk after their visible duration."""
    if command in ONE_SHOT_GAIT_COMMANDS:
        return 0.0
    return max(0.0, float(configured_ms) / 1000.0)
# The BPX SDK example leaves roughly one second between setLeftFlip/
# setRightFlip and setWalk.  The old 100 ms configuration allowed Walk to
# cancel a flip request before the robot firmware had started the manoeuvre.
MIN_FLIP_COMMAND_WINDOW_SECONDS = 1.0
ONE_SHOT_MOTION_RECOVERY_GRACE_SECONDS = 1.0
POST_STUNT_YAW_COARSE_ERROR_RAD = math.radians(15.0)
POST_STUNT_YAW_FINE_ERROR_RAD = math.radians(5.0)
POST_STUNT_YAW_FINE_PULSE_PERIOD = 3
TAG_POSITION_TOLERANCE_M = 0.10
TAG_DIRECT_POSITION_TOLERANCE_M = 0.12
TAG_POSITION_REACQUIRE_M = TAG_POSITION_TOLERANCE_M
TAG_RETURN_PULSE_SECONDS = 0.35
TAG_MIN_RETURN_SPEED_M_S = 0.20
TAG_RETURN_DIVERGENCE_M = 0.05
TAG_DIRECT_RETURN_MAX_DISTANCE_M = 1.0
TAG_STUNT_RETURN_MAX_DISTANCE_M = 10.0
TAG_DIRECT_RETURN_NOMINAL_SPEED_M_S = 0.36
TAG_RETURN_MAX_DISTANCE_M = 1.0
TAG_TARGET_MAX_DISTANCE_M = 10.0
TAG_CONTINUOUS_ODOMETRY_HORIZON_SECONDS = 4.0
TAG_CONTINUOUS_ODOMETRY_UNCERTAINTY_M = 0.25
TAG_BRAKING_DECEL_M_S2 = 0.60
TAG_CONTROL_LATENCY_SECONDS = 0.16
TAG_BRAKING_MARGIN_M = 0.02
TAG_VELOCITY_KP = 0.35
TAG_VELOCITY_KI = 0.12
TAG_VELOCITY_INTEGRAL_LIMIT = 0.20
TAG_HIGH_LATENCY_THRESHOLD_SECONDS = 1.0
TAG_HIGH_LATENCY_MAX_BURST_DISTANCE_M = 0.28
TAG_HIGH_LATENCY_MIN_BURST_SECONDS = 0.16
TAG_HIGH_LATENCY_MAX_BURST_SECONDS = 1.10
TAG_RETURN_NOMINAL_SPEED_M_S = 0.45
TAG_RETURN_MAX_WINDOW_SECONDS = 30.0
TAG_RETURN_MIN_EXTENSION_SECONDS = 1.0

DIRECTION_LOCK_CHOICES = ("xy", "x", "y")


def direction_lock_mask(lock: object) -> tuple[bool, bool]:
    """Map a per-robot correction lock to (allow_x, allow_y)."""
    if lock == "x":
        return (True, False)
    if lock == "y":
        return (False, True)
    return (True, True)


def effective_flip_trigger_seconds(safety: SafetyConfig) -> float:
    """Return the minimum reliable SDK flip-to-Walk command window."""
    return max(
        MIN_FLIP_COMMAND_WINDOW_SECONDS,
        float(safety.one_shot_trigger_ms) / 1000.0,
    )


def effective_flip_lock_seconds(safety: SafetyConfig) -> float:
    """Timeline occupancy from flip acceptance through its safety lock."""
    return (
        effective_flip_trigger_seconds(safety)
        + float(safety.one_shot_return_ms) / 1000.0
    )


def post_stunt_yaw_command(
    error: float,
    pid_output: float,
    pulse_tick: int,
    safety: SafetyConfig,
) -> tuple[float, str]:
    """Shape post-stunt yaw into fast approach and pulse-width fine control."""
    magnitude = abs(error)
    if magnitude >= POST_STUNT_YAW_COARSE_ERROR_RAD:
        return math.copysign(safety.max_yaw, error), "快速拉近"
    if magnitude >= POST_STUNT_YAW_FINE_ERROR_RAD:
        return pid_output, "减速接近"
    if magnitude > safety.feedback_deadband_rad:
        # BPX ignores isolated yaw values below min_yaw_actuation.  Sparse
        # minimum-size pulses create a lower average correction without asking
        # the firmware for an ineffective tiny continuous command.
        if pulse_tick % POST_STUNT_YAW_FINE_PULSE_PERIOD == 0:
            return math.copysign(safety.min_yaw_actuation, error), "脉冲微调"
        return 0.0, "微调停顿"
    return 0.0, "稳定确认"


@dataclass(frozen=True)
class WorkItem:
    command: str | Command
    params: dict[str, float]
    delay_seconds: float = 0.0
    connect_generation: int = 0


class PositionPid:
    def __init__(
        self, kp: float = 0.9, ki: float = 0.06, kd: float = 0.10
    ) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.previous_error: float | None = None
        self.previous_time: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def update(self, error: float, now: float) -> float:
        dt = (
            0.1
            if self.previous_time is None
            else max(0.02, min(0.2, now - self.previous_time))
        )
        derivative = (
            0.0
            if self.previous_error is None
            else (error - self.previous_error) / dt
        )
        candidate = max(-0.20, min(0.20, self.integral + error * dt))
        output = self.kp * error + self.ki * candidate + self.kd * derivative
        if abs(output) < 0.12:
            self.integral = candidate
        self.previous_error = error
        self.previous_time = now
        return output


def effective_return_speed(
    value: float, error: float, minimum: float, maximum: float
) -> float:
    """Raise a non-zero PID command above the robot's velocity dead zone."""
    if abs(error) <= 0.02:
        return 0.0
    magnitude = max(minimum, min(maximum, abs(value)))
    return math.copysign(magnitude, error)


def braking_speed_limit(
    distance_to_tolerance: float,
    deceleration: float = TAG_BRAKING_DECEL_M_S2,
    latency_seconds: float = TAG_CONTROL_LATENCY_SECONDS,
    margin: float = TAG_BRAKING_MARGIN_M,
) -> float:
    """Maximum speed that can still stop before the position tolerance.

    Solve ``v * latency + v**2 / (2*a) <= available_distance`` for
    non-negative ``v``.  Keeping this planner separate from the PID prevents
    a delayed absolute position update from requesting full speed near zero.
    """
    if not all(
        math.isfinite(value)
        for value in (distance_to_tolerance, deceleration, latency_seconds, margin)
    ):
        return 0.0
    available = max(0.0, distance_to_tolerance - max(0.0, margin))
    if available <= 0.0 or deceleration <= 0.0:
        return 0.0
    latency = max(0.0, latency_seconds)
    a_latency = deceleration * latency
    return max(
        0.0,
        -a_latency + math.sqrt(a_latency * a_latency + 2.0 * deceleration * available),
    )


class RobotWorker:
    """Owns one SDK object and serializes every call made to it."""

    def __init__(self, driver: RobotDriver, safety: SafetyConfig) -> None:
        self.driver = driver
        self.safety = safety
        self._queue: queue.Queue[WorkItem] = queue.Queue()
        self._priority_queue: queue.Queue[WorkItem] = queue.Queue()
        self._connect_generation = 0
        self._reconnect_interval_seconds = RECONNECT_WATCHDOG_INTERVAL_SECONDS
        self._disconnected_since: float | None = None
        self._next_reconnect_at = 0.0
        self._stop_event = threading.Event()
        self._close_requested = threading.Event()
        self._shutdown_posture: Command | None = None
        self._lock = threading.Lock()
        self._snapshot = Telemetry(driver.config.robot_id)
        self._velocity_active = False
        self._last_velocity_command = 0.0
        self._last_commanded_velocity = (0.0, 0.0, 0.0)
        self._protection_status = "正常"
        self._feedback_status = "待命"
        self._tracker = StraightLineTracker(safety)
        self._speed_sync = VelocitySynchronizer()
        self._turn_speed_sync = VelocitySynchronizer()
        self._heading_last_aligned_sample = None
        self._lateral_speed_pulse = ActuationPulse()
        self._turn_speed_pulse = ActuationPulse()
        self._adaptive_slip = AdaptiveSlipCompensator(safety)
        self._adaptive_slip_direction = ""
        self._adaptive_slip_cross_velocity: float | None = None
        self._adaptive_slip_bias = 0.0
        self._adaptive_slip_correction = (0.0, 0.0)
        self._odometry = CorrectedOdometry()
        self._last_odometry_update: float | None = None
        self._last_odometry_token: tuple[object, ...] | None = None
        self._one_shot_walk_at: float | None = None
        self._one_shot_unlock_at: float | None = None
        self._one_shot_hard_unlock_at: float | None = None
        self._one_shot_recovery_complete = False
        self._one_shot_name: str | None = None
        self._one_shot_target_yaw: float | None = None
        self._next_one_shot_yaw = 0.0
        self._one_shot_is_flip = False
        self._one_shot_expected_sub_gait: int | None = None
        self._one_shot_acknowledged = False
        self._stunt_return_to_start = False
        self._stunt_tag_return_target: tuple[
            float, float, float, float
        ] | None = None
        self._heading_hold_active = False
        self._heading_hold_aligned = 0
        self._heading_fine_pulse_tick = 0
        self._heading_hold_started = 0.0
        self._freeze_odometry = False
        self._position_pid_x = PositionPid()
        self._position_pid_y = PositionPid()
        self._position_pid_yaw = PositionPid(kp=1.6, ki=0.04, kd=0.10)
        self._tag_position: tuple[float, float, float] | None = None
        self._tag_position_generation = 0
        self._tag_origin: tuple[float, float, float] | None = None
        self._tag_zero: tuple[float, float, float, float] | None = None
        self._tag_stage_heading_deg: float | None = None
        self._tag_return_speed_scale = 1.0
        self._tag_continuous_return = False
        self._tag_return_latency_seconds = 0.30
        self._tag_continuous_anchor: tuple[
            float, float, float, float, float
        ] | None = None
        self._tag_waiting_for_generation: int | None = None
        self._tag_velocity_integral_x = 0.0
        self._tag_velocity_integral_y = 0.0
        self._tag_velocity_previous_time: float | None = None
        self._tag_position_hold = False
        self._tag_auto_return_enabled = False
        self._tag_target_mode = False
        self._tag_direct_return = False
        self._tag_hold_arrived = False
        self._tag_return_pending = False
        self._tag_direction_lock = "xy"
        self._tag_hold_stale_seconds = 12.0
        self._tag_return_max_seconds = 4.0
        self._tag_return_nominal_seconds = 4.0
        self._tag_return_window_deadline: float | None = None
        self._tag_last_return_extension_at: float | None = None
        self._tag_last_extension_raw_distance: float | None = None
        self._tag_hold_started_at: float | None = None
        self._next_tag_hold = 0.0
        self._tag_last_control_generation = -1
        self._tag_pulse_stop_at = 0.0
        self._tag_latency_wait_until = 0.0
        self._tag_latency_burst_params: dict[str, float] | None = None
        self._tag_last_error_distance: float | None = None
        self._tag_divergence_ticks = 0
        self._thread = threading.Thread(
            target=self._run,
            name=f"robot-{driver.config.robot_id}",
            daemon=True,
        )
        self._thread.start()

    def submit(self, command: str | Command, **params: float) -> None:
        item = WorkItem(command, params)
        if command == "shutdown" or command in (
            ONE_SHOT_COMMANDS | {Command.STOP, Command.DAMPING}
        ):
            self._priority_queue.put(item)
        else:
            self._queue.put(item)

    def submit_connect(self, delay_seconds: float = 0.0) -> None:
        self._connect_generation += 1
        self._queue.put(
            WorkItem(
                "connect",
                {},
                max(0.0, float(delay_seconds)),
                self._connect_generation,
            )
        )

    def cancel_pending_connect(self) -> None:
        self._connect_generation += 1

    def snapshot(self) -> Telemetry:
        with self._lock:
            return self._snapshot.copy()

    def close(
        self, timeout: float = 3.0, final_posture: Command | None = None
    ) -> None:
        self.request_close(final_posture)
        self.wait_closed(timeout)

    def request_close(self, final_posture: Command | None = None) -> None:
        if not self._close_requested.is_set():
            self._close_requested.set()
            self._shutdown_posture = final_posture
            self._priority_queue.put(WorkItem("shutdown", {}))

    def wait_closed(self, timeout: float) -> None:
        self._thread.join(timeout=max(0.0, timeout))

    def _publish(self, data: Telemetry) -> None:
        with self._lock:
            self._snapshot = data

    def _set_error(self, message: str) -> None:
        data = self.snapshot()
        data.error = message
        data.protection = "故障"
        self._publish(data)
        LOG.error("%s: %s", self.driver.config.robot_id, message)

    def _run(self) -> None:
        next_poll = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            timeout = max(0.0, min(0.05, next_poll - now))
            try:
                try:
                    item = self._priority_queue.get_nowait()
                except queue.Empty:
                    item = self._queue.get(timeout=timeout)
                # Keep only the newest consecutive velocity command. This
                # prevents a stale mouse/keyboard direction from running late.
                if item.command == Command.VELOCITY:
                    while True:
                        try:
                            newer = self._queue.get_nowait()
                        except queue.Empty:
                            break
                        if newer.command == Command.VELOCITY:
                            item = newer
                        else:
                            self._queue.put(newer)
                            break
                if item.command == "shutdown":
                    self._shutdown()
                    return
                self._handle(item)
            except queue.Empty:
                pass
            except Exception as exc:
                self._set_error(str(exc))

            now = time.monotonic()
            if self._one_shot_walk_at is not None and now >= self._one_shot_walk_at:
                try:
                    if (
                        self._one_shot_expected_sub_gait is not None
                        and not self._one_shot_acknowledged
                    ):
                        current = self.snapshot()
                        LOG.error(
                            "%s %s 触发窗口结束但未收到固件 Flip 回执；"
                            "motion_state=%s gait=%s sub_gait=%s",
                            self.driver.config.robot_id,
                            self._one_shot_name,
                            current.motion_state,
                            current.gait,
                            current.sub_gait,
                        )
                    recover_walk = getattr(self.driver, "recover_walk", None)
                    safe_stop = getattr(self.driver, "safe_stop", None)
                    if self._one_shot_name in {
                        Command.BIPEDAL.value,
                        Command.INV_BIPEDAL.value,
                    } and callable(recover_walk):
                        recover_walk()
                    elif callable(safe_stop):
                        safe_stop()
                    else:
                        self.driver.command(Command.WALK, {})
                    mark_fresh = getattr(self.driver, "mark_telemetry_fresh", None)
                    if callable(mark_fresh):
                        mark_fresh()
                    self._one_shot_recovery_complete = True
                    data = self.snapshot()
                    data.updated_at = time.monotonic()
                    data.protection = "正常"
                    self._protection_status = "正常"
                    data.action = "walk"
                    data.feedback = f"{self._one_shot_name} 结束，已切回 Walk"
                    self._feedback_status = data.feedback
                    self._publish(data)
                except Exception as exc:
                    self._set_error(f"特技结束后切回 Walk 失败: {exc}")
                finally:
                    self._one_shot_walk_at = None
                    self._one_shot_expected_sub_gait = None

            if (
                self._one_shot_walk_at is None
                and self._one_shot_unlock_at is None
                and self._heading_hold_active
                and self._one_shot_target_yaw is not None
                and now >= self._next_one_shot_yaw
            ):
                self._apply_heading_return(now)

            if (
                self._tag_position_hold
                and self._tag_origin is not None
                and now >= self._next_tag_hold
            ):
                try:
                    self._apply_tag_position_return(now)
                except Exception as exc:
                    self._set_error(f"标签位置回位失败: {exc}")
                    self._tag_position_hold = False

            if self._one_shot_unlock_at is not None and now >= self._one_shot_unlock_at:
                current = self.snapshot()
                waiting_for_motion = (
                    current.motion_state is not None
                    and current.motion_state != 6
                    and self._one_shot_hard_unlock_at is not None
                    and now < self._one_shot_hard_unlock_at
                )
                if waiting_for_motion:
                    self._feedback_status = (
                        f"等待机器人恢复 Motion 状态（当前 {current.motion_state}）"
                    )
                else:
                    if current.motion_state is not None and current.motion_state != 6:
                        LOG.warning(
                            "%s 特技保护锁达到最长等待时间；motion_state=%s，"
                            "释放软件锁但保留连接/温度保护",
                            self.driver.config.robot_id,
                            current.motion_state,
                        )
                    self._finish_one_shot_lock(now)

            if self._velocity_active and (
                now - self._last_velocity_command
                > self.safety.command_timeout_ms / 1000
            ):
                try:
                    self.driver.command(Command.STOP, {})
                    self._velocity_active = False
                    self._last_commanded_velocity = (0.0, 0.0, 0.0)
                    data = self.snapshot()
                    data.action = "看门狗停止"
                    data.protection = "指令超时，已停止"
                    self._protection_status = data.protection
                    self._publish(data)
                except Exception as exc:
                    self._set_error(f"看门狗停止失败: {exc}")

            if now >= next_poll:
                self._poll()
                # Full SDK telemetry is synchronous. A slightly lower polling
                # rate leaves bandwidth for motion commands on a busy AP.
                next_poll = now + 0.12

    def _apply_heading_return(self, now: float) -> None:
        try:
            self._assert_motion_allowed()
            data = self.snapshot()
            if self._heading_hold_started and now - self._heading_hold_started >= 12.0:
                raise RuntimeError("航向纠正超过 12 秒，未到位；请检查 IMU 和实际转向")
            if data.rpy is None:
                if self._velocity_active:
                    self.driver.command(Command.STOP, {})
                    self._velocity_active = False
                self._heading_hold_aligned = 0
                self._feedback_status = "动作后等待有效 IMU 航向，机器人保持停止"
                data.action = "walk"
                data.feedback = self._feedback_status
                self._publish(data)
                if self._heading_hold_started and now - self._heading_hold_started >= 8.0:
                    self._heading_hold_active = False
                    self._tag_return_pending = False
                    self._feedback_status = "动作后 8 秒仍无有效 IMU，已取消自动回正/回位"
                    data.feedback = self._feedback_status
                    self._publish(data)
                self._next_one_shot_yaw = now + 0.12
                return
            feedback = self._tracker.apply(
                0.0,
                0.0,
                0.0,
                data,
                target_yaw=self._one_shot_target_yaw,
            )
            shaped_yaw, correction_stage = feedback.yaw, "预测制动/微调"
            self._heading_fine_pulse_tick += 1
            params = self._safe_velocity(
                {"x": 0.0, "y": 0.0, "yaw": shaped_yaw}
            )
            self.driver.command(Command.VELOCITY, params)
            self._last_commanded_velocity = (0.0, 0.0, shaped_yaw)
            self._last_velocity_command = now
            self._velocity_active = True
            self._feedback_status = (
                f"动作后角度纠正[{correction_stage}]：{feedback.status}"
            )
            data.action = "walk"
            data.feedback = self._feedback_status
            self._publish(data)
            if (
                abs(feedback.heading_error) <= self.safety.feedback_deadband_rad
                and abs(feedback.yaw) < 1e-9
                and data.rpy is not None
                and all(math.isfinite(v) for v in data.rpy)
                and now - data.updated_at <= self.safety.telemetry_timeout_ms / 1000.0
                and (
                    data.corrected_yaw_rate is not None
                    and abs(data.corrected_yaw_rate) <= 0.04
                )
            ):
                sample = data.odometry_token or data.updated_at
                if sample != self._heading_last_aligned_sample:
                    self._heading_hold_aligned += 1
                    self._heading_last_aligned_sample = sample
                if self._heading_hold_aligned >= 5:
                    self._heading_hold_active = False
                    self._heading_hold_started = 0.0
                    self._velocity_active = False
                    if self._tag_return_pending:
                        self._arm_tag_return()
                    elif self._stunt_tag_return_target is not None:
                        self._arm_stunt_tag_return()
                    else:
                        self.driver.command(Command.STOP, {})
                        self._feedback_status = "航向纠正完成"
                        LOG.info("%s 航向纠正完成，剩余误差=%.2f°", self.driver.config.robot_id, math.degrees(feedback.heading_error))
                    data.feedback = self._feedback_status
                    self._publish(data)
            else:
                self._heading_hold_aligned = 0
        except Exception as exc:
            if self._velocity_active:
                try:
                    self.driver.command(Command.STOP, {})
                except Exception:
                    LOG.exception("%s 航向故障后的停止指令失败", self.driver.config.robot_id)
                finally:
                    self._velocity_active = False
                    self._last_commanded_velocity = (0.0, 0.0, 0.0)
            self._set_error(f"动作后角度纠正失败: {exc}")
            self._heading_hold_active = False
        self._next_one_shot_yaw = now + 0.05

    def _finish_one_shot_lock(self, now: float) -> None:
        self._one_shot_unlock_at = None
        self._one_shot_hard_unlock_at = None
        self._one_shot_recovery_complete = False
        stunt_tag_return_ready = (
            self._stunt_tag_return_target is not None
            and self._tag_position is not None
        )
        tag_return_ready = (
            not stunt_tag_return_ready
            and self._tag_auto_return_enabled
            and self._tag_zero is not None
            and self._tag_position is not None
        )
        self._tag_return_pending = tag_return_ready
        action_name = self._one_shot_name
        self._one_shot_name = None
        self._tracker.reset()
        self._freeze_odometry = False
        if stunt_tag_return_ready or tag_return_ready:
            # 特技安全锁结束后立即用 UWB 拉回标零位置，不等 IMU 先回正；
            # 平移与航向在同一个速度指令里并行纠正。
            self._heading_hold_active = False
            self._heading_hold_aligned = 0
            self._heading_hold_started = 0.0
            self._one_shot_target_yaw = None
            self._next_one_shot_yaw = now
            self._tag_direct_return = True
            self._feedback_status = (
                f"{action_name} 完成，立即开始 UWB 回零"
                f"（最长 {self._tag_return_max_seconds:.1f} s）"
            )
            if stunt_tag_return_ready:
                self._arm_stunt_tag_return()
            else:
                self._arm_tag_return()
            return

        heading_return = self._one_shot_target_yaw is not None
        self._feedback_status = (
            f"{action_name} 完成，先用 IMU 回正航向"
            if heading_return
            else f"{action_name} 安全锁定结束，不执行位置回零"
        )
        self._heading_hold_active = heading_return
        self._heading_hold_aligned = 0
        self._heading_fine_pulse_tick = 0
        self._heading_hold_started = now if heading_return else 0.0
        self._next_one_shot_yaw = now

    def _handle(self, item: WorkItem) -> None:
        if item.command == "connect":
            if (
                item.connect_generation
                and item.connect_generation != self._connect_generation
            ):
                LOG.info(
                    "%s 已取消排队中的连接请求",
                    self.driver.config.robot_id,
                )
                return
            if item.delay_seconds > 0.0:
                time.sleep(item.delay_seconds)
            if (
                item.connect_generation
                and item.connect_generation != self._connect_generation
            ):
                LOG.info(
                    "%s 已取消排队中的连接请求",
                    self.driver.config.robot_id,
                )
                return
            if self.snapshot().connected or getattr(self.driver, "connection_requested", False):
                LOG.info("%s 已连接或正在后台重连，忽略重复连接请求", self.driver.config.robot_id)
                return
            self.driver.connect()
            self._last_odometry_update = None
            self._last_odometry_token = None
            # Publish the successful SDK connection before requesting the
            # optional/extended telemetry set.  A transient first read must
            # not leave the worker stuck forever on its initial offline
            # snapshot.
            self._publish(
                Telemetry(
                    self.driver.config.robot_id,
                    connected=bool(getattr(self.driver, "_connected", True)),
                    action="连接已请求，等待状态",
                    feedback="等待首帧状态数据",
                )
            )
            self._publish(self.driver.telemetry())
            return
        if item.command == "disconnect":
            self.driver.disconnect()
            self._velocity_active = False
            self._one_shot_walk_at = None
            self._one_shot_unlock_at = None
            self._one_shot_hard_unlock_at = None
            self._one_shot_recovery_complete = False
            self._one_shot_name = None
            self._one_shot_is_flip = False
            self._one_shot_expected_sub_gait = None
            self._one_shot_acknowledged = False
            self._one_shot_target_yaw = None
            self._last_odometry_update = None
            self._last_odometry_token = None
            self._clear_stunt_recovery()
            self._publish(self.driver.telemetry())
            return
        if item.command == "odometry_set_pose":
            self._odometry.set_pose(
                item.params["x"], item.params["y"], item.params["yaw_degrees"]
            )
            return
        if item.command == "odometry_add_correction":
            self._odometry.add_correction(
                item.params["dx"],
                item.params["dy"],
                item.params["delta_yaw_degrees"],
            )
            return
        if item.command == "odometry_set_scales":
            self._odometry.set_scales(item.params)
            return
        if item.command == "lateral_feedback_config":
            self._tracker.configure_lateral(
                item.params["kp"],
                item.params["ki"],
                item.params["kd"],
                item.params["max_correction"],
            )
            return
        if item.command == "tag_position_update":
            position = (
                float(item.params["x"]),
                float(item.params["y"]),
                float(item.params["received_at"]),
            )
            if all(math.isfinite(value) for value in position):
                self._tag_position = position
                self._tag_position_generation += 1
            return
        if item.command == "tag_hold_start":
            self._start_tag_position_hold(item.params)
            return
        if item.command == "tag_target_start":
            self._start_tag_position_target(item.params)
            return
        if item.command == "tag_hold_stop":
            self._stop_tag_position_hold()
            return
        if item.command == "tag_zero_set":
            self._set_tag_zero(item.params)
            return

        command = Command(item.command)
        now = time.monotonic()
        if (
            self._one_shot_unlock_at is not None
            and now >= self._one_shot_unlock_at
            and self._one_shot_recovery_complete
        ):
            self._finish_one_shot_lock(now)
        if self._one_shot_unlock_at is not None:
            if (
                command in ONE_SHOT_COMMANDS
                and self._one_shot_hard_unlock_at is not None
                and now >= self._one_shot_hard_unlock_at
            ):
                # Do not let stale telemetry hold the software lock forever.
                self._finish_one_shot_lock(now)
            elif command in ONE_SHOT_COMMANDS:
                remaining = max(
                    0.0,
                    (self._one_shot_hard_unlock_at or self._one_shot_unlock_at)
                    - now,
                )
                raise RuntimeError(
                    f"{self._one_shot_name} 保护锁尚未结束（最多还需 {remaining:.1f}s）"
                )
        if self._one_shot_unlock_at is not None:
            if command is Command.VELOCITY:
                return
            if command not in {Command.STOP, Command.DAMPING}:
                raise RuntimeError(f"{self._one_shot_name} 执行中，普通指令已锁定")
            self._one_shot_walk_at = None
            self._one_shot_unlock_at = None
            self._one_shot_hard_unlock_at = None
            self._one_shot_recovery_complete = False
            self._one_shot_name = None
            self._one_shot_is_flip = False
            self._one_shot_expected_sub_gait = None
            self._one_shot_acknowledged = False
            self._clear_stunt_recovery()

        if command is Command.VELOCITY and self._tag_position_hold:
            # Only an explicitly marked operator command cancels tag hold.
            # Scheduled/background velocity must not silently defeat the
            # requested absolute-position lock.
            if bool(item.params.get("operator_override", False)) and any(
                abs(float(item.params.get(axis, 0.0))) > 1e-6
                for axis in ("x", "y", "yaw")
            ):
                self._stop_tag_position_hold()
            else:
                return

        if command is Command.VELOCITY and (
            self._one_shot_walk_at is not None
            or self._one_shot_unlock_at is not None
            or self._heading_hold_active
        ):
            return

        if command in ONE_SHOT_COMMANDS:
            self._handle_one_shot(command, item.params)
            return

        if command is Command.VELOCITY:
            target_yaw = item.params.get("target_yaw")
            formation_status = str(item.params.get("formation_status", ""))
            path_correction = bool(
                item.params.get(
                    "path_correction",
                    item.params.get("lateral_correction", False),
                )
            )
            sync_enabled = bool(item.params.get("speed_sync", False))
            params = self._safe_velocity(dict(item.params, allow_low_lateral=sync_enabled))
            self._assert_motion_allowed()
            speed_sync_status = ""
            adaptive_status = ""
            manual_turn = abs(params["yaw"]) > 1e-6
            if manual_turn:
                # Operator rotation has absolute priority over every feedback
                # loop.  In particular, do not retain an old yaw target here.
                self._speed_sync.reset()
                self._pause_adaptive_slip()
                self._tracker.reset()
                self._feedback_status = "人工转向（最高优先级）"
                target_yaw = None
                path_correction = False
                current = self.snapshot()
                if bool(item.params.get("speed_sync", False)):
                    rate = current.corrected_yaw_rate
                    if rate is None and current.imu_angular_velocity is not None:
                        rate = current.imu_angular_velocity[2]
                    params["yaw"], _, status = self._turn_speed_sync.apply(
                        params["yaw"], 0.0,
                        None if rate is None else (rate, 0.0), time.monotonic(),
                        limits=(self.safety.max_yaw, self.safety.max_yaw),
                        sample_token=current.odometry_token or current.updated_at,
                    )
                    self._feedback_status += "；旋转" + status
                    params["yaw"] = self._turn_speed_pulse.apply(
                        params["yaw"], self.safety.min_yaw_actuation
                    )
                else:
                    self._turn_speed_sync.reset()
                    self._turn_speed_pulse.reset()
            elif bool(item.params.get("speed_sync", False)):
                self._turn_speed_sync.reset()
                synced_x, synced_y, speed_sync_status = self._speed_sync.apply(
                    params["x"],
                    params["y"],
                    self._calibrated_leg_velocity(
                        self.snapshot().leg_velocity_body
                    ),
                    time.monotonic(),
                    limits=(self.safety.max_x, self.safety.max_y),
                    sample_token=self.snapshot().odometry_token or self.snapshot().updated_at,
                )
                params = self._safe_velocity(
                    {"x": synced_x, "y": synced_y, "yaw": params["yaw"], "allow_low_lateral": True}
                )
            else:
                self._speed_sync.reset()
                self._turn_speed_sync.reset()
            if not manual_turn:
                current_snapshot = self.snapshot()
                adaptive_result = None
                if bool(item.params.get("adaptive_slip", False)):
                    adaptive_result = self._adaptive_slip.apply(
                        params["x"],
                        params["y"],
                        self._calibrated_leg_velocity(
                            current_snapshot.leg_velocity_body
                        ),
                        time.monotonic(),
                        sample_token=current_snapshot.odometry_token,
                        support_legs=current_snapshot.odom_support_legs,
                    )
                    self._adaptive_slip_direction = adaptive_result.direction
                    self._adaptive_slip_cross_velocity = (
                        adaptive_result.filtered_cross_velocity
                    )
                    self._adaptive_slip_bias = adaptive_result.learned_bias
                    self._adaptive_slip_correction = (
                        adaptive_result.correction_x,
                        adaptive_result.correction_y,
                    )
                    adaptive_status = adaptive_result.status
                else:
                    self._pause_adaptive_slip()
                feedback = self._tracker.apply(
                    params["x"],
                    params["y"],
                    params["yaw"],
                    current_snapshot,
                    target_yaw=(
                        float(target_yaw) if target_yaw is not None else None
                    ),
                    path_correction=path_correction,
                )
                adaptive_x = (
                    0.0 if adaptive_result is None else adaptive_result.correction_x
                )
                adaptive_y = (
                    0.0 if adaptive_result is None else adaptive_result.correction_y
                )
                final_y = feedback.y + adaptive_y
                if sync_enabled and abs(feedback.x + adaptive_x) < 1e-9 and abs(feedback.yaw) < 1e-9:
                    final_y = self._lateral_speed_pulse.apply(final_y, self.safety.min_lateral_actuation)
                else:
                    self._lateral_speed_pulse.reset()
                params = self._safe_velocity(
                    {
                        "x": feedback.x + adaptive_x,
                        "y": final_y,
                        "yaw": feedback.yaw,
                    }
                )
                self._feedback_status = (
                    f"{feedback.status}；{speed_sync_status}"
                    if speed_sync_status
                    else feedback.status
                )
                if adaptive_status:
                    self._feedback_status += f"；{adaptive_status}"
            if formation_status:
                self._feedback_status += f"；{formation_status}"
            self._last_velocity_command = time.monotonic()
            self._velocity_active = True
        else:
            params = item.params
            if command in {Command.STOP, Command.DAMPING, Command.SIT}:
                self._velocity_active = False
                self._last_commanded_velocity = (0.0, 0.0, 0.0)
                self._speed_sync.reset()
                self._turn_speed_sync.reset()
                self._lateral_speed_pulse.reset()
                self._turn_speed_pulse.reset()
                self._pause_adaptive_slip()
                self._clear_stunt_recovery()
                self._tracker.reset()
                self._feedback_status = "待命"

        self.driver.command(command, params)
        if command is Command.VELOCITY:
            self._last_commanded_velocity = (
                float(params["x"]), float(params["y"]), float(params["yaw"])
            )
            self._protection_status = "正常"

    def _pause_adaptive_slip(self) -> None:
        self._adaptive_slip.pause()
        self._adaptive_slip_direction = ""
        self._adaptive_slip_cross_velocity = None
        self._adaptive_slip_bias = 0.0
        self._adaptive_slip_correction = (0.0, 0.0)

    def _handle_one_shot(
        self, command: Command, params: dict[str, float]
    ) -> None:
        self._assert_motion_allowed()
        self._one_shot_expected_sub_gait = (
            -1
            if command is Command.LEFT_FLIP
            else -2
            if command is Command.RIGHT_FLIP
            else None
        )
        self._one_shot_acknowledged = False
        self._preempt_tag_position_control_for_stunt(command)
        requested_start_return = bool(params.get("return_to_start", False))
        self._stunt_return_to_start = requested_start_return
        try:
            if self._stunt_return_to_start:
                stale_seconds = max(
                    0.5, min(30.0, float(params.get("stale_seconds", 12.0)))
                )
                self._tag_hold_stale_seconds = stale_seconds
                self._tag_stage_heading_deg = (
                    (float(params.get("stage_heading_deg", 0.0)) + 180.0)
                    % 360.0
                ) - 180.0
                speed_percent = float(
                    params.get("return_speed_percent", 100.0)
                )
                self._tag_return_speed_scale = max(
                    0.70, min(2.00, speed_percent / 100.0)
                )
                self._tag_continuous_return = bool(
                    params.get("continuous_return", True)
                )
                latency_ms = float(params.get("return_latency_ms", 300.0))
                self._tag_return_latency_seconds = max(
                    0.05, min(6.00, latency_ms / 1000.0)
                )
                self._configure_return_timeout(
                    float(params.get("return_max_seconds", 4.0))
                )
                locks = params.get("direction_locks", {})
                if isinstance(locks, dict):
                    lock = locks.get(self.driver.config.robot_id, "xy")
                    if lock in DIRECTION_LOCK_CHOICES:
                        self._tag_direction_lock = lock
            self._capture_stunt_anchor(command)
        except Exception as exc:
            # Recovery is optional. A bad/missing positioning reference must
            # never suppress the stunt command the operator explicitly sent.
            LOG.warning(
                "%s 特技回位准备失败，本次只执行特技: %s",
                self.driver.config.robot_id,
                exc,
            )
            self._stunt_return_to_start = False
            self._stunt_tag_return_target = None
            self._capture_stunt_anchor(command)
        current = self.snapshot()
        preparing = self.snapshot()
        preparing.action = "特技准备"
        preparing.feedback = (
            f"{command.value} 准备中：退出 TAG/速度控制并恢复 Walk"
        )
        self._feedback_status = preparing.feedback
        self._publish(preparing)
        stunt_context = self.driver.command_stunt(command, {})
        self._velocity_active = False
        now = time.monotonic()
        if stunt_context is not None:
            before = stunt_context.get("before", {})
            ready = stunt_context.get("ready", {})
            ready_confirmed = stunt_context.get("ready_confirmed") is True
            log_method = LOG.warning if ready_confirmed else LOG.error
            log_method(
                "%s SDK 已接受特技指令 %s；准备前 motion_state=%s gait=%s；"
                "触发前 motion_state=%s gait=%s；Walk前置状态%s确认",
                self.driver.config.robot_id,
                command.value,
                before.get("motion_state") if isinstance(before, dict) else None,
                before.get("gait") if isinstance(before, dict) else None,
                ready.get("motion_state") if isinstance(ready, dict) else None,
                ready.get("gait") if isinstance(ready, dict) else None,
                "已" if ready_confirmed else "未",
            )
        else:
            LOG.warning(
                "%s SDK 已接受特技指令 %s",
                self.driver.config.robot_id,
                command.value,
            )
        if command in FLIP_COMMANDS or command in ONE_SHOT_DISCRETE_COMMANDS:
            trigger_seconds = effective_flip_trigger_seconds(self.safety)
            self._one_shot_is_flip = command in FLIP_COMMANDS
        else:
            trigger_seconds = float(
                params.get("duration_s", self.safety.non_flip_action_seconds)
            )
            trigger_seconds = max(0.5, min(10.0, trigger_seconds))
            self._one_shot_is_flip = False
        self._one_shot_walk_at = now + trigger_seconds
        self._one_shot_unlock_at = (
            now
            + trigger_seconds
            + one_shot_recovery_seconds(command, self.safety.one_shot_return_ms)
        )
        self._one_shot_hard_unlock_at = (
            self._one_shot_unlock_at
            + ONE_SHOT_MOTION_RECOVERY_GRACE_SECONDS
        )
        self._one_shot_name = command.value
        self._one_shot_recovery_complete = False
        target_yaw = params.get("target_yaw")
        self._one_shot_target_yaw = (
            float(target_yaw)
            if target_yaw is not None
            else (current.rpy[2] if current.rpy is not None else None)
        )
        self._next_one_shot_yaw = now
        ready_note = ""
        if stunt_context is not None:
            ready_note = (
                "；触发前已确认 Motion/Walk"
                if stunt_context.get("ready_confirmed") is True
                else "；警告：触发前未确认 Motion/Walk，请查看日志中的状态值"
            )
        self._feedback_status = (
            f"{command.value} 触发一次，持续 {trigger_seconds:.2f} s 后恢复 Walk"
            f"{ready_note}"
        )
        data = self.snapshot()
        data.action = command.value
        data.feedback = self._feedback_status
        self._publish(data)

    def _preempt_tag_position_control_for_stunt(self, command: Command) -> None:
        """Give an explicit stunt command priority over an old TAG return.

        A TAG controller may have just stopped itself after detecting apparent
        divergence, while the GUI still shows the return option as selected.
        Its target, counters and pending correction must not leak into the
        operator's next one-shot command. Keep the saved TAG zero and the
        auto-return preference: ``_capture_stunt_anchor`` will either create a
        new pre-stunt target or, after the stunt, deliberately fall back to the
        saved zero.
        """
        had_tag_control = bool(
            self._tag_position_hold
            or self._tag_return_pending
            or self._tag_origin is not None
            or self._tag_divergence_ticks
        )
        # _handle_one_shot performs one authoritative BPX STOP/Walk
        # transition after the return anchor is captured. Sending another
        # STOP here doubled the real-robot preparation delay whenever TAG PID
        # happened to be moving.
        self._velocity_active = False
        self._tag_position_hold = False
        self._tag_target_mode = False
        self._tag_direct_return = False
        self._tag_origin = None
        self._tag_return_pending = False
        self._tag_hold_arrived = False
        self._tag_hold_started_at = None
        self._next_tag_hold = 0.0
        self._reset_tag_return_pid()
        if had_tag_control:
            LOG.info(
                "%s 人工触发 %s：已撤销上一轮标签回位状态",
                self.driver.config.robot_id,
                command.value,
            )

    def _reset_tag_return_pid(self) -> None:
        self._tag_last_control_generation = -1
        self._tag_pulse_stop_at = 0.0
        self._tag_latency_wait_until = 0.0
        self._tag_latency_burst_params = None
        self._tag_last_error_distance = None
        self._tag_return_window_deadline = None
        self._tag_last_return_extension_at = None
        self._tag_last_extension_raw_distance = None
        self._tag_divergence_ticks = 0
        self._tag_continuous_anchor = None
        self._tag_waiting_for_generation = None
        self._tag_velocity_integral_x = 0.0
        self._tag_velocity_integral_y = 0.0
        self._tag_velocity_previous_time = None
        self._position_pid_x.reset()
        self._position_pid_y.reset()
        self._position_pid_yaw.reset()

    def _direct_return_allowed(self, raw_dist: float) -> bool:
        """A stunt return may start from far away and still pull back to zero."""
        if self._tag_direct_return and not self._tag_target_mode:
            return raw_dist <= TAG_STUNT_RETURN_MAX_DISTANCE_M
        return raw_dist <= TAG_RETURN_MAX_DISTANCE_M

    def _configure_return_timeout(self, return_max_seconds: float) -> float:
        timeout = max(0.5, min(TAG_RETURN_MAX_WINDOW_SECONDS, float(return_max_seconds)))
        self._tag_return_nominal_seconds = timeout
        self._tag_return_max_seconds = timeout
        self._tag_return_window_deadline = None
        self._tag_last_return_extension_at = None
        self._tag_last_extension_raw_distance = None
        return timeout

    def _estimated_return_window(
        self, raw_dist: float, latency_seconds: float
    ) -> float:
        """Return a deadline long enough for the current UWB error distance."""
        speed = (
            TAG_DIRECT_RETURN_NOMINAL_SPEED_M_S
            if self._tag_direct_return
            else TAG_RETURN_NOMINAL_SPEED_M_S
        )
        speed = max(TAG_MIN_RETURN_SPEED_M_S, speed * self._tag_return_speed_scale)
        travel_seconds = raw_dist / speed
        latency_factor = 1.0 + min(2.0, max(0.0, latency_seconds / 2.0))
        window = travel_seconds * latency_factor + 1.5
        return max(
            self._tag_return_nominal_seconds,
            min(TAG_RETURN_MAX_WINDOW_SECONDS, window),
        )

    def _extend_return_window_if_needed(
        self,
        raw_dist: float,
        raw_dx: float,
        raw_dy: float,
        now: float,
    ) -> bool:
        """Give a still-moving stunt return more time instead of cutting it off."""
        if not self._tag_direct_return or self._tag_target_mode:
            return False
        if raw_dist <= TAG_DIRECT_POSITION_TOLERANCE_M:
            return False
        deadline = self._tag_return_window_deadline
        if deadline is None:
            deadline = self._tag_hold_started_at or now
            deadline += self._tag_return_nominal_seconds
            self._tag_return_window_deadline = deadline
        if now < deadline - 0.15:
            return False
        desired_window = self._estimated_return_window(
            raw_dist, self._tag_return_latency_seconds
        )
        if deadline is not None and now - deadline < desired_window - self._tag_return_nominal_seconds - 0.5:
            return False
        previous = self._tag_last_extension_raw_distance
        progress_ok = previous is None or raw_dist <= previous * 0.985 + 0.05
        if not progress_ok:
            return False
        new_deadline = now + max(
            TAG_RETURN_MIN_EXTENSION_SECONDS,
            desired_window - self._tag_return_nominal_seconds,
        )
        new_deadline = min(
            max(deadline or 0.0, new_deadline),
            (self._tag_hold_started_at or now) + TAG_RETURN_MAX_WINDOW_SECONDS,
        )
        if new_deadline <= deadline:
            return False
        self._tag_return_window_deadline = new_deadline
        self._tag_last_return_extension_at = now
        self._tag_last_extension_raw_distance = raw_dist
        LOG.info(
            "%s 特技回零仍在靠近，延长回位窗口至 %.1f s；"
            "剩余 %.2f m（origin=(%.2f, %.2f)）",
            self.driver.config.robot_id,
            new_deadline - (self._tag_hold_started_at or now),
            raw_dist,
            self._tag_origin[0] if self._tag_origin is not None else float("nan"),
            self._tag_origin[1] if self._tag_origin is not None else float("nan"),
        )
        self._feedback_status = (
            f"回位剩余 {raw_dist:.2f} m，已自动延长窗口至 "
            f"{new_deadline - (self._tag_hold_started_at or now):.1f} s"
        )
        data = self.snapshot()
        data.action = "walk"
        data.feedback = self._feedback_status
        self._publish(data)
        return True

    def _set_tag_zero(self, params: dict[str, float]) -> None:
        origin_x = float(params["origin_x"])
        origin_y = float(params["origin_y"])
        stale_seconds = max(
            0.5, min(30.0, float(params.get("stale_seconds", 12.0)))
        )
        stage_heading_deg = float(params.get("stage_heading_deg", 0.0))
        return_speed_percent = float(params.get("return_speed_percent", 100.0))
        continuous_return = bool(params.get("continuous_return", False))
        return_latency_ms = float(params.get("return_latency_ms", 300.0))
        return_max_seconds = float(params.get("return_max_seconds", 4.0))
        return_max_seconds = max(0.5, min(30.0, return_max_seconds))
        direction_lock = str(params.get("direction_lock", "xy"))
        if direction_lock not in DIRECTION_LOCK_CHOICES:
            direction_lock = "xy"
        if not all(math.isfinite(value) for value in (origin_x, origin_y, stage_heading_deg, return_speed_percent, return_latency_ms, return_max_seconds)):
            raise RuntimeError("标签零位参数无效")
        data = self.snapshot()
        pose = data.corrected_pose or self._odometry.pose()
        if pose is None:
            raise RuntimeError("无有效里程计位姿，不能保存标签零位")
        self._tag_zero = (origin_x, origin_y, float(pose[2]), stale_seconds)
        self._tag_stage_heading_deg = ((stage_heading_deg + 180.0) % 360.0) - 180.0
        self._tag_return_speed_scale = max(0.70, min(2.00, return_speed_percent / 100.0))
        self._tag_continuous_return = continuous_return
        self._tag_return_latency_seconds = max(0.05, min(6.00, return_latency_ms / 1000.0))
        self._tag_return_max_seconds = return_max_seconds
        self._tag_direction_lock = direction_lock
        if self._tag_position_hold:
            self._tag_origin = (origin_x, origin_y, float(pose[2]))
            self._tag_hold_stale_seconds = stale_seconds
            self._tag_hold_arrived = False
            self._tag_hold_started_at = time.monotonic()
            self._next_tag_hold = time.monotonic()
            self._reset_tag_return_pid()
        self._feedback_status = (
            f"标签零位已保存：({origin_x:.3f}, {origin_y:.3f}) m；"
            f"纠偏方向={direction_lock}"
        )
        data.feedback = self._feedback_status
        self._publish(data)
        LOG.info("%s %s", self.driver.config.robot_id, self._feedback_status)

    def _start_tag_position_hold(self, params: dict[str, float]) -> None:
        self._assert_motion_allowed()
        if self._velocity_active:
            self.driver.command(Command.STOP, {})
            self._velocity_active = False
        data = self.snapshot()
        pose = data.corrected_pose or self._odometry.pose()
        if pose is None:
            raise RuntimeError("无有效航向，无法启动标签回位")
        origin_x = float(params["origin_x"])
        origin_y = float(params["origin_y"])
        stale_seconds = max(
            0.5, min(30.0, float(params.get("stale_seconds", 12.0)))
        )
        stage_heading_deg = float(
            params.get(
                "stage_heading_deg",
                0.0 if self._tag_stage_heading_deg is None else self._tag_stage_heading_deg,
            )
        )
        return_speed_percent = float(params.get("return_speed_percent", 100.0))
        continuous_return = bool(params.get("continuous_return", False))
        return_latency_ms = float(params.get("return_latency_ms", 300.0))
        return_max_seconds = float(params.get("return_max_seconds", 4.0))
        return_max_seconds = max(0.5, min(30.0, return_max_seconds))
        direction_lock = str(params.get("direction_lock", self._tag_direction_lock))
        if direction_lock not in DIRECTION_LOCK_CHOICES:
            direction_lock = "xy"
        now = time.monotonic()
        if self._tag_position is None or now - self._tag_position[2] > stale_seconds:
            raise RuntimeError("标签位置过期，无法启动标签回位")
        if not all(math.isfinite(value) for value in (origin_x, origin_y, pose[2], stage_heading_deg, return_speed_percent, return_latency_ms, return_max_seconds)):
            raise RuntimeError("标签零位或航向数值无效")
        origin_yaw = (
            float(self._tag_zero[2])
            if self._tag_zero is not None
            else float(pose[2])
        )
        self._tag_zero = (origin_x, origin_y, origin_yaw, stale_seconds)
        self._tag_stage_heading_deg = ((stage_heading_deg + 180.0) % 360.0) - 180.0
        self._tag_return_speed_scale = max(0.70, min(2.00, return_speed_percent / 100.0))
        self._tag_continuous_return = continuous_return
        self._tag_return_latency_seconds = max(0.05, min(6.00, return_latency_ms / 1000.0))
        self._tag_return_max_seconds = return_max_seconds
        self._tag_direction_lock = direction_lock
        self._tag_origin = (origin_x, origin_y, origin_yaw)
        self._tag_hold_stale_seconds = stale_seconds
        self._tag_position_hold = True
        self._tag_auto_return_enabled = True
        self._tag_target_mode = False
        self._tag_direct_return = False
        self._tag_return_pending = False
        self._tag_hold_arrived = False
        self._tag_hold_started_at = now
        self._reset_tag_return_pid()
        self._next_tag_hold = now
        self._heading_hold_active = False
        self._feedback_status = (
            f"标签零位回位已启用：目标=({origin_x:.3f}, {origin_y:.3f}) m，"
            f"yaw={origin_yaw:.2f}°"
        )
        data.action = "walk"
        data.feedback = self._feedback_status
        self._publish(data)
        LOG.info("%s 启用标签零位回位", self.driver.config.robot_id)

    def _start_tag_position_target(self, params: dict[str, float]) -> None:
        """Move to a temporary map/choreography target without replacing TAG zero."""
        saved_zero = self._tag_zero
        try:
            self._start_tag_position_hold(params)
        finally:
            self._tag_zero = saved_zero
        self._tag_auto_return_enabled = False
        self._tag_target_mode = True
        if self._tag_origin is not None:
            target_x, target_y, _ = self._tag_origin
            self._feedback_status = (
                f"地图目标点控制已启动：({target_x:.3f}, {target_y:.3f}) m"
            )
            data = self.snapshot()
            data.feedback = self._feedback_status
            self._publish(data)
            LOG.info("%s %s", self.driver.config.robot_id, self._feedback_status)

    def _stop_tag_position_hold(self) -> None:
        was_active = self._tag_position_hold or self._tag_return_pending
        if self._velocity_active:
            self.driver.command(Command.STOP, {})
            self._velocity_active = False
        self._tag_position_hold = False
        self._tag_auto_return_enabled = False
        self._tag_target_mode = False
        self._tag_direct_return = False
        self._tag_origin = None
        self._tag_return_pending = False
        self._tag_hold_arrived = False
        self._tag_hold_started_at = None
        self._next_tag_hold = 0.0
        self._reset_tag_return_pid()
        self._feedback_status = (
            "标签零位回位已取消" if was_active else "标签零位回位未启用"
        )
        data = self.snapshot()
        data.action = "walk"
        data.feedback = self._feedback_status
        self._publish(data)

    def _apply_tag_position_return(self, now: float) -> None:
        if self._tag_origin is None or self._tag_position is None:
            self._tag_position_hold = False
            raise RuntimeError("无标签位置")
        data = self.snapshot()
        pose = data.corrected_pose or self._odometry.pose()
        if pose is None:
            self._tag_position_hold = False
            raise RuntimeError("无有效里程计位姿")
        started_at = self._tag_hold_started_at
        if (
            not self._tag_hold_arrived
            and started_at is not None
            and now - started_at >= self._tag_return_max_seconds
        ):
            if self._tag_direct_return and self._tag_origin is not None:
                origin = self._tag_origin
                lock_x, lock_y = direction_lock_mask(self._tag_direction_lock)
                err_x = (
                    origin[0] - self._tag_position[0]
                    if lock_x
                    else 0.0
                )
                err_y = (
                    origin[1] - self._tag_position[1]
                    if lock_y
                    else 0.0
                )
                raw_dist = math.hypot(err_x, err_y)
                if raw_dist <= TAG_DIRECT_POSITION_TOLERANCE_M:
                    self._finish_direct_tag_return()
                    return
                if self._extend_return_window_if_needed(
                    raw_dist,
                    origin[0] - self._tag_position[0],
                    origin[1] - self._tag_position[1],
                    now,
                ):
                    self._tag_return_max_seconds = max(
                        self._tag_return_nominal_seconds,
                        (self._tag_return_window_deadline or now) - started_at,
                    )
                    self._next_tag_hold = now + 0.05
                    return
            if self._velocity_active:
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
            timeout_raw_dist = (
                math.hypot(
                    self._tag_origin[0] - self._tag_position[0],
                    self._tag_origin[1] - self._tag_position[1],
                )
                if self._tag_origin is not None
                else float("nan")
            )
            self._tag_position_hold = False
            self._tag_auto_return_enabled = False
            self._tag_target_mode = False
            self._tag_return_pending = False
            self._tag_hold_started_at = None
            self._feedback_status = (
                "UWB 回位超时"
                f"（已给 {now - started_at:.1f}s，目标容差内仍差 "
                f"{timeout_raw_dist:.2f}m），已停止"
            )
            LOG.error(
                "%s %s，当前位置=%.3f,%.3f；标零位置=(%.3f, %.3f)",
                self.driver.config.robot_id,
                self._feedback_status,
                self._tag_position[0],
                self._tag_position[1],
                self._tag_origin[0],
                self._tag_origin[1],
            )
            data = self.snapshot()
            data.action = "walk"
            data.feedback = self._feedback_status
            self._publish(data)
            return

        origin_x, origin_y, origin_yaw_deg = self._tag_origin
        tag_x, tag_y, received_at = self._tag_position
        high_latency_mode = (
            self._tag_return_latency_seconds
            >= TAG_HIGH_LATENCY_THRESHOLD_SECONDS
        )
        # 特技后直接回零只用 UWB 新鲜坐标，里程计不参与位置推算。
        continuous_odometry_mode = (
            not self._tag_direct_return
            and self._tag_continuous_return
            and (not high_latency_mode or self._tag_target_mode)
        )
        segmented_high_latency_mode = (
            high_latency_mode and not continuous_odometry_mode
        )
        if segmented_high_latency_mode and self._tag_latency_burst_params is not None:
            if now < self._tag_pulse_stop_at:
                # Refresh this bounded command so the normal command watchdog
                # remains active during the deliberately short movement leg.
                self.driver.command(
                    Command.VELOCITY, dict(self._tag_latency_burst_params)
                )
                self._last_velocity_command = now
                self._velocity_active = True
                self._next_tag_hold = now + 0.08
                return
            if self._velocity_active:
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
            self._tag_latency_burst_params = None
            self._tag_pulse_stop_at = 0.0
            self._tag_latency_wait_until = now + self._tag_return_latency_seconds
            self._feedback_status = (
                "秒级 UWB：短程移动已停车，"
                f"等待 {self._tag_return_latency_seconds:.1f}s 后的新定位"
            )
            data.action = "walk"
            data.feedback = self._feedback_status
            self._publish(data)
            self._next_tag_hold = now + 0.12
            return
        if high_latency_mode and self._tag_latency_wait_until > 0.0:
            if self._velocity_active:
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
            if (
                now < self._tag_latency_wait_until
                or received_at < self._tag_latency_wait_until
            ):
                self._feedback_status = (
                    "秒级 UWB：保持停车，等待能反映停车后位置的新数据"
                )
                data.action = "walk"
                data.feedback = self._feedback_status
                self._publish(data)
                self._next_tag_hold = now + 0.12
                return
            self._tag_latency_wait_until = 0.0
        if received_at > now + 1.0 or now - received_at > self._tag_hold_stale_seconds:
            if self._velocity_active:
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
            self._feedback_status = "标签位置过期，已暂停回位（新数据到达后自动恢复）"
            data.action = "walk"
            data.feedback = self._feedback_status
            self._publish(data)
            self._next_tag_hold = now + 0.25
            return

        raw_dx = origin_x - tag_x
        raw_dy = origin_y - tag_y
        lock_x, lock_y = direction_lock_mask(self._tag_direction_lock)
        if not lock_x:
            raw_dx = 0.0
        if not lock_y:
            raw_dy = 0.0
        raw_dist = math.hypot(raw_dx, raw_dy)
        if not all(math.isfinite(value) for value in (raw_dx, raw_dy, raw_dist)):
            raise RuntimeError("标签位置误差无效")
        if self._tag_stage_heading_deg is None:
            self._tag_position_hold = False
            raise RuntimeError("未标定 TAG 舞台坐标中的机头方向")
        # corrected_pose yaw is relative to the robot's yaw-zero frame, while
        # TAG X/Y belongs to the stage frame.  Their zero axes are not
        # generally aligned.  Reconstruct the current heading in the TAG stage
        # frame from the explicitly calibrated heading plus IMU yaw change.
        yaw_delta_deg = ((pose[2] - origin_yaw_deg + 180.0) % 360.0) - 180.0
        stage_yaw_rad = math.radians(self._tag_stage_heading_deg + yaw_delta_deg)
        yaw_err_deg = ((origin_yaw_deg - pose[2] + 180.0) % 360.0) - 180.0

        new_tag_sample = (
            self._tag_last_control_generation != self._tag_position_generation
        )
        # In seconds-latency continuous navigation, ordinary UWB frames that
        # arrive while moving describe historical positions. Comparing those
        # frames as if they were current can falsely report two consecutive
        # increases and stop a correctly moving robot. A sample received after
        # the odometry planner has braked and explicitly requested fresh
        # confirmation is safe to use for the divergence check.
        post_brake_validation_sample = bool(
            new_tag_sample
            and self._tag_waiting_for_generation is not None
        )
        if new_tag_sample and continuous_odometry_mode:
            measured_velocity = data.leg_velocity_body
            if (
                measured_velocity is not None
                and len(measured_velocity) >= 2
                and all(
                    math.isfinite(float(value))
                    for value in measured_velocity[:2]
                )
            ):
                body_vx = float(measured_velocity[0])
                body_vy = float(measured_velocity[1])
                stage_vx = (
                    math.cos(stage_yaw_rad) * body_vx
                    - math.sin(stage_yaw_rad) * body_vy
                )
                stage_vy = (
                    math.sin(stage_yaw_rad) * body_vx
                    + math.cos(stage_yaw_rad) * body_vy
                )
                # UWB carries no sensor timestamp in the current serial
                # protocol. Project its delayed position forward using the
                # measured body velocity and the operator's latency estimate.
                raw_dx -= stage_vx * self._tag_return_latency_seconds
                raw_dy -= stage_vy * self._tag_return_latency_seconds
                if not lock_x:
                    raw_dx = 0.0
                if not lock_y:
                    raw_dy = 0.0
                raw_dist = math.hypot(raw_dx, raw_dy)
        if not high_latency_mode and not self._tag_continuous_return and not new_tag_sample:
            if (
                self._velocity_active
                and self._tag_pulse_stop_at > 0.0
                and now >= self._tag_pulse_stop_at
            ):
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
                self._tag_pulse_stop_at = 0.0
                self._feedback_status = "标签PID脉冲结束，等待下一帧定位"
                data.feedback = self._feedback_status
                self._publish(data)
            self._next_tag_hold = now + 0.05
            return

        if new_tag_sample:
            self._tag_last_control_generation = self._tag_position_generation
            self._tag_waiting_for_generation = None
            self._tag_pulse_stop_at = 0.0
            self._tag_continuous_anchor = (
                raw_dx,
                raw_dy,
                float(pose[0]),
                float(pose[1]),
                received_at,
            )
        elif (
            continuous_odometry_mode
            and self._tag_waiting_for_generation == self._tag_position_generation
        ):
            if self._velocity_active:
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
            self._feedback_status = "已提前刹车，等待新 TAG 确认剩余误差"
            data.feedback = self._feedback_status
            self._publish(data)
            self._next_tag_hold = now + 0.12
            return

        max_distance = (
            TAG_TARGET_MAX_DISTANCE_M
            if self._tag_target_mode
            else (
                TAG_STUNT_RETURN_MAX_DISTANCE_M
                if self._tag_direct_return
                else TAG_RETURN_MAX_DISTANCE_M
            )
        )
        if raw_dist > max_distance:
            if self._velocity_active:
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
            self._feedback_status = (
                f"标签偏差 {raw_dist:.2f}m 超过安全回位范围，已停止；请检查绑定和坐标"
            )
            data.feedback = self._feedback_status
            self._publish(data)
            self._tag_last_error_distance = raw_dist
            self._next_tag_hold = now + 0.25
            return

        # Divergence is evaluated only on independent absolute TAG samples.
        # The high-rate odometry interpolation below must not count as a new
        # external observation.
        divergence_observation = bool(
            new_tag_sample
            and (
                not (
                    high_latency_mode
                    and self._tag_target_mode
                    and continuous_odometry_mode
                )
                or post_brake_validation_sample
            )
        )
        if divergence_observation:
            previous_distance = self._tag_last_error_distance
            if (
                previous_distance is not None
                and raw_dist > TAG_POSITION_TOLERANCE_M
                and raw_dist > previous_distance + TAG_RETURN_DIVERGENCE_M
            ):
                self._tag_divergence_ticks += 1
            elif previous_distance is None or raw_dist < previous_distance - 0.02:
                self._tag_divergence_ticks = 0
            self._tag_last_error_distance = raw_dist
        if self._tag_divergence_ticks >= 2:
            if self._velocity_active:
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
            self._tag_position_hold = False
            self._tag_auto_return_enabled = False
            target_label = "目标点" if self._tag_target_mode else "零位"
            self._feedback_status = (
                f"标签PID连续远离{target_label}，已停机；请核对 TAG X/Y 方向"
            )
            data.feedback = self._feedback_status
            self._publish(data)
            LOG.error("%s %s", self.driver.config.robot_id, self._feedback_status)
            return

        dx, dy = raw_dx, raw_dy
        if continuous_odometry_mode:
            anchor = self._tag_continuous_anchor
            if anchor is None:
                self._next_tag_hold = now + 0.05
                return
            anchor_dx, anchor_dy, anchor_odom_x, anchor_odom_y, anchor_time = anchor
            odometry_horizon = TAG_CONTINUOUS_ODOMETRY_HORIZON_SECONDS
            if high_latency_mode and self._tag_target_mode:
                odometry_horizon = max(
                    odometry_horizon,
                    1.5 * self._tag_return_latency_seconds,
                )
            if now - anchor_time > min(
                self._tag_hold_stale_seconds,
                odometry_horizon,
            ):
                if self._velocity_active:
                    self.driver.command(Command.STOP, {})
                    self._velocity_active = False
                self._feedback_status = "连续回位等待下一帧 TAG，已保持停止"
                data.feedback = self._feedback_status
                self._publish(data)
                self._next_tag_hold = now + 0.12
                return
            if (
                data.odom_uncertainty is not None
                and max(data.odom_uncertainty[:2])
                > TAG_CONTINUOUS_ODOMETRY_UNCERTAINTY_M
            ):
                if self._velocity_active:
                    self.driver.command(Command.STOP, {})
                    self._velocity_active = False
                self._feedback_status = "连续回位里程计置信度不足，等待下一帧 TAG"
                data.feedback = self._feedback_status
                self._publish(data)
                self._next_tag_hold = now + 0.12
                return
            odom_dx = float(pose[0]) - anchor_odom_x
            odom_dy = float(pose[1]) - anchor_odom_y
            zero_stage_yaw = math.radians(self._tag_stage_heading_deg)
            stage_travel_x = (
                math.cos(zero_stage_yaw) * odom_dx
                - math.sin(zero_stage_yaw) * odom_dy
            )
            stage_travel_y = (
                math.sin(zero_stage_yaw) * odom_dx
                + math.cos(zero_stage_yaw) * odom_dy
            )
            dx = anchor_dx - stage_travel_x
            dy = anchor_dy - stage_travel_y
            if not lock_x:
                dx = 0.0
            if not lock_y:
                dy = 0.0
        dist = math.hypot(dx, dy)
        if not math.isfinite(dist) or dist > max_distance:
            if self._velocity_active:
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
            self._feedback_status = "连续回位估算偏差异常，已停止等待新 TAG"
            data.feedback = self._feedback_status
            self._publish(data)
            self._next_tag_hold = now + 0.12
            return

        if self._tag_hold_arrived:
            if (
                abs(yaw_err_deg) <= 3.0
                and (
                    not new_tag_sample
                    or raw_dist <= TAG_POSITION_REACQUIRE_M
                )
            ):
                self._next_tag_hold = now + 0.25
                return
            self._tag_hold_arrived = False

        tolerance = (
            TAG_DIRECT_POSITION_TOLERANCE_M
            if self._tag_direct_return
            else TAG_POSITION_TOLERANCE_M
        )
        position_ok = dist <= tolerance
        yaw_ok = abs(yaw_err_deg) <= 3.0
        if position_ok and yaw_ok:
            if self._velocity_active:
                self.driver.command(Command.STOP, {})
                self._velocity_active = False
            self._tag_hold_arrived = True
            self._position_pid_x.reset()
            self._position_pid_y.reset()
            self._position_pid_yaw.reset()
            self._tag_velocity_integral_x = 0.0
            self._tag_velocity_integral_y = 0.0
            self._tag_velocity_previous_time = None
            self._tag_waiting_for_generation = None
            if high_latency_mode and self._tag_target_mode:
                # Odometry provided the smooth approach. Keep the robot still
                # for a full transport-delay interval, then let a post-stop
                # UWB sample either confirm arrival or authorize one correction.
                self._tag_latency_wait_until = (
                    now + self._tag_return_latency_seconds
                )
            if self._tag_direct_return:
                self._finish_direct_tag_return()
                return
                data.action = "walk"
                data.feedback = self._feedback_status
                self._publish(data)
                return
            self._feedback_status = "标签零位回位完成，保持监测中"
            data.action = "walk"
            data.feedback = self._feedback_status
            self._publish(data)
            self._next_tag_hold = now + 0.25
            return
        body_error_x = math.cos(stage_yaw_rad) * dx + math.sin(stage_yaw_rad) * dy
        body_error_y = -math.sin(stage_yaw_rad) * dx + math.cos(stage_yaw_rad) * dy
        # 直接回零模式在特技结束后立即同时平移和转航向；
        # 普通标签回位仍保留“航向偏差过大先不横移”的安全限制。
        hold_translation = position_ok or (
            not self._tag_direct_return and abs(yaw_err_deg) > 5.0
        )
        world_vx = 0.0 if hold_translation else self._position_pid_x.update(dx, now)
        world_vy = 0.0 if hold_translation else self._position_pid_y.update(dy, now)
        raw_vx = math.cos(stage_yaw_rad) * world_vx + math.sin(stage_yaw_rad) * world_vy
        raw_vy = -math.sin(stage_yaw_rad) * world_vx + math.cos(stage_yaw_rad) * world_vy
        speed_scale = self._tag_return_speed_scale
        direct_speed = 0.36 if self._tag_direct_return else 0.24
        max_vx = min(
            max(direct_speed * speed_scale, TAG_MIN_RETURN_SPEED_M_S),
            self.safety.max_x,
        )
        max_vy = min(
            max(direct_speed * speed_scale, TAG_MIN_RETURN_SPEED_M_S),
            self.safety.max_y,
        )
        if hold_translation:
            vx, vy = 0.0, 0.0
            self._tag_velocity_integral_x = 0.0
            self._tag_velocity_integral_y = 0.0
            self._tag_velocity_previous_time = now
        elif continuous_odometry_mode:
            requested_x = raw_vx * speed_scale
            requested_y = raw_vy * speed_scale
            requested_magnitude = math.hypot(requested_x, requested_y)
            if requested_magnitude > 1e-9:
                direction_x = requested_x / requested_magnitude
                direction_y = requested_y / requested_magnitude
            else:
                error_magnitude = max(1e-9, math.hypot(body_error_x, body_error_y))
                direction_x = body_error_x / error_magnitude
                direction_y = body_error_y / error_magnitude

            directional_limit = float("inf")
            if abs(direction_x) > 1e-9:
                directional_limit = min(
                    directional_limit, max_vx / abs(direction_x)
                )
            if abs(direction_y) > 1e-9:
                directional_limit = min(
                    directional_limit, max_vy / abs(direction_y)
                )
            distance_to_tolerance = max(0.0, dist - TAG_POSITION_TOLERANCE_M)
            brake_limit = braking_speed_limit(
                distance_to_tolerance,
                latency_seconds=(
                    TAG_CONTROL_LATENCY_SECONDS
                    if high_latency_mode and self._tag_target_mode
                    else self._tag_return_latency_seconds
                ),
            )
            planned_limit = min(directional_limit, brake_limit)
            minimum_actuation = max(
                TAG_MIN_RETURN_SPEED_M_S,
                math.hypot(
                    0.12 * direction_x,
                    min(self.safety.min_lateral_actuation, self.safety.max_y)
                    * direction_y,
                ),
            )
            target_magnitude = min(requested_magnitude, planned_limit)
            if target_magnitude < minimum_actuation:
                if new_tag_sample:
                    # One fresh absolute observation may authorize a bounded
                    # dead-zone crossing. The next interpolation tick brakes
                    # and waits for another TAG confirmation.
                    target_magnitude = min(minimum_actuation, directional_limit)
                else:
                    if self._velocity_active:
                        self.driver.command(Command.STOP, {})
                        self._velocity_active = False
                    self._tag_waiting_for_generation = self._tag_position_generation
                    self._tag_velocity_integral_x = 0.0
                    self._tag_velocity_integral_y = 0.0
                    self._tag_velocity_previous_time = now
                    self._feedback_status = (
                        f"预测刹车：剩余 {distance_to_tolerance:.3f}m，"
                        "等待新 TAG 后再微调"
                    )
                    data.feedback = self._feedback_status
                    self._publish(data)
                    self._next_tag_hold = now + 0.12
                    return

            target_vx = direction_x * target_magnitude
            target_vy = direction_y * target_magnitude
            dt = (
                0.12
                if self._tag_velocity_previous_time is None
                else max(0.02, min(0.25, now - self._tag_velocity_previous_time))
            )
            self._tag_velocity_previous_time = now
            measured = data.leg_velocity_body
            if (
                measured is not None
                and len(measured) >= 2
                and all(math.isfinite(float(value)) for value in measured[:2])
            ):
                velocity_error_x = target_vx - float(measured[0])
                velocity_error_y = target_vy - float(measured[1])
                self._tag_velocity_integral_x = max(
                    -TAG_VELOCITY_INTEGRAL_LIMIT,
                    min(
                        TAG_VELOCITY_INTEGRAL_LIMIT,
                        self._tag_velocity_integral_x + velocity_error_x * dt,
                    ),
                )
                self._tag_velocity_integral_y = max(
                    -TAG_VELOCITY_INTEGRAL_LIMIT,
                    min(
                        TAG_VELOCITY_INTEGRAL_LIMIT,
                        self._tag_velocity_integral_y + velocity_error_y * dt,
                    ),
                )
                vx = (
                    target_vx
                    + TAG_VELOCITY_KP * velocity_error_x
                    + TAG_VELOCITY_KI * self._tag_velocity_integral_x
                )
                vy = (
                    target_vy
                    + TAG_VELOCITY_KP * velocity_error_y
                    + TAG_VELOCITY_KI * self._tag_velocity_integral_y
                )
            else:
                self._tag_velocity_integral_x = 0.0
                self._tag_velocity_integral_y = 0.0
                vx, vy = target_vx, target_vy

            # The velocity loop may improve tracking but may never exceed the
            # braking planner's magnitude or the per-axis safety envelope.
            command_limit = max(target_magnitude, 1e-9)
            command_magnitude = math.hypot(vx, vy)
            if command_magnitude > command_limit:
                ratio = command_limit / command_magnitude
                vx *= ratio
                vy *= ratio
            vx = max(-max_vx, min(max_vx, vx))
            vy = max(-max_vy, min(max_vy, vy))
        else:
            vx = effective_return_speed(
                raw_vx * speed_scale,
                body_error_x,
                min(TAG_MIN_RETURN_SPEED_M_S, max_vx),
                max_vx,
            )
            vy = effective_return_speed(
                raw_vy * speed_scale,
                body_error_y,
                min(TAG_MIN_RETURN_SPEED_M_S, max_vy),
                max_vy,
            )
        yaw_error_rad = math.radians(yaw_err_deg)
        raw_yaw = self._position_pid_yaw.update(yaw_error_rad, now)
        yaw_command = (
            effective_return_speed(raw_yaw, yaw_error_rad, 0.25, 0.30)
            if abs(yaw_err_deg) > 3.0
            else 0.0
        )
        params = self._safe_velocity(
            {
                "x": vx,
                "y": vy,
                "yaw": yaw_command,
                "allow_low_lateral": True,
            }
        )
        self.driver.command(Command.VELOCITY, params)
        self._last_velocity_command = now
        self._velocity_active = True
        if segmented_high_latency_mode:
            translation_speed = math.hypot(
                float(params.get("x", 0.0)), float(params.get("y", 0.0))
            )
            distance_to_tolerance = max(
                0.0, dist - TAG_POSITION_TOLERANCE_M
            )
            if translation_speed > 0.01:
                # One delayed absolute observation authorizes only part of the
                # remaining route. Every leg ends in a mandatory stop before
                # another UWB sample can authorize further movement.
                burst_distance = min(
                    TAG_HIGH_LATENCY_MAX_BURST_DISTANCE_M,
                    max(0.03, distance_to_tolerance * 0.45),
                )
                burst_seconds = burst_distance / translation_speed
            else:
                burst_seconds = 0.25
            burst_seconds = max(
                TAG_HIGH_LATENCY_MIN_BURST_SECONDS,
                min(TAG_HIGH_LATENCY_MAX_BURST_SECONDS, burst_seconds),
            )
            self._tag_latency_burst_params = dict(params)
            self._tag_pulse_stop_at = now + burst_seconds
        elif self._tag_continuous_return:
            self._tag_pulse_stop_at = 0.0
        else:
            pulse_scale = 0.85 + 0.15 * min(speed_scale, 1.30)
            self._tag_pulse_stop_at = now + TAG_RETURN_PULSE_SECONDS * pulse_scale
        if segmented_high_latency_mode:
            self._feedback_status = (
                f"秒级UWB分段PID 距离={dist:.3f}m，"
                f"本段={burst_seconds:.2f}s，随后停车等待新定位 "
                f"yaw={yaw_err_deg:+.1f}° 速度={speed_scale * 100:.0f}%"
            )
        elif high_latency_mode and self._tag_target_mode:
            self._feedback_status = (
                f"秒级UWB里程计连续导航 剩余={dist:.3f}m "
                f"（最新TAG={raw_dist:.3f}m） yaw={yaw_err_deg:+.1f}° "
                f"速度={speed_scale * 100:.0f}%"
            )
        else:
            self._feedback_status = (
                f"标签PID{'连续' if self._tag_continuous_return else '脉冲'}回位 "
                f"距离={dist:.3f}m（TAG={raw_dist:.3f}m，容差={TAG_POSITION_TOLERANCE_M:.2f}m）"
                f" yaw={yaw_err_deg:+.1f}° 速度={speed_scale * 100:.0f}%"
            )
        data.action = "walk"
        data.feedback = self._feedback_status
        self._publish(data)
        self._next_tag_hold = now + 0.12

    def _finish_direct_tag_return(self) -> None:
        if self._velocity_active:
            self.driver.command(Command.STOP, {})
            self._velocity_active = False
        self._tag_position_hold = False
        self._tag_direct_return = False
        self._tag_hold_started_at = None
        self._feedback_status = "UWB 回零完成"
        LOG.info(
            "%s UWB 回零完成，当前位置=%.3f,%.3f",
            self.driver.config.robot_id,
            self._tag_position[0],
            self._tag_position[1],
        )
        data = self.snapshot()
        data.action = "walk"
        data.feedback = self._feedback_status
        self._publish(data)

    def _arm_tag_return(self) -> None:
        if self._tag_zero is None or not self._tag_auto_return_enabled:
            self._tag_return_pending = False
            return
        origin_x, origin_y, origin_yaw, stale_seconds = self._tag_zero
        self._tag_origin = (origin_x, origin_y, origin_yaw)
        self._tag_hold_stale_seconds = stale_seconds
        if self._tag_position is not None:
            dx = origin_x - self._tag_position[0]
            dy = origin_y - self._tag_position[1]
            LOG.info(
                "%s 标零位置=(%.3f, %.3f) m；动作后UWB位置=(%.3f, %.3f) m；"
                "待回直线距离=%.3f m",
                self.driver.config.robot_id,
                origin_x,
                origin_y,
                self._tag_position[0],
                self._tag_position[1],
                math.hypot(dx, dy),
            )
        self._tag_position_hold = True
        self._tag_target_mode = False
        self._tag_direct_return = True
        self._tag_return_pending = False
        self._tag_hold_arrived = False
        self._tag_hold_started_at = time.monotonic()
        self._reset_tag_return_pid()
        self._next_tag_hold = time.monotonic()
        self._feedback_status = "特技完成，立即开始 UWB 回零"
        data = self.snapshot()
        data.action = "walk"
        data.feedback = self._feedback_status
        self._publish(data)
        LOG.info("%s 特技结束，立即自动回标零位置", self.driver.config.robot_id)

    def _arm_stunt_tag_return(self) -> None:
        target = self._stunt_tag_return_target
        if target is None:
            return
        origin_x, origin_y, origin_yaw, stale_seconds = target
        self._tag_origin = (origin_x, origin_y, origin_yaw)
        self._tag_hold_stale_seconds = stale_seconds
        if self._tag_position is not None:
            dx = origin_x - self._tag_position[0]
            dy = origin_y - self._tag_position[1]
            LOG.info(
                "%s 标零位置=(%.3f, %.3f) m；动作后UWB位置=(%.3f, %.3f) m；"
                "待回直线距离=%.3f m",
                self.driver.config.robot_id,
                origin_x,
                origin_y,
                self._tag_position[0],
                self._tag_position[1],
                math.hypot(dx, dy),
            )
        self._tag_position_hold = True
        self._tag_target_mode = False
        self._tag_direct_return = True
        self._tag_return_pending = False
        self._tag_hold_arrived = False
        self._tag_hold_started_at = time.monotonic()
        self._stunt_tag_return_target = None
        self._reset_tag_return_pid()
        self._next_tag_hold = time.monotonic()
        self._feedback_status = (
            "特技结束，立即开始回到标零位置"
        )
        data = self.snapshot()
        data.action = "walk"
        data.feedback = self._feedback_status
        self._publish(data)

    def _clear_stunt_recovery(self) -> None:
        self._freeze_odometry = False
        self._tag_position_hold = False
        self._tag_target_mode = False
        self._tag_direct_return = False
        self._tag_origin = None
        self._tag_return_pending = False
        self._tag_hold_arrived = False
        self._tag_hold_started_at = None
        self._next_tag_hold = 0.0
        self._reset_tag_return_pid()
        self._stunt_return_to_start = False
        self._stunt_tag_return_target = None
        self._heading_hold_active = False
        self._heading_hold_started = 0.0

    def _capture_stunt_anchor(self, command: Command) -> None:
        pose = self._odometry.pose()
        self._stunt_tag_return_target = None
        if self._stunt_return_to_start and self._tag_position is not None:
            tag_x, tag_y, received_at = self._tag_position
            now = time.monotonic()
            if (
                received_at <= now + 1.0
                and now - received_at <= self._tag_hold_stale_seconds
            ):
                if self._tag_zero is not None:
                    # 已标零：目标固定为标零位置，不做“动作前位置”快照。
                    self._stunt_tag_return_target = self._tag_zero
                elif pose is not None:
                    # 未标零时保持旧兼容：回到动作开始前的 UWB 位置。
                    self._stunt_tag_return_target = (
                        float(tag_x),
                        float(tag_y),
                        float(pose[2]),
                        self._tag_hold_stale_seconds,
                    )
        self._freeze_odometry = command in FLIP_COMMANDS
        self._tag_position_hold = False
        self._tag_target_mode = False
        self._tag_direct_return = False
        self._tag_origin = None
        self._tag_return_pending = False
        self._tag_hold_arrived = False
        self._tag_hold_started_at = None
        self._reset_tag_return_pid()
        self._heading_hold_active = False
        self._heading_hold_started = 0.0

    def _safe_velocity(self, params: dict[str, float]) -> dict[str, float]:
        x = max(
            -self.safety.max_x,
            min(self.safety.max_x, float(params.get("x", 0))),
        )
        y = max(
            -self.safety.max_y,
            min(self.safety.max_y, float(params.get("y", 0))),
        )
        yaw = max(
            -self.safety.max_yaw,
            min(self.safety.max_yaw, float(params.get("yaw", 0))),
        )
        # BPX Walk treats a command vector below about 0.2 as stationary. Raise
        # only a pure lateral command; combined correction remains fine-grained.
        if (
            0 < abs(y) < self.safety.min_lateral_actuation
            and abs(x) < 1e-9
            and abs(yaw) < 1e-9
            and not bool(params.get("allow_low_lateral", False))
        ):
            y = math.copysign(self.safety.min_lateral_actuation, y)
        return {"x": x, "y": y, "yaw": yaw}

    def _calibrated_leg_velocity(self, measured_velocity):
        """Apply the per-robot six-direction calibration before speed PI."""
        if measured_velocity is None:
            return None
        vx, vy = float(measured_velocity[0]), float(measured_velocity[1])
        scales = self._odometry.scales
        vx *= scales["forward"] if vx >= 0.0 else scales["backward"]
        vy *= scales["left"] if vy >= 0.0 else scales["right"]
        tail = tuple(float(value) for value in measured_velocity[2:])
        return (vx, vy, *tail)

    def _assert_motion_allowed(self) -> None:
        data = self.snapshot()
        if not data.connected:
            raise ConnectionError("设备未连接")
        if time.monotonic() - data.updated_at > self.safety.telemetry_timeout_ms / 1000.0:
            raise RuntimeError("遥测过期，拒绝运动")
        # Battery levels are deliberately warning-only per operator policy.
        if (
            data.max_temperature is not None
            and data.max_temperature >= self.safety.stop_temperature_c
        ):
            raise RuntimeError("温度已达到停止阈值")

    def _poll(self) -> None:
        # Keep connection/startup errors visible until the operator retries.
        current = self.snapshot()
        now = time.monotonic()
        if current.error and not current.connected and not getattr(self.driver, "connection_requested", False):
            return
        try:
            data = self.driver.telemetry()
            self._maybe_reconnect(now, data.connected)
            if current.connected and not data.connected:
                self._cancel_disconnected_actions()
                LOG.warning("%s 连接中断，已取消旧动作；重连后等待新指令", self.driver.config.robot_id)
            elif not current.connected and data.connected:
                LOG.info("%s TCP 已连接，状态读取恢复", self.driver.config.robot_id)
            if (
                self._one_shot_expected_sub_gait is not None
                and not self._one_shot_acknowledged
                and data.gait == 4
                and (
                    data.sub_gait is None
                    or data.sub_gait == self._one_shot_expected_sub_gait
                )
            ):
                self._one_shot_acknowledged = True
                self._feedback_status = (
                    f"{self._one_shot_name} 固件回执已确认："
                    f"gait=4, sub_gait={data.sub_gait}"
                )
                LOG.warning(
                    "%s %s 固件已进入 Flip；motion_state=%s gait=%s sub_gait=%s",
                    self.driver.config.robot_id,
                    self._one_shot_name,
                    data.motion_state,
                    data.gait,
                    data.sub_gait,
                )
            data.commanded_velocity = self._last_commanded_velocity
            data.adaptive_slip_direction = self._adaptive_slip_direction
            data.adaptive_slip_cross_velocity = self._adaptive_slip_cross_velocity
            data.adaptive_slip_bias = self._adaptive_slip_bias
            data.adaptive_slip_correction = self._adaptive_slip_correction
            if data.connected:
                self._update_odometry(data)
                self._apply_protection(data)
                if data.protection == "正常":
                    data.protection = self._protection_status
                data.feedback = self._feedback_status
            self._publish(data)
        except Exception as exc:
            self._set_error(f"状态读取失败: {exc}")

    def _maybe_reconnect(self, now: float, connected: bool) -> None:
        if self._close_requested.is_set():
            return
        if not getattr(self.driver, "connection_requested", False) or connected:
            self._disconnected_since = None
            self._next_reconnect_at = 0.0
            self._reconnect_interval_seconds = RECONNECT_WATCHDOG_INTERVAL_SECONDS
            return
        if self._disconnected_since is None:
            self._disconnected_since = now
            self._next_reconnect_at = (
                now
                + self._reconnect_interval_seconds
                + self._reconnect_jitter_seconds()
            )
            return
        if now < self._next_reconnect_at:
            return
        LOG.warning(
            "%s 持续离线达 %.0f 秒，重置 SDK 连接并重新发起连接",
            self.driver.config.robot_id,
            now - self._disconnected_since,
        )
        try:
            self.driver.disconnect()
            self.driver.connect()
        except Exception as exc:
            LOG.error("%s 自动恢复连接失败：%s", self.driver.config.robot_id, exc)
        self._disconnected_since = now
        self._reconnect_interval_seconds = min(
            RECONNECT_WATCHDOG_MAX_INTERVAL_SECONDS,
            self._reconnect_interval_seconds * 2.0,
        )
        self._next_reconnect_at = (
            now
            + self._reconnect_interval_seconds
            + self._reconnect_jitter_seconds()
        )

    def _reconnect_jitter_seconds(self) -> float:
        session_id = getattr(self.driver.config, "session_id", 1) or 1
        return (session_id % 10) * RECONNECT_WATCHDOG_JITTER_SECONDS_PER_ROBOT

    def _cancel_disconnected_actions(self) -> None:
        self._velocity_active = False
        self._last_commanded_velocity = (0.0, 0.0, 0.0)
        self._one_shot_walk_at = None
        self._one_shot_unlock_at = None
        self._one_shot_hard_unlock_at = None
        self._one_shot_recovery_complete = False
        self._one_shot_name = None
        self._one_shot_is_flip = False
        self._one_shot_expected_sub_gait = None
        self._one_shot_acknowledged = False
        self._one_shot_target_yaw = None
        self._clear_stunt_recovery()
        self._tracker.reset()
        self._speed_sync.reset()
        self._turn_speed_sync.reset()
        self._pause_adaptive_slip()
        self._last_odometry_update = None
        self._last_odometry_token = None
        self._feedback_status = "断联已取消动作，重连后等待新指令"
        for pending_queue in (self._queue, self._priority_queue):
            retained = []
            while True:
                try:
                    pending = pending_queue.get_nowait()
                except queue.Empty:
                    break
                if pending.command in {"shutdown", "disconnect", Command.STOP, Command.DAMPING}:
                    retained.append(pending)
            for pending in retained:
                pending_queue.put(pending)

    def _update_odometry(self, data: Telemetry) -> None:
        now = time.monotonic()
        required = (
            data.leg_velocity_body,
            data.leg_orientation,
            data.imu_acceleration,
            data.imu_angular_velocity,
            data.imu_quaternion,
        )
        fresh_sample = (
            data.odometry_token is None
            or data.odometry_token != self._last_odometry_token
        )
        if all(value is not None for value in required) and fresh_sample:
            if self._freeze_odometry:
                self._last_odometry_update = now
                self._last_odometry_token = data.odometry_token
            elif self._last_odometry_update is not None:
                self._odometry.update(
                    data.leg_velocity_body,
                    data.leg_orientation,
                    data.imu_acceleration,
                    data.imu_angular_velocity,
                    data.imu_quaternion,
                    data.joint_position,
                    data.joint_velocity,
                    now - self._last_odometry_update,
                )
            self._last_odometry_update = now
            self._last_odometry_token = data.odometry_token
        data.corrected_pose = self._odometry.pose()
        data.corrected_velocity_world = self._odometry.velocity_world()
        if data.imu_angular_velocity is not None:
            data.corrected_yaw_rate = self._odometry.corrected_yaw_rate(
                data.imu_angular_velocity[2]
            )
        data.odom_uncertainty = self._odometry.uncertainty()
        data.odom_support_legs = self._odometry.support_legs
        candidates = self._odometry.leg_velocity_candidates
        inliers = self._odometry.leg_velocity_inliers
        data.leg_velocity_candidates_body = candidates or None
        data.leg_velocity_inliers = inliers or None

    def _apply_protection(self, data: Telemetry) -> None:
        reason = None
        stale = (
            time.monotonic() - data.updated_at
            >= self.safety.telemetry_timeout_ms / 1000
        )
        has_imu = data.rpy is not None
        if stale and has_imu:
            reason = "状态数据超时"
        elif stale and not has_imu:
            data.protection = "等待 IMU / 状态上报"
        elif (
            data.max_temperature is not None
            and data.max_temperature >= self.safety.stop_temperature_c
        ):
            reason = "温度过高"
        elif data.rpy is not None and (
            abs(degrees(data.rpy[0])) >= self.safety.max_roll_deg
            or abs(degrees(data.rpy[1])) >= self.safety.max_pitch_deg
        ):
            reason = "机身倾角过大"
        if reason and self._velocity_active:
            if reason == "状态数据超时":
                self._velocity_active = False
                data.protection = reason
                self._protection_status = reason
                LOG.warning("%s 状态超时：尝试停止并站立；断网时无法确认狗端姿态", self.driver.config.robot_id)
                self.driver.hold_stand()
                data.action = "已请求停止并站立（状态待确认）"
            else:
                self.driver.command(Command.DAMPING, {})
                data.action = "自动阻尼"
            self._velocity_active = False
            data.protection = reason
            self._protection_status = reason
        elif (
            data.battery is not None
            and data.battery <= self.safety.critical_battery_percent
        ):
            data.protection = "严重低电量（仅报警，不限制动作）"
        elif (
            data.battery is not None
            and data.battery <= self.safety.low_battery_percent
        ):
            data.protection = "低电量预警"
        elif (
            data.max_temperature is not None
            and data.max_temperature >= self.safety.warn_temperature_c
        ):
            data.protection = "温度预警"

    def _shutdown(self) -> None:
        posture = self._shutdown_posture
        try:
            if self.snapshot().connected or getattr(self.driver, "connection_requested", False):
                posture_label = "请求坐下" if posture is Command.SIT else "请求站立"
                LOG.info(
                    "%s 退出：停止运动并%s后断开",
                    self.driver.config.robot_id,
                    posture_label,
                )
                self.driver.disconnect(posture)
        except Exception as exc:
            LOG.error("%s 安全退出失败: %s", self.driver.config.robot_id, exc)
        self._stop_event.set()


class FleetController:
    def __init__(
        self,
        robots: Iterable[RobotConfig],
        safety: SafetyConfig,
        factory: Callable[[RobotConfig], RobotDriver],
    ) -> None:
        self.robots = {
            config.robot_id: RobotWorker(factory(config), safety)
            for config in robots
        }
        self.connect_stagger_seconds = 0.0

    def submit(
        self, targets: Iterable[str], command: Command, **params: float
    ) -> None:
        for robot_id in targets:
            if robot_id in self.robots:
                self.robots[robot_id].submit(command, **params)

    def connect(self, targets: Iterable[str]) -> None:
        unique_targets = list(dict.fromkeys(targets))
        stagger_seconds = max(0.0, float(self.connect_stagger_seconds))
        for index, robot_id in enumerate(unique_targets):
            worker = self.robots.get(robot_id)
            if worker is not None:
                worker.submit_connect(index * stagger_seconds)

    def disconnect(self, targets: Iterable[str]) -> None:
        for robot_id in dict.fromkeys(targets):
            worker = self.robots.get(robot_id)
            if worker is not None:
                worker.cancel_pending_connect()
                worker.submit("disconnect")

    def set_odometry_pose(
        self, robot_id: str, x: float, y: float, yaw_degrees: float
    ) -> None:
        if robot_id in self.robots:
            self.robots[robot_id].submit(
                "odometry_set_pose",
                x=x,
                y=y,
                yaw_degrees=yaw_degrees,
            )

    def add_odometry_correction(
        self,
        robot_id: str,
        dx: float,
        dy: float,
        delta_yaw_degrees: float,
    ) -> None:
        if robot_id in self.robots:
            self.robots[robot_id].submit(
                "odometry_add_correction",
                dx=dx,
                dy=dy,
                delta_yaw_degrees=delta_yaw_degrees,
            )

    def set_odometry_scales(
        self, robot_id: str, scales: dict[str, float]
    ) -> None:
        if robot_id in self.robots:
            self.robots[robot_id].submit("odometry_set_scales", **scales)

    def set_lateral_feedback_config(
        self,
        robot_id: str,
        kp: float,
        ki: float,
        kd: float,
        max_correction: float,
    ) -> None:
        if robot_id in self.robots:
            self.robots[robot_id].submit(
                "lateral_feedback_config",
                kp=kp,
                ki=ki,
                kd=kd,
                max_correction=max_correction,
            )

    def update_tag_position(
        self, robot_id: str, x: float, y: float, received_at: float
    ) -> None:
        if robot_id in self.robots:
            self.robots[robot_id].submit(
                "tag_position_update", x=x, y=y, received_at=received_at
            )

    def set_tag_zero(
        self,
        robot_id: str,
        origin_x: float,
        origin_y: float,
        stale_seconds: float = 12.0,
        stage_heading_deg: float = 0.0,
        return_speed_percent: float = 100.0,
        continuous_return: bool = False,
        return_latency_ms: float = 300.0,
        return_max_seconds: float = 4.0,
        direction_lock: str = "xy",
    ) -> None:
        if robot_id in self.robots:
            self.robots[robot_id].submit(
                "tag_zero_set",
                origin_x=origin_x,
                origin_y=origin_y,
                stale_seconds=stale_seconds,
                stage_heading_deg=stage_heading_deg,
                return_speed_percent=return_speed_percent,
                continuous_return=continuous_return,
                return_latency_ms=return_latency_ms,
                return_max_seconds=return_max_seconds,
                direction_lock=direction_lock,
            )

    def start_tag_position_hold(
        self,
        robot_id: str,
        origin_x: float,
        origin_y: float,
        stale_seconds: float = 12.0,
        stage_heading_deg: float = 0.0,
        return_speed_percent: float = 100.0,
        continuous_return: bool = False,
        return_latency_ms: float = 300.0,
        return_max_seconds: float = 4.0,
        direction_lock: str = "xy",
    ) -> None:
        if robot_id in self.robots:
            self.robots[robot_id].submit(
                "tag_hold_start",
                origin_x=origin_x,
                origin_y=origin_y,
                stale_seconds=stale_seconds,
                stage_heading_deg=stage_heading_deg,
                return_speed_percent=return_speed_percent,
                continuous_return=continuous_return,
                return_latency_ms=return_latency_ms,
                return_max_seconds=return_max_seconds,
                direction_lock=direction_lock,
            )

    def start_tag_position_target(
        self,
        robot_id: str,
        target_x: float,
        target_y: float,
        stale_seconds: float = 12.0,
        stage_heading_deg: float = 0.0,
        return_speed_percent: float = 100.0,
        continuous_return: bool = False,
        return_latency_ms: float = 300.0,
        return_max_seconds: float = 4.0,
        direction_lock: str = "xy",
    ) -> None:
        if robot_id in self.robots:
            self.robots[robot_id].submit(
                "tag_target_start",
                origin_x=target_x,
                origin_y=target_y,
                stale_seconds=stale_seconds,
                stage_heading_deg=stage_heading_deg,
                return_speed_percent=return_speed_percent,
                continuous_return=continuous_return,
                return_latency_ms=return_latency_ms,
                return_max_seconds=return_max_seconds,
                direction_lock=direction_lock,
            )

    def stop_tag_position_hold(self, robot_id: str) -> None:
        if robot_id in self.robots:
            self.robots[robot_id].submit("tag_hold_stop")

    def emergency_stop(self) -> None:
        for worker in self.robots.values():
            if worker.snapshot().connected:
                worker.submit(Command.DAMPING)

    def snapshots(self) -> list[Telemetry]:
        return [worker.snapshot() for worker in self.robots.values()]

    def close(
        self,
        timeout: float = 3.0,
        final_posture: Command | None = None,
    ) -> None:
        # Ask all workers to stop first, then share one overall wait budget.
        for worker in self.robots.values():
            worker.request_close(final_posture=final_posture)
        deadline = time.monotonic() + max(0.0, timeout)
        for worker in self.robots.values():
            worker.wait_closed(deadline - time.monotonic())
