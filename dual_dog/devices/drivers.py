from __future__ import annotations

from abc import ABC, abstractmethod
import math
import multiprocessing as mp
from multiprocessing.connection import Connection
import threading
import time

from dual_dog.models import Command, RobotConfig, Telemetry


ROBOT_STATE_UPLOAD_RATE_HZ = 20
ROBOT_MOTION_COMMAND_RATE_HZ = 20


JUMP_COMMANDS = frozenset(
    {
        Command.UP_JUMP,
        Command.FRONT_JUMP,
        Command.BACK_JUMP,
        Command.LEFT_JUMP,
        Command.RIGHT_JUMP,
    }
)
BPX_ZERO_VELOCITY_FLUSH_SECONDS = 0.5
BPX_WALK_FLUSH_SECONDS = 1.0
BPX_VELOCITY_FLAG_FLUSH_SECONDS = 0.5
BPX_WALK_READY_TIMEOUT_SECONDS = 1.0
BPX_WALK_READY_POLL_SECONDS = 0.2


class RobotDriver(ABC):
    def __init__(self, config: RobotConfig) -> None:
        self.config = config

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self, final_posture: Command | None = None) -> None: ...

    @abstractmethod
    def command(self, command: Command, params: dict[str, float]) -> None: ...

    def command_stunt(
        self, command: Command, params: dict[str, float]
    ) -> dict[str, object] | None:
        """Stop and submit a one-shot command without requiring SDK internals.

        The BPX driver overrides this so the transition is atomic inside its
        native-SDK subprocess. Other drivers retain the controller's original
        stop/settle/command behaviour.
        """
        self.command(Command.STOP, {})
        if command in {Command.LEFT_FLIP, Command.RIGHT_FLIP}:
            time.sleep(0.25)
        self.command(command, params)
        return None

    def hold_stand(self) -> None:
        self.command(Command.STOP, {})
        self.command(Command.STAND, {})

    def hold_sit(self) -> None:
        self.command(Command.STOP, {})
        self.command(Command.SIT, {})

    @abstractmethod
    def telemetry(self) -> Telemetry: ...


class SimDriver(RobotDriver):
    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)
        self._data = Telemetry(config.robot_id)
        self._lock = threading.Lock()
        self._velocity = (0.0, 0.0, 0.0)
        self._pose = [0.0, 0.0, 0.0]
        self._last_update = time.monotonic()

    def _advance(self) -> None:
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - self._last_update))
        self._last_update = now
        vx, vy, wz = self._velocity
        yaw = self._pose[2]
        self._pose[0] += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
        self._pose[1] += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
        self._pose[2] = (yaw + wz * dt + math.pi) % (2 * math.pi) - math.pi
        half_yaw = self._pose[2] / 2.0
        quaternion = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
        self._data.rpy = (0.0, 0.0, self._pose[2])
        self._data.odom_position = tuple(self._pose)
        self._data.leg_velocity_body = self._velocity
        self._data.leg_orientation = quaternion
        self._data.imu_acceleration = (0.0, 0.0, 9.80665)
        self._data.imu_angular_velocity = (0.0, 0.0, wz)
        self._data.imu_quaternion = quaternion
        self._data.odometry_token = (now,)

    def connect(self) -> None:
        with self._lock:
            self._data.connected = True
            self._data.action = "就绪（模拟）"
            self._data.battery = 100
            self._data.motion_state = 6
            self._data.gait = 0
            self._data.sub_gait = 0
            self._last_update = time.monotonic()
            self._advance()
            self._data.updated_at = time.monotonic()
            self._data.error = None

    def disconnect(self, final_posture: Command | None = None) -> None:
        if final_posture is not None:
            try:
                if self._data.connected:
                    if final_posture is Command.SIT:
                        self.command(Command.SIT, {})
                    elif final_posture is Command.STAND:
                        self.command(Command.STAND, {})
            except ConnectionError:
                pass
        with self._lock:
            self._advance()
            self._velocity = (0.0, 0.0, 0.0)
            self._data.connected = False
            self._data.action = "离线"
            self._data.updated_at = time.monotonic()

    def command(self, command: Command, params: dict[str, float]) -> None:
        with self._lock:
            if not self._data.connected:
                raise ConnectionError(f"{self.config.robot_id} 未连接")
            if command is Command.VELOCITY:
                self._advance()
                self._velocity = (params["x"], params["y"], params["yaw"])
                self._data.action = (
                    f"速度 x={params['x']:.2f}, y={params['y']:.2f}, "
                    f"yaw={params['yaw']:.2f}"
                )
            else:
                if command in {Command.STOP, Command.DAMPING, Command.SIT}:
                    self._advance()
                    self._velocity = (0.0, 0.0, 0.0)
                if command is Command.LEFT_FLIP:
                    self._data.gait = 4
                    self._data.sub_gait = -1
                elif command is Command.RIGHT_FLIP:
                    self._data.gait = 4
                    self._data.sub_gait = -2
                elif command in {Command.WALK, Command.STOP}:
                    self._data.gait = 0
                    self._data.sub_gait = 0
                elif command is Command.SIT:
                    self._data.motion_state = 3
                elif command is Command.STAND:
                    self._data.motion_state = 6
                elif command in JUMP_COMMANDS:
                    self._data.gait = 5
                    self._data.sub_gait = 0
                elif command is Command.UPRIGHT:
                    self._data.motion_state = 6
                self._data.action = command.value
            self._data.updated_at = time.monotonic()

    def telemetry(self) -> Telemetry:
        with self._lock:
            if self._data.connected:
                self._advance()
                self._data.updated_at = time.monotonic()
            return self._data.copy()


def _bpx_process_main(connection: Connection, config: RobotConfig) -> None:
    """Run the native SDK outside the GUI's Python interpreter.

    The vendor extension keeps the GIL while connect() waits on the network.
    A separate process is therefore required; a Python thread cannot keep Qt
    responsive when a robot is offline.
    """
    try:
        import bpx_sdk

        motion = bpx_sdk.MotionLevelControl()
        connection_requested = False
        while True:
            try:
                operation, args = connection.recv()
            except EOFError:
                return
            try:
                if operation in {"command", "prepared_stunt", "safe_stop", "hold_stand", "hold_sit"} and not motion.isConnected():
                    raise ConnectionError("TCP 未连接，指令未发送；等待 SDK 自动重连")
                if operation == "connect":
                    if not connection_requested:
                        motion.setRobotIp(config.ip)
                        motion.setRobotStateUploadPort(config.state_port)
                        motion.setTcpLocalPort(0)
                        motion.setSessionId(config.session_id)
                        motion.setRobotStateUploadRate(ROBOT_STATE_UPLOAD_RATE_HZ)
                        motion.setMotionCommandRate(ROBOT_MOTION_COMMAND_RATE_HZ)
                        if motion.connect() is False:
                            raise ConnectionError("SDK 拒绝启动后台连接")
                        connection_requested = True
                    result = bool(motion.isConnected())
                elif operation == "disconnect":
                    motion.disconnect()
                    connection_requested = False
                    result = None
                elif operation == "hold_stand":
                    motion.setVelocityControlFlag(True)
                    motion.setVelocity(0.0, 0.0, 0.0)
                    time.sleep(0.5)
                    motion.setVelocityControlFlag(False)
                    result = motion.setStandUp()
                    time.sleep(0.5)
                elif operation == "hold_sit":
                    motion.setVelocityControlFlag(True)
                    motion.setVelocity(0.0, 0.0, 0.0)
                    time.sleep(0.5)
                    motion.setVelocityControlFlag(False)
                    result = motion.setSitDown()
                    time.sleep(0.5)
                elif operation == "safe_stop":
                    motion.setVelocityControlFlag(True)
                    motion.setVelocity(0.0, 0.0, 0.0)
                    # A gait/stunt request sent immediately after velocity
                    # control can be acknowledged by the transport but
                    # ignored by robot firmware. Restore the known-good BPX
                    # transition used by the vendor/colleague implementation.
                    time.sleep(0.5)
                    motion.setWalk()
                    motion.setVelocityControlFlag(False)
                    result = None
                elif operation == "recover_walk":
                    motion.setVelocityControlFlag(True)
                    motion.setVelocity(0.0, 0.0, 0.0)
                    time.sleep(BPX_ZERO_VELOCITY_FLUSH_SECONDS)
                    motion.setWalk()
                    time.sleep(BPX_WALK_FLUSH_SECONDS)
                    motion.setVelocityControlFlag(False)
                    time.sleep(BPX_VELOCITY_FLAG_FLUSH_SECONDS)
                    result = True
                elif operation == "prepared_stunt":
                    command = Command(args[0])
                    if command not in {Command.LEFT_FLIP, Command.RIGHT_FLIP}:
                        raise ValueError(f"不支持的原子特技指令: {command}")

                    def motion_context() -> dict[str, object]:
                        context: dict[str, object] = {
                            "motion_state": None,
                            "gait": None,
                            "sub_gait": None,
                        }
                        for key, getter_name in (
                            ("motion_state", "getCurrentMotionState"),
                            ("gait", "getCurrentGait"),
                            ("sub_gait", "getSubGait"),
                        ):
                            getter = getattr(motion, getter_name, None)
                            if getter is not None:
                                try:
                                    context[key] = getter()
                                except Exception:
                                    pass
                        return context

                    before = motion_context()
                    # Keep the complete real-robot transition in one child
                    # request. No telemetry or queued TAG velocity packet can
                    # be inserted between restoring Walk and triggering Flip.
                    # These flush windows mirror the SDK's real-robot gait test:
                    # Walk must be transmitted for about one second, then the
                    # disabled velocity-control flag must itself be transmitted
                    # before the firmware will reliably accept a flip.
                    motion.setVelocityControlFlag(True)
                    zero_result = motion.setVelocity(0.0, 0.0, 0.0)
                    if zero_result is False:
                        raise RuntimeError("固件拒绝侧翻前零速度指令")
                    time.sleep(BPX_ZERO_VELOCITY_FLUSH_SECONDS)
                    motion.setWalk()
                    time.sleep(BPX_WALK_FLUSH_SECONDS)
                    motion.setVelocityControlFlag(False)
                    time.sleep(BPX_VELOCITY_FLAG_FLUSH_SECONDS)
                    ready = motion_context()
                    ready_deadline = (
                        time.monotonic() + BPX_WALK_READY_TIMEOUT_SECONDS
                    )

                    def walk_ready(context: dict[str, object]) -> bool:
                        state = context.get("motion_state")
                        gait = context.get("gait")
                        has_feedback = state is not None or gait is not None
                        return has_feedback and (
                            state is None or state == 6
                        ) and (
                            gait is None or gait == 0
                        )

                    # State uploads can lag behind the periodic command sender.
                    # If feedback is available and still says non-Walk, extend
                    # preparation briefly and reassert Walk. Do not permanently
                    # suppress an operator's explicit stunt when feedback is
                    # missing or stale; report the unconfirmed state instead.
                    while (
                        (ready.get("motion_state") is not None
                         or ready.get("gait") is not None)
                        and not walk_ready(ready)
                        and time.monotonic() < ready_deadline
                    ):
                        motion.setWalk()
                        time.sleep(BPX_WALK_READY_POLL_SECONDS)
                        ready = motion_context()
                    if command is Command.LEFT_FLIP:
                        stunt_result = motion.setLeftFlip()
                    else:
                        stunt_result = motion.setRightFlip()
                    result = {
                        "accepted": stunt_result is not False,
                        "before": before,
                        "ready": ready,
                        "ready_confirmed": walk_ready(ready),
                    }
                elif operation == "command":
                    command = Command(args[0])
                    params = args[1]
                    if command is Command.VELOCITY:
                        motion.setVelocityControlFlag(True)
                        result = motion.setVelocity(params["x"], params["y"], params["yaw"])
                    elif command is Command.STOP:
                        motion.setVelocityControlFlag(True)
                        motion.setVelocity(0.0, 0.0, 0.0)
                        time.sleep(0.5)
                        motion.setWalk()
                        motion.setVelocityControlFlag(False)
                        result = True
                    elif command is Command.STAND:
                        result = motion.setStandUp()
                    elif command is Command.SIT:
                        result = motion.setSitDown()
                    elif command is Command.WALK:
                        result = motion.setWalk()
                    elif command is Command.DAMPING:
                        motion.setVelocityControlFlag(True)
                        motion.setVelocity(0.0, 0.0, 0.0)
                        motion.setVelocityControlFlag(False)
                        result = motion.setDamping()
                    elif command is Command.LEFT_FLIP:
                        result = motion.setLeftFlip()
                    elif command is Command.RIGHT_FLIP:
                        result = motion.setRightFlip()
                    elif command is Command.PRONK:
                        result = motion.setPronk()
                    elif command is Command.PACE:
                        result = motion.setPace()
                    elif command is Command.BOUND:
                        result = motion.setBound()
                    elif command is Command.BIPEDAL:
                        result = motion.setBipedal()
                    elif command is Command.INV_BIPEDAL:
                        result = motion.setInvBipedal()
                    elif command is Command.RUN:
                        result = motion.setRunning()
                    elif command is Command.UP_JUMP:
                        result = motion.setUpJump()
                    elif command is Command.FRONT_JUMP:
                        result = motion.setFrontJump()
                    elif command is Command.BACK_JUMP:
                        result = motion.setBackJump()
                    elif command is Command.LEFT_JUMP:
                        result = motion.setLeftJump()
                    elif command is Command.RIGHT_JUMP:
                        result = motion.setRightJump()
                    elif command is Command.UPRIGHT:
                        result = motion.setUpright()
                    else:
                        raise ValueError(f"不支持的指令: {command}")
                    # Several SDK action methods use None to mean success.
                    result = True if result is None else bool(result)
                elif operation == "telemetry":
                    if not motion.isConnected():
                        connection.send((True, {"connected": False}))
                        continue
                    odom = motion.getLegOdom()
                    imu_acceleration = motion.getImuAcc()
                    imu_angular_velocity = motion.getImuOmega()
                    imu_quaternion = motion.getImuQuat()
                    joint_position = motion.getJointPosition()
                    joint_velocity = motion.getJointVelocity()
                    joint_timestamp = motion.getJointStateTimestamp()
                    imu_timestamp = motion.getImuTimestamp()
                    odometry_timestamp = motion.getOdometryTimestamp()
                    motor = motion.getMotorTemperature()
                    driver = motion.getDriverTemperature()
                    result = {
                        "connected": bool(motion.isConnected()),
                        "battery": motion.getBatteryLevel(),
                        "rpy": motion.getImuRpy(),
                        "odom_position": None if odom is None else tuple(odom["position"]),
                        "leg_velocity_body": None if odom is None else tuple(odom["velocity_body"]),
                        "leg_orientation": None if odom is None else tuple(odom["orientation"]),
                        "imu_acceleration": None if imu_acceleration is None else tuple(imu_acceleration),
                        "imu_angular_velocity": None if imu_angular_velocity is None else tuple(imu_angular_velocity),
                        "imu_quaternion": None if imu_quaternion is None else tuple(imu_quaternion),
                        "joint_position": None if joint_position is None else tuple(joint_position),
                        "joint_velocity": None if joint_velocity is None else tuple(joint_velocity),
                        "odometry_token": (
                            joint_timestamp,
                            imu_timestamp,
                            odometry_timestamp,
                        ),
                        "motion_state": motion.getCurrentMotionState(),
                        "gait": motion.getCurrentGait(),
                        "sub_gait": motion.getSubGait(),
                        "temperatures": list(motor or []) + list(driver or []),
                        "timestamps": (
                            motion.getMotionStateTimestamp(),
                            imu_timestamp,
                            motion.getBatteryTimestamp(),
                        ),
                    }
                else:
                    raise ValueError(f"未知 SDK 操作: {operation}")
                connection.send((True, result))
            except Exception as exc:
                connection.send((False, f"{type(exc).__name__}: {exc}"))
    except Exception as exc:
        try:
            connection.send((False, f"SDK 进程启动失败: {type(exc).__name__}: {exc}"))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        connection.close()


class BpxDriver(RobotDriver):
    CONNECT_TIMEOUT_SECONDS = 8.0
    CALL_TIMEOUT_SECONDS = 1.0
    STOP_TIMEOUT_SECONDS = 2.0
    STUNT_TIMEOUT_SECONDS = 5.5

    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)
        self._process: mp.Process | None = None
        self._connection: Connection | None = None
        self._connected = False
        self.connection_requested = False
        self._action = "离线"
        self._last_state_token: tuple[object, ...] | None = None
        self._last_telemetry_at = time.monotonic()

    def _start_process(self) -> None:
        if self._process is not None and self._process.is_alive():
            return
        context = mp.get_context("spawn")
        parent_connection, child_connection = context.Pipe()
        process = context.Process(
            target=_bpx_process_main,
            args=(child_connection, self.config),
            name=f"bpx-sdk-{self.config.robot_id}",
            daemon=True,
        )
        process.start()
        child_connection.close()
        self._process = process
        self._connection = parent_connection

    def _terminate_process(self) -> None:
        if self._connection is not None:
            self._connection.close()
            self._connection = None
        if self._process is not None:
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(timeout=1.0)
            self._process.close()
            self._process = None

    def _request(self, operation: str, *args: object, timeout: float | None = None):
        self._start_process()
        assert self._connection is not None
        wait_seconds = self.CALL_TIMEOUT_SECONDS if timeout is None else timeout
        try:
            self._connection.send((operation, args))
            if not self._connection.poll(wait_seconds):
                raise TimeoutError(f"{operation} 超过 {wait_seconds:.1f} 秒无响应")
            ok, result = self._connection.recv()
        except (BrokenPipeError, EOFError, OSError, TimeoutError):
            self._connected = False
            self.connection_requested = False
            self._action = "离线"
            self._terminate_process()
            raise
        if not ok:
            # The child catches individual SDK call errors and remains able to
            # serve later requests.  Preserve an established session for a
            # telemetry/command error; only a failed connect owns no session.
            if operation == "connect":
                self._terminate_process()
            raise RuntimeError(str(result))
        return result

    def connect(self) -> None:
        if self.connection_requested:
            return
        self._connected = bool(self._request("connect", timeout=self.CONNECT_TIMEOUT_SECONDS))
        self.connection_requested = True
        self._action = "就绪" if self._connected else "连接中（SDK 自动重试）"
        self._last_state_token = None
        self._last_telemetry_at = time.monotonic()

    def disconnect(self, final_posture: Command | None = None) -> None:
        try:
            if self.connection_requested:
                try:
                    if self._connected:
                        if final_posture is Command.SIT:
                            self.hold_sit()
                        else:
                            self.hold_stand()
                finally:
                    if self.connection_requested:
                        self._request("disconnect")
        finally:
            self._connected = False
            self.connection_requested = False
            self._action = "离线"
            self._terminate_process()

    def hold_stand(self) -> None:
        self._request("hold_stand", timeout=self.STOP_TIMEOUT_SECONDS)

    def hold_sit(self) -> None:
        self._request("hold_sit", timeout=self.STOP_TIMEOUT_SECONDS)

    def safe_stop(self) -> None:
        self._request("safe_stop", timeout=self.STOP_TIMEOUT_SECONDS)

    def recover_walk(self) -> None:
        self._request("recover_walk", timeout=4.0)

    def mark_telemetry_fresh(self) -> None:
        """Avoid a false stale-telemetry protection after a blocking recovery."""
        self._last_telemetry_at = time.monotonic()

    def command(self, command: Command, params: dict[str, float]) -> None:
        if not self._connected:
            raise ConnectionError(f"{self.config.robot_id} 未连接")
        timeout = (
            self.STOP_TIMEOUT_SECONDS
            if command is Command.STOP
            else self.CALL_TIMEOUT_SECONDS
        )
        ok = self._request("command", command.value, params, timeout=timeout)
        if ok is False:
            raise RuntimeError(f"{self.config.robot_id} 拒绝 {command.value} 指令")
        self._action = command.value

    def command_stunt(
        self, command: Command, params: dict[str, float]
    ) -> dict[str, object] | None:
        if not self._connected:
            raise ConnectionError(f"{self.config.robot_id} 未连接")
        if command not in {Command.LEFT_FLIP, Command.RIGHT_FLIP}:
            return super().command_stunt(command, params)
        context = self._request(
            "prepared_stunt",
            command.value,
            timeout=self.STUNT_TIMEOUT_SECONDS,
        )
        if not isinstance(context, dict) or context.get("accepted") is not True:
            raise RuntimeError(f"{self.config.robot_id} 拒绝 {command.value} 指令")
        self._action = command.value
        return context

    def telemetry(self) -> Telemetry:
        data = Telemetry(self.config.robot_id, connected=self._connected, action=self._action)
        if not self.connection_requested:
            return data
        try:
            raw = self._request("telemetry")
        except Exception as exc:
            # Extended odometry/IMU getters are useful for correction but a
            # single failed getter must not erase a successful SDK connection.
            # Transport failures update _connected inside _request, so the UI
            # still reports an actual lost subprocess as offline.
            data.connected = self._connected
            data.action = self._action if self._connected else "离线"
            data.updated_at = self._last_telemetry_at
            data.error = f"状态读取暂不可用: {exc}"
            return data
        was_connected = self._connected
        self._connected = bool(raw["connected"])
        data.connected = self._connected
        if not self._connected:
            self._action = "连接中（SDK 自动重试）"
            self._last_state_token = None
            self._last_telemetry_at = 0.0
            data.action = self._action
            data.error = "TCP 未连接，SDK 正在后台重试"
            data.updated_at = 0.0
            return data
        if not was_connected:
            self._action = "已连接，等待新指令"
            data.action = self._action
            self._last_state_token = None
            self._last_telemetry_at = 0.0
        battery = raw["battery"]
        rpy = raw["rpy"]
        data.battery = int(battery) if battery is not None else None
        data.rpy = tuple(float(v) for v in rpy) if rpy is not None else None
        if raw["odom_position"] is not None:
            data.odom_position = tuple(float(v) for v in raw["odom_position"])
        for field_name in (
            "leg_velocity_body",
            "leg_orientation",
            "imu_acceleration",
            "imu_angular_velocity",
            "imu_quaternion",
            "joint_position",
            "joint_velocity",
        ):
            value = raw[field_name]
            if value is not None:
                setattr(data, field_name, tuple(float(item) for item in value))
        odometry_token = tuple(raw["odometry_token"])
        if any(value is not None for value in odometry_token):
            data.odometry_token = odometry_token
        data.motion_state = raw["motion_state"]
        data.gait = raw["gait"]
        raw_sub_gait = raw["sub_gait"]
        if raw_sub_gait is not None:
            raw_sub_gait = int(raw_sub_gait)
            data.sub_gait = (
                raw_sub_gait - 256 if raw_sub_gait > 127 else raw_sub_gait
            )
        temperatures = raw["temperatures"]
        data.max_temperature = max(temperatures) if temperatures else None
        timestamps = tuple(raw["timestamps"])
        if any(value is not None for value in timestamps) and timestamps != self._last_state_token:
            self._last_state_token = timestamps
            self._last_telemetry_at = time.monotonic()
        data.updated_at = self._last_telemetry_at
        return data


def degrees(radians: float) -> float:
    return math.degrees(radians)
