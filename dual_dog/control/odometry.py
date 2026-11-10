from __future__ import annotations

import math
import threading

import numpy as np


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_rot(quaternion) -> np.ndarray:
    """Convert an SDK quaternion in x/y/z/w order to a rotation matrix."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def yaw_from_quat(quaternion) -> float:
    x, y, z, w = (float(value) for value in quaternion)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class FourLegKinematics:
    """BPX URDF kinematics used for robust stance-foot velocity constraints."""

    LEGS = ((1, 1), (1, -1), (-1, 1), (-1, -1))  # FL, FR, HL, HR

    def __init__(self) -> None:
        self.last_candidates: tuple[tuple[float, float], ...] = ()
        self.last_inliers: tuple[bool, ...] = ()

    @staticmethod
    def _rx(angle: float) -> np.ndarray:
        cosine, sine = math.cos(angle), math.sin(angle)
        return np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]])

    @staticmethod
    def _ry(angle: float) -> np.ndarray:
        cosine, sine = math.cos(angle), math.sin(angle)
        return np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]])

    def foot_position(self, leg_index: int, joint_angles) -> np.ndarray:
        longitudinal_sign, lateral_sign = self.LEGS[leg_index]
        roll, hip, knee = (float(value) for value in joint_angles)
        position = np.array(
            [longitudinal_sign * 0.2386, lateral_sign * 0.056, 0.0]
        )
        roll_rotation = self._rx(roll)
        position += roll_rotation @ np.array([0.0, lateral_sign * 0.11522, 0.0])
        hip_rotation = roll_rotation @ self._ry(hip)
        position += hip_rotation @ np.array([0.0, 0.0, -0.23])
        knee_rotation = hip_rotation @ self._ry(knee)
        position += knee_rotation @ np.array([0.0, 0.0, -0.2315648])
        return position

    def jacobian(self, leg_index: int, joint_angles) -> np.ndarray:
        epsilon = 1e-5
        angles = np.asarray(joint_angles, dtype=float)
        jacobian = np.zeros((3, 3))
        for index in range(3):
            plus, minus = angles.copy(), angles.copy()
            plus[index] += epsilon
            minus[index] -= epsilon
            jacobian[:, index] = (
                self.foot_position(leg_index, plus)
                - self.foot_position(leg_index, minus)
            ) / (2 * epsilon)
        return jacobian

    def body_velocity_measurement(
        self,
        joint_position,
        joint_velocity,
        angular_velocity,
        reference_velocity=None,
    ) -> tuple[np.ndarray | None, int]:
        positions = np.asarray(joint_position, dtype=float).reshape(4, 3)
        velocities = np.asarray(joint_velocity, dtype=float).reshape(4, 3)
        omega = np.asarray(angular_velocity, dtype=float)
        candidates = []
        for index in range(4):
            foot = self.foot_position(index, positions[index])
            relative_velocity = self.jacobian(index, positions[index]) @ velocities[index]
            candidates.append((-relative_velocity - np.cross(omega, foot))[:2])
        candidate_array = np.asarray(candidates)
        self.last_candidates = tuple(
            (float(candidate[0]), float(candidate[1]))
            for candidate in candidate_array
        )

        # In trot, two stance legs should infer nearly the same body velocity,
        # while swing-leg estimates are usually far away.  Find the most
        # consistent pair and optionally prefer a pair near the SDK leg-odom
        # velocity.  This is more robust than averaging all four legs first.
        reference = (
            None
            if reference_velocity is None
            else np.asarray(reference_velocity, dtype=float)[:2]
        )
        best_pair = (0, 1)
        best_score = math.inf
        best_distance = math.inf
        for first in range(4):
            for second in range(first + 1, 4):
                pair_distance = float(
                    np.linalg.norm(candidate_array[first] - candidate_array[second])
                )
                center = (candidate_array[first] + candidate_array[second]) * 0.5
                reference_penalty = (
                    0.0
                    if reference is None
                    else 0.35 * float(np.linalg.norm(center - reference))
                )
                score = pair_distance + reference_penalty
                if score < best_score:
                    best_score = score
                    best_distance = pair_distance
                    best_pair = (first, second)
        if best_distance > 0.22:
            self.last_inliers = (False, False, False, False)
            return None, 0

        center = np.mean(candidate_array[list(best_pair)], axis=0)
        residual = np.linalg.norm(candidate_array - center, axis=1)
        threshold = max(0.06, min(0.16, 0.04 + best_distance * 1.5))
        keep = residual <= threshold
        keep[list(best_pair)] = True
        count = int(np.count_nonzero(keep))
        self.last_inliers = tuple(bool(value) for value in keep)
        if count < 2:
            return None, 0
        return np.mean(candidate_array[keep], axis=0), count


class CorrectedOdometry:
    """Eight-state IMU, leg-odometry and stance-foot EKF for one robot."""

    DEFAULT_SCALES = {
        "forward": 1.0,
        "backward": 1.0,
        "left": 1.0,
        "right": 1.0,
        "turn_left": 1.0,
        "turn_right": 1.0,
    }

    def __init__(self) -> None:
        self._lock = threading.RLock()
        # x, y, yaw, vx_world, vy_world, gyro_bias_z, accel_bias_x, accel_bias_y
        self._state = np.zeros(8)
        self._covariance = np.diag(
            [0.05, 0.05, 0.03, 0.2, 0.2, 0.01, 0.1, 0.1]
        ) ** 2
        self._scales = dict(self.DEFAULT_SCALES)
        self._kinematics = FourLegKinematics()
        self._support_legs = 0
        self._yaw_frame_offset: float | None = None
        self._last_raw_yaw: float | None = None

    @property
    def scales(self) -> dict[str, float]:
        with self._lock:
            return dict(self._scales)

    def set_scales(self, values: dict[str, float]) -> None:
        checked = dict(self.DEFAULT_SCALES)
        checked.update({key: float(value) for key, value in values.items() if key in checked})
        if any(not math.isfinite(value) or not 0.5 <= value <= 1.5 for value in checked.values()):
            raise ValueError("里程修正系数必须处于 0.5～1.5")
        with self._lock:
            self._scales = checked

    def _ekf_update(self, measurement, prediction, matrix, noise, angle_index=None) -> None:
        innovation = np.asarray(measurement) - np.asarray(prediction)
        if angle_index is not None:
            innovation[angle_index] = wrap_angle(innovation[angle_index])
        innovation_covariance = matrix @ self._covariance @ matrix.T + noise
        gain = self._covariance @ matrix.T @ np.linalg.inv(innovation_covariance)
        self._state += gain @ innovation
        self._state[2] = wrap_angle(self._state[2])
        identity = np.eye(8)
        remainder = identity - gain @ matrix
        self._covariance = (
            remainder @ self._covariance @ remainder.T + gain @ noise @ gain.T
        )

    def set_pose(self, x: float, y: float, yaw_degrees: float) -> None:
        with self._lock:
            self._state[0] = float(x)
            self._state[1] = float(y)
            self._state[2] = wrap_angle(math.radians(float(yaw_degrees)))
            self._covariance[:3, :3] = np.diag(
                [0.02, 0.02, math.radians(1.0)]
            ) ** 2
            if self._last_raw_yaw is not None:
                self._yaw_frame_offset = wrap_angle(self._state[2] - self._last_raw_yaw)

    def add_correction(self, dx: float, dy: float, delta_yaw_degrees: float) -> None:
        with self._lock:
            delta_yaw = math.radians(float(delta_yaw_degrees))
            self._state[0] += float(dx)
            self._state[1] += float(dy)
            self._state[2] = wrap_angle(self._state[2] + delta_yaw)
            if self._yaw_frame_offset is not None:
                self._yaw_frame_offset = wrap_angle(self._yaw_frame_offset + delta_yaw)

    def update(
        self,
        leg_velocity_body,
        leg_orientation,
        imu_acceleration,
        imu_angular_velocity,
        imu_quaternion,
        joint_position,
        joint_velocity,
        dt: float,
    ) -> None:
        if dt <= 0.0 or dt > 0.2:
            return
        vx, vy = (float(value) for value in leg_velocity_body[:2])
        wz = float(imu_angular_velocity[2])

        with self._lock:
            scales = self._scales
            vx *= scales["forward"] if vx >= 0.0 else scales["backward"]
            vy *= scales["left"] if vy >= 0.0 else scales["right"]
            wz *= scales["turn_left"] if wz >= 0.0 else scales["turn_right"]

            raw_yaw = yaw_from_quat(leg_orientation)
            self._last_raw_yaw = raw_yaw
            if self._yaw_frame_offset is None:
                self._yaw_frame_offset = wrap_angle(self._state[2] - raw_yaw)
            frame_cosine = math.cos(self._yaw_frame_offset)
            frame_sine = math.sin(self._yaw_frame_offset)
            frame_rotation = np.array(
                [
                    [frame_cosine, -frame_sine, 0.0],
                    [frame_sine, frame_cosine, 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )
            rotation = frame_rotation @ quat_to_rot(imu_quaternion)
            acceleration_world = rotation @ np.asarray(
                imu_acceleration, dtype=float
            ) - np.array([0.0, 0.0, 9.80665])
            ax = acceleration_world[0] - self._state[6]
            ay = acceleration_world[1] - self._state[7]
            gyro = wz - self._state[5]
            self._state[0] += self._state[3] * dt + 0.5 * ax * dt * dt
            self._state[1] += self._state[4] * dt + 0.5 * ay * dt * dt
            self._state[2] = wrap_angle(self._state[2] + gyro * dt)
            self._state[3] += ax * dt
            self._state[4] += ay * dt

            transition = np.eye(8)
            transition[0, 3], transition[1, 4], transition[2, 5] = dt, dt, -dt
            transition[0, 6] = transition[1, 7] = -0.5 * dt * dt
            transition[3, 6] = transition[4, 7] = -dt
            process_noise = np.diag(
                [0.002, 0.002, 0.004, 0.12, 0.12, 0.001, 0.01, 0.01]
            ) ** 2 * dt
            self._covariance = (
                transition @ self._covariance @ transition.T + process_noise
            )

            cosine, sine = math.cos(self._state[2]), math.sin(self._state[2])
            velocity_world = np.array(
                [cosine * vx - sine * vy, sine * vx + cosine * vy]
            )
            velocity_matrix = np.zeros((2, 8))
            velocity_matrix[0, 3] = velocity_matrix[1, 4] = 1
            self._ekf_update(
                velocity_world,
                self._state[3:5],
                velocity_matrix,
                np.diag([0.08, 0.08]) ** 2,
            )

            self._support_legs = 0
            if joint_position is not None and joint_velocity is not None:
                kinematic_velocity, count = self._kinematics.body_velocity_measurement(
                    joint_position,
                    joint_velocity,
                    imu_angular_velocity,
                    reference_velocity=leg_velocity_body,
                )
                if kinematic_velocity is not None:
                    kvx = kinematic_velocity[0] * (
                        scales["forward"] if kinematic_velocity[0] >= 0 else scales["backward"]
                    )
                    kvy = kinematic_velocity[1] * (
                        scales["left"] if kinematic_velocity[1] >= 0 else scales["right"]
                    )
                    kinematic_world = np.array(
                        [cosine * kvx - sine * kvy, sine * kvx + cosine * kvy]
                    )
                    sigma = 0.10 / math.sqrt(count)
                    self._ekf_update(
                        kinematic_world,
                        self._state[3:5],
                        velocity_matrix,
                        np.eye(2) * sigma * sigma,
                    )
                    self._support_legs = count

            observed_yaw = wrap_angle(raw_yaw + self._yaw_frame_offset)
            yaw_matrix = np.zeros((1, 8))
            yaw_matrix[0, 2] = 1
            self._ekf_update(
                [observed_yaw],
                [self._state[2]],
                yaw_matrix,
                np.array([[math.radians(8.0) ** 2]]),
                angle_index=0,
            )

    def pose(self) -> tuple[float, float, float]:
        with self._lock:
            return (
                float(self._state[0]),
                float(self._state[1]),
                math.degrees(float(self._state[2])),
            )

    def uncertainty(self) -> tuple[float, float, float]:
        with self._lock:
            return (
                math.sqrt(max(0.0, float(self._covariance[0, 0]))),
                math.sqrt(max(0.0, float(self._covariance[1, 1]))),
                math.degrees(math.sqrt(max(0.0, float(self._covariance[2, 2])))),
            )

    def velocity_world(self) -> tuple[float, float]:
        with self._lock:
            return float(self._state[3]), float(self._state[4])

    def corrected_yaw_rate(self, raw_wz: float) -> float:
        with self._lock:
            value = float(raw_wz)
            value *= (
                self._scales["turn_left"]
                if value >= 0.0
                else self._scales["turn_right"]
            )
            return value - float(self._state[5])

    @property
    def leg_velocity_candidates(self) -> tuple[tuple[float, float], ...]:
        with self._lock:
            return self._kinematics.last_candidates

    @property
    def leg_velocity_inliers(self) -> tuple[bool, ...]:
        with self._lock:
            return self._kinematics.last_inliers

    @property
    def support_legs(self) -> int:
        with self._lock:
            return self._support_legs
