from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
import time
from typing import Iterable

from .models import Telemetry


class StraightLineCsvRecorder:
    """Write exact commands and proprioceptive straight-line observations."""

    LEG_NAMES = ("fl", "fr", "hl", "hr")

    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = directory / f"straight_line_{stamp}.csv"
        self._stream = self.path.open("w", newline="", encoding="utf-8-sig")
        fields = [
            "wall_time", "monotonic_s", "robot_id", "connected",
            "cmd_x", "cmd_y", "cmd_yaw", "imu_yaw_rad",
            "gyro_z_raw_rad_s", "gyro_z_corrected_rad_s",
            "sdk_vx_body", "sdk_vy_body", "ekf_vx_world", "ekf_vy_world",
            "ekf_x", "ekf_y", "ekf_yaw_deg", "sigma_x", "sigma_y",
            "support_legs", "adaptive_direction", "adaptive_cross_velocity",
            "adaptive_bias", "adaptive_correction_x", "adaptive_correction_y",
            "motion_state", "gait", "sub_gait",
        ]
        for name in self.LEG_NAMES:
            fields.extend((f"{name}_vx_body", f"{name}_vy_body", f"{name}_used"))
        fields.extend(("action", "feedback", "protection"))
        self._writer = csv.DictWriter(self._stream, fieldnames=fields)
        self._writer.writeheader()
        self._rows_since_flush = 0

    @staticmethod
    def _pair(value, default=(None, None)):
        if value is None:
            return default
        return value[0], value[1]

    def record(self, snapshots: Iterable[Telemetry]) -> None:
        wall_time = datetime.now().isoformat(timespec="milliseconds")
        monotonic = time.monotonic()
        for data in snapshots:
            cmd_x, cmd_y, cmd_yaw = (
                data.commanded_velocity
                if data.commanded_velocity is not None
                else (None, None, None)
            )
            sdk_vx, sdk_vy = self._pair(data.leg_velocity_body)
            ekf_vx, ekf_vy = self._pair(data.corrected_velocity_world)
            pose = data.corrected_pose or (None, None, None)
            sigma = data.odom_uncertainty or (None, None, None)
            adaptive_x, adaptive_y = self._pair(data.adaptive_slip_correction)
            row = {
                "wall_time": wall_time,
                "monotonic_s": f"{monotonic:.6f}",
                "robot_id": data.robot_id,
                "connected": int(data.connected),
                "cmd_x": cmd_x,
                "cmd_y": cmd_y,
                "cmd_yaw": cmd_yaw,
                "imu_yaw_rad": None if data.rpy is None else data.rpy[2],
                "gyro_z_raw_rad_s": (
                    None
                    if data.imu_angular_velocity is None
                    else data.imu_angular_velocity[2]
                ),
                "gyro_z_corrected_rad_s": data.corrected_yaw_rate,
                "sdk_vx_body": sdk_vx,
                "sdk_vy_body": sdk_vy,
                "ekf_vx_world": ekf_vx,
                "ekf_vy_world": ekf_vy,
                "ekf_x": pose[0],
                "ekf_y": pose[1],
                "ekf_yaw_deg": pose[2],
                "sigma_x": sigma[0],
                "sigma_y": sigma[1],
                "support_legs": data.odom_support_legs,
                "adaptive_direction": data.adaptive_slip_direction,
                "adaptive_cross_velocity": data.adaptive_slip_cross_velocity,
                "adaptive_bias": data.adaptive_slip_bias,
                "adaptive_correction_x": adaptive_x,
                "adaptive_correction_y": adaptive_y,
                "motion_state": data.motion_state,
                "gait": data.gait,
                "sub_gait": data.sub_gait,
                "action": data.action,
                "feedback": data.feedback,
                "protection": data.protection,
            }
            candidates = data.leg_velocity_candidates_body or ()
            inliers = data.leg_velocity_inliers or ()
            for index, name in enumerate(self.LEG_NAMES):
                candidate = candidates[index] if index < len(candidates) else (None, None)
                row[f"{name}_vx_body"] = candidate[0]
                row[f"{name}_vy_body"] = candidate[1]
                row[f"{name}_used"] = (
                    int(inliers[index]) if index < len(inliers) else None
                )
            self._writer.writerow(row)
            self._rows_since_flush += 1
        if self._rows_since_flush >= 20:
            self._stream.flush()
            self._rows_since_flush = 0

    def close(self) -> None:
        if not self._stream.closed:
            self._stream.flush()
            self._stream.close()
