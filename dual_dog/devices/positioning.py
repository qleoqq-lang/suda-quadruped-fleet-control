"""Receive and filter the ten UWB tags used by the robot fleet."""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class TagSample:
    tag: str
    x: float
    y: float


UNIT_TO_METERS = {"m": 1.0, "cm": 0.01, "mm": 0.001}
SUPPORTED_TAGS = tuple(f"TAG{index}" for index in range(1, 11))
WAYPOINT_TAGS = SUPPORTED_TAGS
MIN_MULTI_TAG_STALE_SECONDS = 10.0


def control_stale_seconds(display_stale_seconds: float) -> float:
    """Timeout for ten-tag round-robin control, with a small scheduling margin."""
    return min(30.0, max(12.0, float(display_stale_seconds) * 1.2))


def default_tag_bindings() -> dict[str, str]:
    """Use the fleet's conventional TAGn -> dogNN mapping by default."""
    return {tag: f"dog{index:02d}" for index, tag in enumerate(SUPPORTED_TAGS, start=1)}


def default_direction_locks() -> dict[str, str]:
    return {f"dog{i:02d}": "xy" for i in range(1, 11)}


def default_tag_targets() -> dict[str, tuple[float, float] | None]:
    """Editable stage-map targets are exposed for TAG1 through TAG10."""
    return {tag: None for tag in WAYPOINT_TAGS}


def default_waypoint_snapshots() -> dict[
    str, dict[str, tuple[float, float] | None]
]:
    return {}


def unit_to_meters(unit: str) -> float | None:
    """Return the conversion factor from one tag coordinate unit to meters."""
    return UNIT_TO_METERS.get(unit)


@dataclass(frozen=True)
class TagCalibration:
    """Similarity transform from raw tag coordinates to the robot odometry frame.

    `raw_zero` is the raw tag position at the reference pose and `world_zero` is
    the corrected odometry pose at the same instant. `scale_m_per_unit` converts
    raw coordinate units to meters and `rotation_rad` aligns the raw axes with
    the odometry world frame.
    """

    raw_zero: tuple[float, float]
    world_zero: tuple[float, float]
    rotation_rad: float
    scale_m_per_unit: float

    @classmethod
    def from_displacements(
        cls,
        raw_zero: tuple[float, float],
        raw_moved: tuple[float, float],
        world_zero: tuple[float, float],
        world_moved: tuple[float, float],
    ) -> "TagCalibration":
        raw_zero = tuple(float(v) for v in raw_zero)
        raw_moved = tuple(float(v) for v in raw_moved)
        world_zero = tuple(float(v) for v in world_zero)
        world_moved = tuple(float(v) for v in world_moved)
        raw_delta = (raw_moved[0] - raw_zero[0], raw_moved[1] - raw_zero[1])
        world_delta = (world_moved[0] - world_zero[0], world_moved[1] - world_zero[1])
        raw_length = math.hypot(*raw_delta)
        world_length = math.hypot(*world_delta)
        if (
            not all(math.isfinite(v) for v in (*raw_delta, *world_delta))
            or raw_length < 3.0
            or world_length < 0.02
        ):
            raise ValueError("标签标定位移不足，不能建立坐标系变换")
        rotation = math.atan2(world_delta[1], world_delta[0]) - math.atan2(
            raw_delta[1], raw_delta[0]
        )
        return cls(raw_zero, world_zero, rotation, world_length / raw_length)

    def to_world(self, raw: tuple[float, float]) -> tuple[float, float]:
        x = float(raw[0]) - self.raw_zero[0]
        y = float(raw[1]) - self.raw_zero[1]
        cosine, sine = math.cos(self.rotation_rad), math.sin(self.rotation_rad)
        scale = self.scale_m_per_unit
        return (
            self.world_zero[0] + scale * (cosine * x - sine * y),
            self.world_zero[1] + scale * (sine * x + cosine * y),
        )

    def to_world_delta(self, raw: tuple[float, float]) -> tuple[float, float]:
        x = float(raw[0]) - self.raw_zero[0]
        y = float(raw[1]) - self.raw_zero[1]
        cosine, sine = math.cos(self.rotation_rad), math.sin(self.rotation_rad)
        scale = self.scale_m_per_unit
        return (
            scale * (cosine * x - sine * y),
            scale * (sine * x + cosine * y),
        )


class TagLineParser:
    """Bounded CRLF/LF framing, including fragmented and coalesced serial reads."""

    def __init__(self, max_line: int = 256):
        self.buffer = bytearray()
        self.max_line = max_line
        self.discarding = False
        self.invalid_lines = 0

    def feed(self, data: bytes) -> list[TagSample]:
        result = []
        for byte in data:
            if byte == 10:
                if not self.discarding:
                    try:
                        line = self.buffer.decode("ascii").strip()
                        if line:
                            fields = line.split(",")
                            if len(fields) != 3:
                                raise ValueError("Expected TAGn,x,y")
                            tag = fields[0].upper()
                            if tag not in SUPPORTED_TAGS:
                                raise ValueError("Only TAG1 through TAG10 are supported")
                            x, y = float(fields[1]), float(fields[2])
                            if not all(math.isfinite(v) and abs(v) <= 1e7 for v in (x, y)):
                                raise ValueError("Invalid coordinates")
                            result.append(TagSample(tag, x, y))
                    except (UnicodeError, ValueError):
                        self.invalid_lines += 1
                self.buffer.clear()
                self.discarding = False
            elif not self.discarding:
                self.buffer.append(byte)
                if len(self.buffer) > self.max_line:
                    self.invalid_lines += 1
                    self.buffer.clear()
                    self.discarding = True
        return result


def validate_corners(corners) -> tuple[tuple[float, float], ...]:
    if len(corners) != 4 or any(len(p) != 2 for p in corners):
        raise ValueError("请输入沿场地边界顺序排列的四个角，每个角包含 X、Y。")
    points = tuple(tuple(float(v) for v in p) for p in corners)
    if not all(math.isfinite(v) and abs(v) <= 1e7 for p in points for v in p):
        raise ValueError("四角坐标必须是有限数值，绝对值不超过 10000000。")
    cross = []
    for i in range(4):
        a, b, c = points[i], points[(i + 1) % 4], points[(i + 2) % 4]
        cross.append((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]))
    if not (all(v > 1e-9 for v in cross) or all(v < -1e-9 for v in cross)):
        raise ValueError("四角必须构成不交叉的凸四边形，不能重合或共线。请按 A→B→C→D 绕边界填写。")
    return points


def inside_stage(point, corners) -> bool:
    x, y = point
    cross = [(corners[(i + 1) % 4][0] - corners[i][0]) * (y - corners[i][1])
             - (corners[(i + 1) % 4][1] - corners[i][1]) * (x - corners[i][0]) for i in range(4)]
    return all(v >= -1e-8 for v in cross) or all(v <= 1e-8 for v in cross)


@dataclass
class PositionSettings:
    port: str = ""
    unit: str = "raw"
    xy_confirmed: bool = False
    heading_confirmed: bool = False
    zero_heading_deg: float = 0.0
    return_speed_percent: float = 100.0
    continuous_return: bool = True
    return_latency_ms: float = 300.0
    return_max_seconds: float = 4.0
    stage_configured: bool = False
    corners: tuple = ((0.0, 0.0), (500.0, 0.0), (500.0, 400.0), (0.0, 400.0))
    bindings: dict[str, str] = field(default_factory=default_tag_bindings)
    direction_locks: dict[str, str] = field(default_factory=default_direction_locks)
    tag_targets: dict[str, tuple[float, float] | None] = field(
        default_factory=default_tag_targets
    )
    waypoint_snapshots: dict[
        str, dict[str, tuple[float, float] | None]
    ] = field(default_factory=default_waypoint_snapshots)
    measurement_sigma: float = 8.0
    acceleration_sigma: float = 2.0
    gate_sigma: float = 5.0
    stale_seconds: float = MIN_MULTI_TAG_STALE_SECONDS
    filter_mode: str = "stable"

    @property
    def unit_label(self) -> str:
        return {"raw": "原始单位", "m": "m", "cm": "cm", "mm": "mm"}[self.unit]

    def validate(self) -> None:
        self.corners = validate_corners(self.corners)
        if self.filter_mode not in {"stable", "tracking"}:
            raise ValueError("滤波模式须为 stable 或 tracking。")
        if self.unit not in {"raw", "m", "cm", "mm"}:
            raise ValueError("不支持的坐标单位。")
        if not isinstance(self.port, str) or not isinstance(self.xy_confirmed, bool) or not isinstance(self.heading_confirmed, bool) or not isinstance(self.continuous_return, bool) or not isinstance(self.stage_configured, bool):
            raise ValueError("串口/坐标确认配置格式错误。")
        if not isinstance(self.zero_heading_deg, (int, float)) or not math.isfinite(self.zero_heading_deg) or not -180 <= self.zero_heading_deg <= 180:
            raise ValueError("标零机头方向必须在 -180°～180° 之间。")
        if not isinstance(self.return_speed_percent, (int, float)) or not math.isfinite(self.return_speed_percent) or not 70 <= self.return_speed_percent <= 200:
            raise ValueError("标签回位速度必须在 70%～200% 之间。")
        if not isinstance(self.return_latency_ms, (int, float)) or not math.isfinite(self.return_latency_ms) or not 50 <= self.return_latency_ms <= 6000:
            raise ValueError("UWB 估计延迟必须在 50～6000 ms 之间。")
        if not isinstance(self.return_max_seconds, (int, float)) or not math.isfinite(self.return_max_seconds) or not 0.5 <= self.return_max_seconds <= 4:
            raise ValueError("UWB 单次回位最长时间必须在 0.5～4 秒之间。")
        for value in (self.measurement_sigma, self.acceleration_sigma):
            if not isinstance(value, (int, float)) or not math.isfinite(value) or not 1e-4 <= value <= 1e5:
                raise ValueError("滤波噪声参数须在 0.0001～100000 之间。")
        if not math.isfinite(self.gate_sigma) or not 2 <= self.gate_sigma <= 10:
            raise ValueError("异常点门限须在 2～10 之间。")
        if not math.isfinite(self.stale_seconds) or not 1 <= self.stale_seconds <= 30:
            raise ValueError("过期时间须在 1～30 秒之间。")
        if not isinstance(self.bindings, dict) or set(self.bindings) != set(SUPPORTED_TAGS):
            raise ValueError("本窗口需要 TAG1～TAG10 的完整绑定配置。")
        if any(not isinstance(v, str) for v in self.bindings.values()):
            raise ValueError("机器狗 ID 必须是字符串。")
        if not isinstance(self.direction_locks, dict):
            raise ValueError("纠偏方向配置必须是对象。")
        normalized_locks = default_direction_locks()
        for robot_id, lock in self.direction_locks.items():
            if not isinstance(robot_id, str):
                continue
            normalized_locks[robot_id] = lock if lock in {"xy", "x", "y"} else "xy"
        self.direction_locks = normalized_locks
        bound = [v for v in self.bindings.values() if v]
        if len(set(bound)) != len(bound):
            raise ValueError("不同 TAG 不能绑定到同一只机器狗。")
        if not isinstance(self.tag_targets, dict) or set(self.tag_targets) != set(WAYPOINT_TAGS):
            raise ValueError("地图目标点必须完整包含 TAG1～TAG10。")
        normalized_targets: dict[str, tuple[float, float] | None] = {}
        for tag, point in self.tag_targets.items():
            if point is None:
                normalized_targets[tag] = None
                continue
            if (
                not isinstance(point, (list, tuple))
                or len(point) != 2
                or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in point)
            ):
                raise ValueError(f"{tag} 地图目标点格式无效。")
            normalized = (float(point[0]), float(point[1]))
            if not inside_stage(normalized, self.corners):
                raise ValueError(f"{tag} 地图目标点位于场地边界外。")
            normalized_targets[tag] = normalized
        self.tag_targets = normalized_targets
        if not isinstance(self.waypoint_snapshots, dict) or len(self.waypoint_snapshots) > 50:
            raise ValueError("点位快照必须是对象且不能超过 50 组。")
        normalized_snapshots = {}
        for name, targets in self.waypoint_snapshots.items():
            if not isinstance(name, str) or not name.strip() or len(name.strip()) > 40:
                raise ValueError("点位快照名称必须为 1～40 个字符。")
            if not isinstance(targets, dict) or set(targets) != set(WAYPOINT_TAGS):
                raise ValueError(f"点位快照“{name}”必须完整包含 TAG1～TAG10。")
            normalized = {}
            for tag, point in targets.items():
                if point is None:
                    normalized[tag] = None
                    continue
                if (
                    not isinstance(point, (list, tuple))
                    or len(point) != 2
                    or not all(
                        isinstance(value, (int, float)) and math.isfinite(value)
                        for value in point
                    )
                ):
                    raise ValueError(f"点位快照“{name}”中的 {tag} 格式无效。")
                normalized_point = (float(point[0]), float(point[1]))
                if not inside_stage(normalized_point, self.corners):
                    raise ValueError(f"点位快照“{name}”中的 {tag} 位于场地外。")
                normalized[tag] = normalized_point
            normalized_snapshots[name.strip()] = normalized
        self.waypoint_snapshots = normalized_snapshots

    @classmethod
    def load(cls, path: Path) -> "PositionSettings":
        if not path.exists():
            return cls()
        data = json.loads(path.read_text(encoding="utf-8-sig"))
        # Migrate the colleague version's TAG3/TAG4-only file without losing
        # either binding. Missing rows get the conventional ten-dog mapping.
        bindings = default_tag_bindings()
        stored_bindings = data.get("bindings", {})
        if not isinstance(stored_bindings, dict):
            raise ValueError("TAG 绑定配置必须是对象。")
        for tag, robot_id in stored_bindings.items():
            if tag in bindings:
                bindings[tag] = robot_id
        data["bindings"] = bindings
        locks = default_direction_locks()
        stored_locks = data.get("direction_locks", {})
        if isinstance(stored_locks, dict):
            for robot_id, lock in stored_locks.items():
                if isinstance(robot_id, str) and lock in {"xy", "x", "y"}:
                    locks[robot_id] = lock
        data["direction_locks"] = locks
        targets = default_tag_targets()
        stored_targets = data.get("tag_targets", {})
        if isinstance(stored_targets, dict):
            for tag, point in stored_targets.items():
                if tag in targets:
                    targets[tag] = point
        data["tag_targets"] = targets
        snapshots = data.get("waypoint_snapshots", {})
        if not isinstance(snapshots, dict):
            snapshots = {}
        # Older files only kept TAG1～TAG6 points; extend every snapshot to the
        # ten-dog layout so the whole stage-map editor stays consistent.
        for name, stored_snapshot in snapshots.items():
            if not isinstance(stored_snapshot, dict):
                continue
            merged_snapshot = default_tag_targets()
            for tag, point in stored_snapshot.items():
                if tag in merged_snapshot:
                    merged_snapshot[tag] = point
            snapshots[name] = merged_snapshot
        # Migrate the original single six-point set into a named library.
        if not snapshots and any(point is not None for point in targets.values()):
            snapshots = {"默认点位": dict(targets)}
        data["waypoint_snapshots"] = snapshots
        settings = cls(**data)
        # TAG3/TAG4 used to refresh quickly enough for a 1-3 second timeout.
        # A six/ten-tag receiver is commonly round-robin, so migrate that old
        # value before validating and preserve the other hardware settings.
        if settings.stale_seconds < MIN_MULTI_TAG_STALE_SECONDS:
            settings.stale_seconds = MIN_MULTI_TAG_STALE_SECONDS
        settings.validate()
        return settings

    def save(self, path: Path) -> None:
        self.validate()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(asdict(self), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)


class PositionKalman:
    """Per-tag [x,y,vx,vy] constant-velocity KF, timestamps in monotonic seconds.

    Noise is expressed in the SAME coordinate units as the incoming tag values.
    Rejected samples do not move the displayed position or refresh its valid age.
    A stream gap resets velocity; continuous outliers require manual filter reset.
    """

    def __init__(self, settings: PositionSettings):
        self.settings = settings
        self.state: np.ndarray | None = None
        self.covariance = np.eye(4)
        self.last_seen: float | None = None
        self.last_accepted: float | None = None
        self.rejected = 0

    def update(self, x: float, y: float, now: float) -> bool:
        if not all(math.isfinite(v) for v in (x, y, now)):
            raise ValueError("Kalman input must be finite")
        if self.last_seen is not None and now <= self.last_seen:
            self.rejected += 1
            return False
        reset = self.last_seen is None or now - self.last_seen > self.settings.stale_seconds
        self.last_seen = now
        r = self.settings.measurement_sigma ** 2
        if self.state is None or reset:
            self.state = np.array([x, y, 0.0, 0.0])
            self.covariance = np.diag([r, r, r, r])
            self.last_accepted = now
            return True
        # Bound extrapolation after rejection; never extrapolate just for rendering.
        dt = min(now - self.last_accepted, self.settings.stale_seconds)
        transition = np.eye(4)
        transition[0, 2] = transition[1, 3] = dt
        acceleration = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        predicted = transition @ self.state
        covariance = transition @ self.covariance @ transition.T
        covariance += acceleration @ acceleration.T * self.settings.acceleration_sigma ** 2
        innovation = np.array([x, y]) - predicted[:2]
        innovation_covariance = covariance[:2, :2] + np.eye(2) * r
        mahalanobis = float(innovation @ np.linalg.solve(innovation_covariance, innovation))
        if mahalanobis > self.settings.gate_sigma ** 2:
            self.rejected += 1
            return False
        gain = np.linalg.solve(innovation_covariance, covariance[:2, :]).T
        self.state = predicted + gain @ innovation
        remainder = np.eye(4)
        remainder[:, :2] -= gain
        # Joseph form keeps covariance positive semidefinite under roundoff.
        self.covariance = remainder @ covariance @ remainder.T + gain @ (np.eye(2) * r) @ gain.T
        self.covariance = (self.covariance + self.covariance.T) / 2
        self.last_accepted = now
        return True

    @property
    def position(self) -> tuple[float, float] | None:
        return None if self.state is None else (float(self.state[0]), float(self.state[1]))


class TagTrack:
    def __init__(self, tag: str, settings: PositionSettings):
        self.tag = tag
        self.kalman = StablePositionKalman(settings) if settings.filter_mode == "stable" else PositionKalman(settings)
        self.raw: tuple[float, float] | None = None
        self.last_raw_at: float | None = None
        self.received = 0
        self.accepted_latest = True
        self.history: deque[tuple[float, float]] = deque(maxlen=240)

    def update(self, sample: TagSample, now: float) -> None:
        self.raw = (sample.x, sample.y)
        self.last_raw_at = now
        self.received += 1
        gap = self.kalman.last_seen is not None and now - self.kalman.last_seen > self.kalman.settings.stale_seconds
        self.accepted_latest = self.kalman.update(sample.x, sample.y, now)
        if self.accepted_latest:
            if gap:
                self.history.clear()
            self.history.append(self.kalman.position)

    def stale(self, now: float) -> bool:
        return self.kalman.last_accepted is None or now - self.kalman.last_accepted > self.kalman.settings.stale_seconds

    def raw_stale(self, now: float) -> bool:
        return (
            self.last_raw_at is None
            or now - self.last_raw_at > self.kalman.settings.stale_seconds
        )


class StablePositionKalman(PositionKalman):
    """Display smoothing for sparse/noisy tags, with no velocity extrapolation.

    A causal three-sample median rejects isolated spikes. Position-only Kalman
    updates use low process noise at rest and higher noise for sustained changes.
    This deliberately trades 1-3 or more packets of motion latency for stability.
    A persistent sensor bias is indistinguishable from real motion, not removed.
    """

    def __init__(self, settings: PositionSettings):
        super().__init__(settings)
        self.recent: deque[np.ndarray] = deque(maxlen=3)
        self.trend: deque[tuple[float, np.ndarray]] = deque(maxlen=9)
        self.following = False
        self.follow_packets = 0

    def update(self, x: float, y: float, now: float) -> bool:
        if not all(math.isfinite(v) for v in (x, y, now)):
            raise ValueError("Kalman input must be finite")
        if self.last_seen is not None and now <= self.last_seen:
            self.rejected += 1
            return False
        reset = self.last_seen is None or now - self.last_seen > self.settings.stale_seconds
        self.last_seen = now
        measurement = np.array([x, y], dtype=float)
        sigma = self.settings.measurement_sigma
        r = sigma ** 2
        if self.state is None or reset:
            self.state = np.array([x, y, 0.0, 0.0])
            self.covariance = np.diag([r, r, 0.0, 0.0])
            self.last_accepted = now
            self.recent.clear()
            self.recent.append(measurement)
            self.trend.clear()
            self.trend.append((now, measurement))
            self.following = False
            self.follow_packets = 0
            return True

        # Reject extreme teleports before contaminating the median/history.
        # Ordinary shifts are handled by median + adaptation, never by old velocity.
        reference = np.median(np.array(self.recent), axis=0)
        if np.linalg.norm(measurement - reference) > 4 * self.settings.gate_sigma * sigma:
            self.rejected += 1
            return False
        self.recent.append(measurement)
        self.trend.append((now, measurement))
        recent = np.array(self.recent)
        median = np.median(recent, axis=0)
        if len(recent) == 3:
            residuals = recent - self.state[:2]
            lengths = np.linalg.norm(residuals, axis=1)
            same_direction = all(float(a @ b) > 0.8 * np.linalg.norm(a) * np.linalg.norm(b)
                                 for a, b in zip(residuals[:-1], residuals[1:]))
            # Three consistent samples displaced from the estimate are evidence
            # to follow, not a guarantee that the physical tag actually moved.
            if np.min(lengths) > 2 * sigma and same_direction:
                self.follow_packets = 4
        # Follow consistent slow movement too, without inferring speed from a
        # single noisy increment. This cannot distinguish sensor bias from motion.
        if len(self.trend) >= 6:
            times = np.array([entry[0] for entry in self.trend])
            points = np.array([entry[1] for entry in self.trend])
            direction = np.median(points[-3:], axis=0) - np.median(points[:3], axis=0)
            distance = float(np.linalg.norm(direction))
            if distance >= 0.75 * sigma:
                projection = points @ (direction / distance)
                correlation = float(np.corrcoef(times - times[0], projection)[0, 1])
                if correlation >= 0.9:
                    self.follow_packets = 4
        self.following = self.follow_packets > 0
        self.follow_packets = max(0, self.follow_packets - 1)
        dt = min(now - self.last_accepted, self.settings.stale_seconds)
        process_sigma = self.settings.acceleration_sigma * 0.25
        if self.following:
            process_sigma = max(self.settings.acceleration_sigma, 2 * sigma)
        predicted_covariance = self.covariance[:2, :2] + np.eye(2) * (process_sigma * dt) ** 2
        # Limit trust in any one (median) observation even during catch-up.
        effective_r = max(r, float(np.max(np.diag(predicted_covariance))) * (1 - 0.35) / 0.35)
        innovation_covariance = predicted_covariance + np.eye(2) * effective_r
        gain = np.linalg.solve(innovation_covariance, predicted_covariance).T
        self.state[:2] += gain @ (median - self.state[:2])
        self.state[2:] = 0.0
        remainder = np.eye(2) - gain
        self.covariance[:2, :2] = remainder @ predicted_covariance @ remainder.T + gain @ (np.eye(2) * effective_r) @ gain.T
        self.last_accepted = now
        return True
