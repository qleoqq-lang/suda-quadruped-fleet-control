from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum
import math
from typing import Any

from dual_dog.models import Command


class PhaseKind(StrEnum):
    TRICK = "trick"
    POSTURE = "posture"
    CORRECT_YAW = "correct_yaw"
    FORWARD = "forward"
    MOVE = "move"
    TAG_TARGET = "tag_target"
    STOP = "stop"


class MoveDirection(StrEnum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"


@dataclass(frozen=True)
class DancePhase:
    kind: PhaseKind
    duration: float
    command: Command | None = None
    speed: float = 0.0
    command_duration: float = 0.0
    label: str = ""
    start_time: float = 0.0
    targets: tuple[str, ...] = ()
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    return_to_start: bool = False


@dataclass(frozen=True)
class DanceStep:
    kind: PhaseKind
    duration: float = 0.0
    command: Command | None = None
    speed: float = 0.0
    start_time: float | None = None
    targets: tuple[str, ...] = ()
    direction: MoveDirection = MoveDirection.FORWARD
    target_x: float = 0.0
    target_y: float = 0.0
    target_name: str = ""
    return_to_start: bool = False


ALLOWED_DANCE_ACTIONS = {
    Command.LEFT_FLIP, Command.RIGHT_FLIP, Command.PRONK, Command.PACE,
    Command.BOUND, Command.BIPEDAL, Command.INV_BIPEDAL, Command.RUN,
    Command.UP_JUMP, Command.FRONT_JUMP, Command.BACK_JUMP,
    Command.LEFT_JUMP, Command.RIGHT_JUMP, Command.UPRIGHT,
}
ALLOWED_POSTURE_ACTIONS = {Command.STAND, Command.SIT}

# A flip is a short SDK trigger followed by a physical manoeuvre. Unlike the
# configurable non-flip gaits, its track must remain reserved for the safety
# lock even though the trigger itself lasts only a fraction of a second.
MINIMUM_FLIP_TRACK_SECONDS = 0.3
STUNT_RETURN_TRACK_SECONDS = 4.0
TRICK_POST_ACTION_CORRECTION_SECONDS = 3.0


def dance_steps_to_payload(
    steps: tuple[DanceStep, ...] | list[DanceStep],
) -> dict[str, Any]:
    """Return a stable, UTF-8 JSON-compatible choreography document."""
    return {
        "version": 1,
        "steps": [
            {
                "kind": step.kind.value,
                "duration": step.duration,
                "command": step.command.value if step.command is not None else None,
                "speed": step.speed,
                "start_time": step.start_time,
                "targets": list(step.targets),
                "direction": step.direction.value,
                "target_x": step.target_x,
                "target_y": step.target_y,
                "target_name": step.target_name,
                "return_to_start": step.return_to_start,
            }
            for step in steps
        ],
    }


def dance_steps_from_payload(payload: object) -> list[DanceStep]:
    """Parse a choreography document without trusting types from JSON."""
    if not isinstance(payload, dict):
        raise ValueError("编舞方案必须是 JSON 对象")
    if payload.get("version") != 1:
        raise ValueError("不支持的编舞方案版本")
    raw_steps = payload.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError("编舞方案至少需要一条指令")
    if len(raw_steps) > 1000:
        raise ValueError("编舞方案不能超过 1000 条指令")

    result: list[DanceStep] = []
    for index, raw in enumerate(raw_steps, start=1):
        if not isinstance(raw, dict):
            raise ValueError(f"第 {index} 条编舞指令格式无效")
        try:
            kind = PhaseKind(str(raw["kind"]))
            duration = float(raw.get("duration", 0.0))
            speed = float(raw.get("speed", 0.0))
            raw_start = raw.get("start_time")
            start_time = None if raw_start is None else float(raw_start)
            command_value = raw.get("command")
            command = None if command_value is None else Command(str(command_value))
            direction = MoveDirection(str(raw.get("direction", MoveDirection.FORWARD.value)))
            target_x = float(raw.get("target_x", 0.0))
            target_y = float(raw.get("target_y", 0.0))
            target_name = raw.get("target_name", "")
            return_to_start = raw.get("return_to_start", False)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"第 {index} 条编舞指令包含无效字段") from exc
        numeric_values = [duration, speed, target_x, target_y]
        if start_time is not None:
            numeric_values.append(start_time)
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError(f"第 {index} 条编舞指令包含非有限数值")
        if not isinstance(target_name, str) or len(target_name) > 80:
            raise ValueError(f"第 {index} 条编舞指令的点位名称无效")
        if not isinstance(return_to_start, bool):
            raise ValueError(f"第 {index} 条编舞指令的特技回位选项无效")
        raw_targets = raw.get("targets", [])
        if not isinstance(raw_targets, list) or not all(
            isinstance(target, str) and target for target in raw_targets
        ):
            raise ValueError(f"第 {index} 条编舞指令的目标轨道无效")
        result.append(
            DanceStep(
                kind=kind,
                duration=duration,
                command=command,
                speed=speed,
                start_time=start_time,
                targets=tuple(dict.fromkeys(raw_targets)),
                direction=direction,
                target_x=target_x,
                target_y=target_y,
                target_name=target_name,
                return_to_start=return_to_start,
            )
        )
    return result


def merge_dance_documents(
    current: tuple[DanceStep, ...] | list[DanceStep],
    documents: list[tuple[str, object]],
    trick_lock_seconds: float,
    default_targets: tuple[str, ...] | list[str],
) -> list[DanceStep]:
    """Merge independent timelines without offsetting them or replacing cues.

    Resolve implicit starts within each source file before merging so that an
    earlier file cannot shift another dog's sequence. Overlaps remain editable.
    Validate the entire batch before the caller changes the editor.
    """
    merged = list(current)
    for name, payload in documents:
        try:
            steps = dance_steps_from_payload(payload)
            compiled = _compile_multitrack_dance(
                steps, trick_lock_seconds, default_targets,
            )
            starts = {index: phase.start_time for index, phase in compiled}
            merged.extend(
                replace(step, start_time=starts[index])
                if step.start_time is None else step
                for index, step in enumerate(steps, start=1)
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name}：{exc}") from exc
        if len(merged) > 1000:
            raise ValueError("合并后超过 1000 条指令，请减少本次导入的方案")
    return merged


def build_custom_dance(
    steps: tuple[DanceStep, ...] | list[DanceStep],
    trick_lock_seconds: float,
) -> tuple[DancePhase, ...]:
    if not steps:
        raise ValueError("编舞至少需要一个步骤")
    if trick_lock_seconds <= 0:
        raise ValueError("特技安全锁定时间必须大于 0")
    phases: list[DancePhase] = []
    for index, step in enumerate(steps, start=1):
        if step.kind is PhaseKind.TRICK:
            if step.command not in ALLOWED_DANCE_ACTIONS:
                raise ValueError(f"步骤 {index} 的动作不受支持")
            is_flip = step.command in {Command.LEFT_FLIP, Command.RIGHT_FLIP}
            hold = 0.0 if is_flip else step.duration
            if not is_flip and not 0.5 <= hold <= 10.0:
                raise ValueError(f"步骤 {index} 的非空翻动作时间必须为 0.5～10 秒")
            # Non-flip duration is the user-visible timeline occupancy.  The
            # scheduler sends STOP at its end before starting the next cue,
            # which safely clears the worker's manual-command lock.  Keeping
            # the hidden lock here used to make adjacent visible cues look
            # non-overlapping while the validator rejected them.
            phase_duration = (
                max(trick_lock_seconds, MINIMUM_FLIP_TRACK_SECONDS)
                if is_flip else hold
            )
            if step.return_to_start:
                phase_duration = (
                    phase_duration
                    + (0.0 if is_flip else trick_lock_seconds)
                    + STUNT_RETURN_TRACK_SECONDS
                )
            label = (
                f"{step.command.value}（安全占用 {phase_duration:.1f}s）"
                if is_flip else f"{step.command.value}（{hold:.1f}s）"
            )
            if step.return_to_start:
                label += " · 动作后回标零"
            phases.append(
                DancePhase(
                    PhaseKind.TRICK, phase_duration, step.command,
                    command_duration=hold, label=label,
                    return_to_start=step.return_to_start,
                )
            )
        elif step.kind is PhaseKind.POSTURE:
            if step.command not in ALLOWED_POSTURE_ACTIONS:
                raise ValueError(f"步骤 {index} 的姿态动作不受支持")
            if not 0.5 <= step.duration <= 15.0:
                raise ValueError(f"步骤 {index} 的起立/坐下等待时间必须为 0.5～15 秒")
            label = "起立" if step.command is Command.STAND else "坐下"
            phases.append(
                DancePhase(
                    step.kind,
                    step.duration,
                    command=step.command,
                    command_duration=step.duration,
                    label=f"{label}（等待 {step.duration:.1f}s）",
                )
            )
        elif step.kind is PhaseKind.CORRECT_YAW:
            if not 0.1 <= step.duration <= 60.0:
                raise ValueError(f"步骤 {index} 的纠偏时间必须为 0.1～60 秒")
            phases.append(DancePhase(step.kind, step.duration, label="停止并纠正方向"))
        elif step.kind is PhaseKind.FORWARD:
            if not 0.1 <= step.duration <= 60.0:
                raise ValueError(f"步骤 {index} 的前进时间必须为 0.1～60 秒")
            if step.speed <= 0:
                raise ValueError(f"步骤 {index} 的前进速度必须大于 0")
            phases.append(
                DancePhase(
                    step.kind, step.duration, speed=step.speed, label="前进",
                    x=step.speed,
                )
            )
        elif step.kind is PhaseKind.MOVE:
            if not 0.1 <= step.duration <= 60.0:
                raise ValueError(f"步骤 {index} 的移动时间必须为 0.1～60 秒")
            if step.speed <= 0:
                raise ValueError(f"步骤 {index} 的移动速度必须大于 0")
            vectors = {
                MoveDirection.FORWARD: (step.speed, 0.0, 0.0, "前进"),
                MoveDirection.BACKWARD: (-step.speed, 0.0, 0.0, "后退"),
                MoveDirection.LEFT: (0.0, step.speed, 0.0, "左移"),
                MoveDirection.RIGHT: (0.0, -step.speed, 0.0, "右移"),
                MoveDirection.TURN_LEFT: (0.0, 0.0, step.speed, "左转"),
                MoveDirection.TURN_RIGHT: (0.0, 0.0, -step.speed, "右转"),
            }
            x, y, yaw, label = vectors[step.direction]
            phases.append(
                DancePhase(
                    step.kind, step.duration, speed=step.speed, label=label,
                    x=x, y=y, yaw=yaw,
                )
            )
        elif step.kind is PhaseKind.TAG_TARGET:
            if not 1.0 <= step.duration <= 60.0:
                raise ValueError(f"步骤 {index} 的 TAG 目标点时间必须为 1～60 秒")
            if not 70.0 <= step.speed <= 200.0:
                raise ValueError(f"步骤 {index} 的 TAG 回位速度必须为 70%～200%")
            if not all(math.isfinite(value) for value in (step.target_x, step.target_y)):
                raise ValueError(f"步骤 {index} 的 TAG 目标坐标无效")
            phases.append(
                DancePhase(
                    step.kind,
                    step.duration,
                    speed=step.speed,
                    label=(
                        f"TAG目标点[{step.target_name}] "
                        if step.target_name else "TAG目标点 "
                    ) + f"({step.target_x:.3f}, {step.target_y:.3f})m",
                    x=step.target_x,
                    y=step.target_y,
                )
            )
        elif step.kind is PhaseKind.STOP:
            phases.append(DancePhase(step.kind, max(0.0, step.duration), label="停止"))
        else:
            raise ValueError(f"步骤 {index} 类型不受支持")
    return tuple(phases)


def _compile_multitrack_dance(
    steps: tuple[DanceStep, ...] | list[DanceStep],
    trick_lock_seconds: float,
    default_targets: tuple[str, ...] | list[str],
    post_action_correction_seconds: float = 0.0,
) -> list[tuple[int, DancePhase]]:
    """Keep source indices so generated corrections can move with their action."""
    if not steps:
        raise ValueError("编舞至少需要一个步骤")
    if (
        not math.isfinite(post_action_correction_seconds)
        or not 0.0 <= post_action_correction_seconds <= 60.0
    ):
        raise ValueError("动作后纠偏时间必须为 0～60 秒")
    available = tuple(dict.fromkeys(default_targets))
    if not available:
        raise ValueError("编舞至少需要一个目标设备")
    available_set = set(available)
    compiled: list[tuple[int, DancePhase]] = []
    cursor = 0.0
    for index, step in enumerate(steps, start=1):
        phase = build_custom_dance([step], trick_lock_seconds)[0]
        start = cursor if step.start_time is None else float(step.start_time)
        if not math.isfinite(start) or start < 0:
            raise ValueError(f"步骤 {index} 的开始时间不能为负数")
        targets = tuple(dict.fromkeys(step.targets or available))
        unknown = set(targets) - available_set
        if unknown:
            raise ValueError(
                f"步骤 {index} 包含未勾选或不存在的轨道：{', '.join(sorted(unknown))}"
            )
        phase = replace(phase, start_time=start, targets=targets)
        compiled.append((index, phase))
        cursor = max(cursor, start + phase.duration)

    if post_action_correction_seconds > 0.0:
        source_order = sorted(compiled, key=lambda item: (item[1].start_time, item[0]))
        for position, (source_index, phase) in enumerate(source_order):
            if phase.kind is not PhaseKind.TRICK:
                continue
            correction_start = phase.start_time + phase.duration
            already_present = any(
                candidate.kind is PhaseKind.CORRECT_YAW
                and bool(set(candidate.targets) & set(phase.targets))
                and correction_start - 1e-6
                <= candidate.start_time
                <= correction_start + post_action_correction_seconds + 1e-6
                for _, candidate in source_order[position + 1:]
            )
            if already_present:
                continue
            compiled.append(
                (
                    len(steps) + source_index,
                    DancePhase(
                        PhaseKind.CORRECT_YAW,
                        post_action_correction_seconds,
                        label="停止并纠正方向",
                        start_time=correction_start,
                        targets=phase.targets,
                    ),
                )
            )
    ordered = sorted(
        compiled,
        key=lambda item: (item[1].start_time, item[0]),
    )
    for position in range(1, len(ordered)):
        _, previous_phase = ordered[position - 1]
        index, phase = ordered[position]
        if (
            phase.kind is PhaseKind.CORRECT_YAW
            and previous_phase.kind is PhaseKind.TRICK
            and set(phase.targets) & set(previous_phase.targets)
        ):
            ordered[position] = (
                index,
                replace(
                    phase,
                    duration=max(TRICK_POST_ACTION_CORRECTION_SECONDS, phase.duration),
                ),
            )
    return ordered


def build_multitrack_dance(
    steps: tuple[DanceStep, ...] | list[DanceStep],
    trick_lock_seconds: float,
    default_targets: tuple[str, ...] | list[str],
    allow_overlap: bool = False,
    post_action_correction_seconds: float = 0.0,
) -> tuple[DancePhase, ...]:
    phases = tuple(phase for _, phase in _compile_multitrack_dance(
        steps, trick_lock_seconds, default_targets, post_action_correction_seconds,
    ))
    # Validate after all hidden corrections and minimum durations are applied.
    if not allow_overlap:
        conflicts = overlapping_dance_cue_messages(phases)
        if conflicts:
            raise ValueError(conflicts[0])
    return phases


def resolve_dance_time_conflicts(
    steps: tuple[DanceStep, ...] | list[DanceStep],
    trick_lock_seconds: float,
    default_targets: tuple[str, ...] | list[str],
    post_action_correction_seconds: float = TRICK_POST_ACTION_CORRECTION_SECONDS,
) -> list[DanceStep]:
    """Delay conflicting cues, preserving all other fields and per-track order.

    Reserve the actual compiled action plus its generated correction as one
    unit. Recompile after scheduling: moving a cue can change whether an
    explicit yaw correction suppresses an automatic one. Reservations only
    grow, and starts round upwards to the editor's hundredth-second precision.
    Independent tracks retain concurrency; multi-target cues stay together.
    """
    compiled = _compile_multitrack_dance(
        steps, trick_lock_seconds, default_targets, post_action_correction_seconds,
    )
    if not overlapping_dance_cue_messages(tuple(phase for _, phase in compiled)):
        return list(steps)
    count = len(steps)
    original = {index: phase for index, phase in compiled if index <= count}
    order = sorted(original, key=lambda index: (original[index].start_time, index))
    result = [replace(step, start_time=original[index].start_time)
              for index, step in enumerate(steps, start=1)]
    reserved = {index: 0.0 for index in original}
    # Each source has at most one generated correction and one minimum-duration
    # extension. This bound also protects the UI if compilation rules change.
    for _ in range(2 * count + 4):
        for phase_index, phase in compiled:
            owner = phase_index if phase_index <= count else phase_index - count
            reserved[owner] = max(
                reserved[owner],
                phase.start_time + phase.duration - result[owner - 1].start_time,
            )
        available: dict[str, float] = {}
        for index in order:
            step = result[index - 1]
            targets = original[index].targets
            required = max(step.start_time, *(available.get(target, 0.0) for target in targets))
            start = step.start_time
            if required > start + 1e-6:
                start = math.ceil((required - 1e-9) * 100.0) / 100.0
                result[index - 1] = replace(step, start_time=start)
            for target in targets:
                available[target] = start + reserved[index]
        compiled = _compile_multitrack_dance(
            result, trick_lock_seconds, default_targets, post_action_correction_seconds,
        )
        if not overlapping_dance_cue_messages(tuple(phase for _, phase in compiled)):
            return result
    raise ValueError("无法自动消除全部时间冲突，原编舞未修改")


def keep_connected_dance_phases(
    phases: tuple[DancePhase, ...],
    connected: set[str],
) -> tuple[DancePhase, ...]:
    """Drop cues that only target unavailable robots and keep the rest intact."""
    kept: list[DancePhase] = []
    for phase in phases:
        targets = tuple(
            dict.fromkeys(target for target in phase.targets if target in connected)
        )
        if targets:
            kept.append(replace(phase, targets=targets))
    return tuple(kept)


def overlapping_dance_cue_messages(
    phases: tuple[DancePhase, ...],
) -> list[str]:
    """Return human-readable overlap messages for the compiled cues in use."""
    messages: list[str] = []
    occupancy: dict[str, list[tuple[float, float, int]]] = {}
    for order, phase in enumerate(phases, start=1):
        if phase.duration <= 0:
            continue
        start = phase.start_time
        end = start + phase.duration
        for target in phase.targets:
            for prior_start, prior_end, prior_order in occupancy.setdefault(target, []):
                if start < prior_end - 1e-6 and prior_start < end - 1e-6:
                    messages.append(
                        f"轨道 {target} 时间重叠：步骤 {prior_order} "
                        f"[{prior_start:.2f}, {prior_end:.2f}) 与步骤 {order} "
                        f"[{start:.2f}, {end:.2f})"
                    )
            occupancy[target].append((start, end, order))
    return messages


def build_flip_forward_dance(
    correction_seconds: float,
    forward_seconds: float,
    trick_lock_seconds: float,
    forward_speed: float = 0.5,
    first_command: Command = Command.LEFT_FLIP,
    second_command: Command = Command.RIGHT_FLIP,
    non_flip_action_seconds: float = 2.0,
) -> tuple[DancePhase, ...]:
    if correction_seconds <= 0 or forward_seconds <= 0:
        raise ValueError("纠偏和前进时间必须大于 0")
    if trick_lock_seconds <= 0:
        raise ValueError("特技安全锁定时间必须大于 0")
    if forward_speed <= 0:
        raise ValueError("前进速度必须大于 0")
    if first_command not in ALLOWED_DANCE_ACTIONS or second_command not in ALLOWED_DANCE_ACTIONS:
        raise ValueError("编舞动作必须是受支持的单次特技或步态")
    if not 0.5 <= non_flip_action_seconds <= 10.0:
        raise ValueError("非空翻动作持续时间必须在 0.5 到 10 秒之间")
    # A short margin prevents the next phase racing the worker's unlock tick.
    return build_custom_dance(
        [
            DanceStep(PhaseKind.TRICK, non_flip_action_seconds, first_command),
            DanceStep(PhaseKind.CORRECT_YAW, correction_seconds),
            DanceStep(PhaseKind.FORWARD, forward_seconds, speed=forward_speed),
            DanceStep(PhaseKind.STOP, 0.1),
            DanceStep(PhaseKind.TRICK, non_flip_action_seconds, second_command),
            DanceStep(PhaseKind.CORRECT_YAW, correction_seconds),
            DanceStep(PhaseKind.FORWARD, forward_seconds, speed=forward_speed),
            DanceStep(PhaseKind.STOP, 0.0),
        ],
        trick_lock_seconds,
    )
