from __future__ import annotations

from dataclasses import dataclass, field
import re
from typing import Iterable, Sequence

from dual_dog.models import Command


@dataclass(frozen=True)
class VoiceRule:
    label: str
    aliases: tuple[str, ...]
    command: Command
    params: dict[str, float] = field(default_factory=dict)
    duration_s: float | None = None
    stunt: bool = False


STUNT_COMMANDS = frozenset(
    {
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
)


VOICE_RULES: tuple[VoiceRule, ...] = (
    VoiceRule("急停", ("急停", "紧急停止", "立刻停止", "马上停止", "立即停止", "刹车", "阻尼"), Command.DAMPING),
    VoiceRule("停止", ("停止", "停下", "停", "别动", "不要动"), Command.STOP),
    VoiceRule("翻身站起", ("翻身", "翻身站起", "翻正", "恢复直立"), Command.UPRIGHT, stunt=True),
    VoiceRule("站立", ("站立", "站起来", "起立", "站起", "起身", "立正"), Command.STAND),
    VoiceRule("趴下", ("坐下", "趴下", "趴", "蹲下"), Command.SIT),
    VoiceRule("前进", ("前进", "直行", "直走", "直线前进", "往前走", "向前走", "向前", "往前走一点"), Command.VELOCITY, {"x": 1.0}, 2.0),
    VoiceRule("后退", ("后退", "往后走", "向后退", "向后", "倒车"), Command.VELOCITY, {"x": -1.0}, 2.0),
    VoiceRule("左移", ("左移", "往左走", "向左走", "向左移动", "往左移动"), Command.VELOCITY, {"y": 1.0}, 2.0),
    VoiceRule("右移", ("右移", "往右走", "向右走", "向右移动", "往右移动"), Command.VELOCITY, {"y": -1.0}, 2.0),
    VoiceRule("左转", ("左转", "向左转", "往左转", "向左旋转"), Command.VELOCITY, {"yaw": 1.0}, 1.0),
    VoiceRule("右转", ("右转", "向右转", "往右转", "向右旋转"), Command.VELOCITY, {"yaw": -1.0}, 1.0),
    VoiceRule("行走", ("走路", "行走", "开始走", "慢走", "走"), Command.WALK),
    VoiceRule("跑步", ("跑步", "快跑", "奔跑", "跑"), Command.RUN, stunt=True),
    VoiceRule("左翻", ("左翻", "向左翻", "往左翻", "左边翻"), Command.LEFT_FLIP, stunt=True),
    VoiceRule("右翻", ("右翻", "向右翻", "往右翻", "右边翻"), Command.RIGHT_FLIP, stunt=True),
    VoiceRule("原地跳", ("原地跳", "原地跳跃", "上跳", "跳高"), Command.UP_JUMP, stunt=True),
    VoiceRule("前跳", ("前跳", "向前跳", "往前跳"), Command.FRONT_JUMP, stunt=True),
    VoiceRule("后跳", ("后跳", "向后跳", "往后跳"), Command.BACK_JUMP, stunt=True),
    VoiceRule("左跳", ("左跳", "向左跳", "往左跳"), Command.LEFT_JUMP, stunt=True),
    VoiceRule("右跳", ("右跳", "向右跳", "往右跳"), Command.RIGHT_JUMP, stunt=True),
    VoiceRule("弹跳", ("弹跳", "蹦跳", "跳跃", "跳一下"), Command.PRONK, stunt=True),
    VoiceRule("踱步", ("踱步", "走马", "碎步"), Command.PACE, stunt=True),
    VoiceRule("跳步", ("跳步", "跳跃前进", "快跳"), Command.BOUND, stunt=True),
    VoiceRule("双足正立", ("双足站立", "双足行走", "双足正立", "人形行走", "直立行走"), Command.BIPEDAL, stunt=True),
    VoiceRule("双足倒立", ("倒立", "双足倒立", "倒立行走"), Command.INV_BIPEDAL, stunt=True),
)


@dataclass(frozen=True)
class VoiceIntent:
    targets: tuple[str, ...]
    command: Command
    params: dict[str, float]
    duration_s: float | None = None
    label: str = ""
    stunt: bool = False


@dataclass(frozen=True)
class VoiceMacroStep:
    command: Command
    duration_s: float | None = None
    speed: float | None = None


@dataclass(frozen=True)
class VoiceMacro:
    label: str
    aliases: tuple[str, ...]
    steps: tuple[VoiceMacroStep, ...]


VOICE_MACROS: tuple[VoiceMacro, ...] = (
    VoiceMacro(
        "全军列阵",
        ("全军列阵", "全体列阵", "全军集合", "全体集合"),
        (
            VoiceMacroStep(Command.STAND),
            VoiceMacroStep(Command.VELOCITY, duration_s=5.0, speed=1.0),
        ),
    ),
)


_PUNCT_RE = re.compile(r"[\s，。！？、；：,.!?;:'\"“”‘’·\-_—]+")
_ARABIC_TARGET_RE = re.compile(r"(\d{1,2})\s*(?:号|台|只|个)")
_CN_TARGET_RE = re.compile(r"([一二两三四五六七八九十]+)\s*(?:号|台|只|个)")
_DOG_ID_RE = re.compile(r"dog\s*(\d{1,2})", re.IGNORECASE)
_CN_NUMERALS = {
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
    "十": 10,
}


def normalize(text: str) -> str:
    return _PUNCT_RE.sub("", text.strip().lower())


def _chinese_to_int(token: str) -> int:
    return _CN_NUMERALS.get(token, 0)


def parse_targets(
    text: str,
    available_ids: Sequence[str],
    all_targets: Sequence[str] | None = None,
) -> tuple[str, ...] | None:
    """Return voice-selected robot ids, or None when the text names no robot."""
    normalized = normalize(text)
    if any(keyword in normalized for keyword in ("全部", "所有", "全体", "全队", "全员")):
        return tuple(all_targets) if all_targets is not None else tuple(available_ids)

    found: set[str] = set()
    for match in _ARABIC_TARGET_RE.finditer(normalized):
        index = int(match.group(1))
        if 1 <= index <= len(available_ids):
            found.add(available_ids[index - 1])
    for match in _CN_TARGET_RE.finditer(normalized):
        index = _chinese_to_int(match.group(1))
        if 1 <= index <= len(available_ids):
            found.add(available_ids[index - 1])
    for match in _DOG_ID_RE.finditer(normalized):
        index = int(match.group(1))
        if 1 <= index <= len(available_ids):
            found.add(available_ids[index - 1])
    if not found:
        return None
    return tuple(sorted(found, key=lambda robot_id: available_ids.index(robot_id)))


def _duration_from_text(text: str, default: float) -> float:
    match = re.search(r"(\d+(?:\.\d+)?)\s*秒", text)
    if not match:
        return default
    return min(10.0, max(0.5, float(match.group(1))))


def _distance_from_text(text: str) -> float | None:
    match = re.search(r"(\d+(?:\.\d+)?)\s*米", text)
    if not match:
        return None
    return float(match.group(1))


def _speed_from_text(text: str, base_speed: float) -> float:
    if any(word in text for word in ("快点", "快速", "快一点")):
        return min(0.8, base_speed * 2.0)
    if any(word in text for word in ("慢点", "慢速", "慢一点")):
        return max(0.1, base_speed * 0.5)
    return base_speed


def parse_voice(
    text: str,
    available_ids: Sequence[str],
    default_targets: Iterable[str],
    *,
    base_speed: float = 0.3,
    base_yaw_speed: float = 0.4,
) -> VoiceIntent | None:
    normalized = normalize(text)
    if not normalized:
        return None

    rule: VoiceRule | None = None
    for candidate in VOICE_RULES:
        if any(alias in normalized for alias in candidate.aliases):
            rule = candidate
            break
    if rule is None:
        return None

    targets = parse_targets(
        normalized,
        available_ids,
        all_targets=tuple(default_targets),
    )
    if targets is None:
        targets = tuple(default_targets)

    if rule.command is Command.VELOCITY:
        speed = _speed_from_text(normalized, base_speed)
        if "yaw" in rule.params:
            speed = _speed_from_text(normalized, base_yaw_speed)
        params = {key: value * speed for key, value in rule.params.items()}
        distance = _distance_from_text(normalized)
        if distance is not None and "yaw" not in rule.params:
            duration = min(30.0, max(0.5, distance / max(speed, 0.05)))
        else:
            duration = _duration_from_text(normalized, rule.duration_s or 2.0)
        return VoiceIntent(
            targets=targets,
            command=Command.VELOCITY,
            params=params,
            duration_s=duration,
            label=rule.label,
        )

    duration = rule.duration_s
    if "秒" in normalized and rule.command in STUNT_COMMANDS:
        duration = _duration_from_text(normalized, rule.duration_s or 2.0)
    return VoiceIntent(
        targets=targets,
        command=rule.command,
        params=dict(rule.params),
        duration_s=duration,
        label=rule.label,
        stunt=rule.stunt or rule.command in STUNT_COMMANDS,
    )


def parse_macro(
    text: str,
    available_ids: Sequence[str],
    default_targets: Iterable[str],
    *,
    base_speed: float = 0.3,
) -> tuple[VoiceIntent, ...] | None:
    normalized = normalize(text)
    if not normalized:
        return None
    macro = next(
        (
            candidate for candidate in VOICE_MACROS
            if any(alias in normalized for alias in candidate.aliases)
        ),
        None,
    )
    if macro is None:
        return None
    targets = parse_targets(
        normalized,
        available_ids,
        all_targets=tuple(default_targets),
    )
    if targets is None:
        targets = tuple(default_targets)

    intents: list[VoiceIntent] = []
    for index, step in enumerate(macro.steps, start=1):
        label = f"{macro.label}·{index}"
        if step.command is Command.VELOCITY:
            speed = base_speed * (step.speed or 1.0)
            intents.append(
                VoiceIntent(
                    targets=targets,
                    command=Command.VELOCITY,
                    params={"x": speed},
                    duration_s=max(0.5, step.duration_s or 2.0),
                    label=label,
                )
            )
        else:
            intents.append(
                VoiceIntent(
                    targets=targets,
                    command=step.command,
                    params={},
                    duration_s=step.duration_s,
                    label=label,
                    stunt=step.command in STUNT_COMMANDS,
                )
            )
    return tuple(intents)


def parse_voice_intents(
    text: str,
    available_ids: Sequence[str],
    default_targets: Iterable[str],
    *,
    base_speed: float = 0.3,
    base_yaw_speed: float = 0.4,
) -> tuple[VoiceIntent, ...]:
    macro = parse_macro(
        text,
        available_ids,
        default_targets,
        base_speed=base_speed,
    )
    if macro is not None:
        return macro
    intent = parse_voice(
        text,
        available_ids,
        default_targets,
        base_speed=base_speed,
        base_yaw_speed=base_yaw_speed,
    )
    return (intent,) if intent is not None else ()
