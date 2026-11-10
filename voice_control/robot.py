from __future__ import annotations

import time
import threading

from dual_dog.control.controller import FleetController
from dual_dog.models import Command

from .commands import STUNT_COMMANDS, VoiceIntent


VELOCITY_REFRESH_S = 0.2


def ensure_targets_connected(fleet: FleetController, targets: tuple[str, ...]) -> tuple[str, ...]:
    snapshots = {snapshot.robot_id: snapshot for snapshot in fleet.snapshots()}
    connected = tuple(
        target for target in targets
        if snapshots.get(target) is not None and snapshots[target].connected
    )
    if not connected:
        raise RuntimeError("没有已连接的目标机器狗")
    offline = tuple(target for target in targets if target not in connected)
    if offline:
        raise RuntimeError(f"以下目标未连接: {', '.join(offline)}")
    return connected


def execute_intent(
    fleet: FleetController,
    intent: VoiceIntent,
    *,
    allow_stunts: bool = False,
    velocity_refresh_s: float = VELOCITY_REFRESH_S,
    stop_event: threading.Event | None = None,
) -> None:
    if intent.command in STUNT_COMMANDS and not allow_stunts:
        raise PermissionError("特技指令已忽略，请确认场地和急停准备后使用 --allow-stunts")

    targets = ensure_targets_connected(fleet, intent.targets)
    if intent.command is Command.VELOCITY:
        duration = max(0.5, intent.duration_s or 2.0)
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline:
                if stop_event is not None and stop_event.is_set():
                    break
                fleet.submit(targets, Command.VELOCITY, **intent.params)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                wait_seconds = min(velocity_refresh_s, remaining)
                if stop_event is not None:
                    stop_event.wait(timeout=wait_seconds)
                else:
                    time.sleep(wait_seconds)
        finally:
            fleet.submit(targets, Command.STOP)
        return

    params = dict(intent.params)
    if (
        intent.command in STUNT_COMMANDS
        and intent.command not in {Command.LEFT_FLIP, Command.RIGHT_FLIP}
    ):
        params.setdefault("duration_s", intent.duration_s or 2.0)
    fleet.submit(targets, intent.command, **params)
