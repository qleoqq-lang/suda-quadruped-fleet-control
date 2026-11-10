"""Qt-friendly voice workers, kept separate from the main GUI module."""

from __future__ import annotations

from pathlib import Path
import tempfile
import threading

from PySide6.QtCore import QObject, Signal

from dual_dog.models import Command


class VoiceRecognitionWorker(QObject):
    status_changed = Signal(str)
    result_ready = Signal(str, object)
    failed = Signal(str)
    finished = Signal()

    def __init__(
        self,
        *,
        api_key: str,
        model: str,
        device: int | None,
        silence_seconds: float,
        max_seconds: float,
        available_ids: list[str],
        default_targets: tuple[str, ...],
        speed: float,
        yaw_speed: float,
    ) -> None:
        super().__init__()
        self.api_key = api_key
        self.model = model
        self.device = device
        self.silence_seconds = silence_seconds
        self.max_seconds = max_seconds
        self.available_ids = list(available_ids)
        self.default_targets = tuple(default_targets)
        self.speed = speed
        self.yaw_speed = yaw_speed
        self.stop_event = threading.Event()
        self.cancelled = False
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="voice-asr",
        )

    def start(self) -> None:
        self._thread.start()

    def cancel(self) -> None:
        self.cancelled = True
        self.stop_event.set()

    def _run(self) -> None:
        try:
            from voice_control.asr import recognize_file
            from voice_control.commands import parse_voice_intents
            from voice_control.recorder import record_wav
        except Exception as exc:
            self.failed.emit(f"语音组件不可用：{exc}")
            return
        try:
            with tempfile.TemporaryDirectory(prefix="dual_voice_") as tmp_dir:
                path = Path(tmp_dir) / "command.wav"
                self.status_changed.emit("录音中…请说话，静音自动结束")
                duration = record_wav(
                    path,
                    device=self.device,
                    silence_seconds=self.silence_seconds,
                    max_seconds=self.max_seconds,
                    stop_event=self.stop_event,
                )
                if self.cancelled:
                    self.failed.emit("已取消")
                    return
                if duration < 0.3:
                    self.failed.emit("录音太短，请重新试一次")
                    return
                self.status_changed.emit("识别中…")
                text = recognize_file(path, model=self.model, api_key=self.api_key)
                if self.cancelled:
                    self.failed.emit("已取消")
                    return
                intents = parse_voice_intents(
                    text,
                    self.available_ids,
                    self.default_targets,
                    base_speed=self.speed,
                    base_yaw_speed=self.yaw_speed,
                )
                self.result_ready.emit(text, intents)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


class VoiceExecutor(QObject):
    step_changed = Signal(str)
    finished = Signal(str)
    failed = Signal(str)

    def __init__(self, fleet, intents) -> None:
        super().__init__()
        self.fleet = fleet
        self.intents = tuple(intents)
        self.stop_requested = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="voice-execute",
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self.stop_requested.set()
        if self.fleet is None:
            return
        targets = tuple(
            dict.fromkeys(
                target for intent in self.intents for target in intent.targets
            )
        )
        if targets:
            self.fleet.submit(targets, Command.STOP)

    def _run(self) -> None:
        try:
            from voice_control.robot import execute_intent

            total = len(self.intents)
            for index, intent in enumerate(self.intents, start=1):
                if self.stop_requested.is_set():
                    break
                self.step_changed.emit(f"执行 {index}/{total}：{intent.label}")
                execute_intent(
                    self.fleet,
                    intent,
                    allow_stunts=True,
                    stop_event=self.stop_requested,
                )
            if self.stop_requested.is_set():
                self.finished.emit("已截止")
            else:
                self.finished.emit("执行完成")
        except Exception as exc:
            self.failed.emit(str(exc))
