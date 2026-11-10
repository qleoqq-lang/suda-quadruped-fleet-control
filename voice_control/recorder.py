from __future__ import annotations

import os
from pathlib import Path
import threading
import time
import wave
from typing import Callable

import numpy as np


SAMPLE_RATE = 16000
CHANNELS = 1
DTYPE = "int16"
SAMPLE_WIDTH = 2


def list_devices() -> str:
    import sounddevice as sd

    return "\n".join(
        f"{index}: {device['name']}" for index, device in enumerate(sd.query_devices())
    )


def wait_for_enter(stop_event: threading.Event) -> None:
    """Non-blocking Enter detection so recording can be stopped from the terminal."""
    if os.name == "nt":
        import msvcrt

        while not stop_event.is_set():
            if msvcrt.kbhit():
                try:
                    msvcrt.getch()
                except Exception:
                    pass
                stop_event.set()
                return
            time.sleep(0.05)
        return

    import select
    import sys

    while not stop_event.is_set():
        ready, _, _ = select.select([sys.stdin], [], [], 0.05)
        if ready:
            sys.stdin.readline()
            stop_event.set()
            return


def record_wav(
    path: str | Path,
    *,
    device: int | None = None,
    silence_threshold: float = 500.0,
    silence_seconds: float = 1.5,
    max_seconds: float = 15.0,
    stop_event: threading.Event | None = None,
    on_audio: Callable[[float, float], None] | None = None,
) -> float:
    """Record mono 16 kHz audio and write a WAV file; return duration seconds."""
    import sounddevice as sd

    local_stop = stop_event if stop_event is not None else threading.Event()
    frames: list[bytes] = []
    started = time.monotonic()
    silence_started: float | None = None
    has_speech = False

    def callback(indata, frame_count, time_info, status):
        nonlocal silence_started, has_speech
        now = time.monotonic()
        data = indata.tobytes()
        frames.append(data)
        samples = np.frombuffer(data, dtype=np.int16)
        rms = float(np.sqrt(np.mean(samples.astype(np.float32) ** 2)))
        if on_audio is not None:
            on_audio(rms, now - started)
        if rms > silence_threshold:
            has_speech = True
            silence_started = None
        elif silence_started is None:
            silence_started = now
        elif has_speech and now - silence_started >= silence_seconds:
            local_stop.set()
        elif not has_speech and now - started >= 5.0 and now - silence_started >= silence_seconds:
            local_stop.set()
        if now - started >= max_seconds:
            local_stop.set()

    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype=DTYPE,
        device=device,
        callback=callback,
    ):
        while not local_stop.is_set():
            time.sleep(0.05)

    payload = b"".join(frames)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(target), "wb") as wav_file:
        wav_file.setnchannels(CHANNELS)
        wav_file.setsampwidth(SAMPLE_WIDTH)
        wav_file.setframerate(SAMPLE_RATE)
        wav_file.writeframes(payload)
    return len(payload) / (SAMPLE_RATE * SAMPLE_WIDTH)
