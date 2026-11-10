"""Generate the original, royalty-free Cluster Pulse demo track."""

from array import array
import math
from pathlib import Path
import random
import struct
import wave


SAMPLE_RATE = 44_100
BPM = 120
BEAT = 60.0 / BPM
BARS = 16
DURATION = BARS * 4 * BEAT


def main() -> None:
    total = int(DURATION * SAMPLE_RATE)
    mix = array("f", [0.0]) * total
    rng = random.Random(20260818)

    def add_tone(start: float, duration: float, frequency: float, amplitude: float, decay: float = 2.0) -> None:
        first = int(start * SAMPLE_RATE)
        count = min(int(duration * SAMPLE_RATE), total - first)
        for index in range(max(0, count)):
            t = index / SAMPLE_RATE
            envelope = math.exp(-decay * t / max(duration, 0.001))
            mix[first + index] += amplitude * envelope * math.sin(2 * math.pi * frequency * t)

    def add_kick(start: float) -> None:
        first, count = int(start * SAMPLE_RATE), int(0.24 * SAMPLE_RATE)
        for index in range(min(count, total - first)):
            t = index / SAMPLE_RATE
            phase = 2 * math.pi * (72 * t - 18 * t * t)
            mix[first + index] += 0.62 * math.exp(-18 * t) * math.sin(phase)

    def add_noise(start: float, duration: float, amplitude: float, decay: float) -> None:
        first, count = int(start * SAMPLE_RATE), int(duration * SAMPLE_RATE)
        previous = 0.0
        for index in range(min(count, total - first)):
            t = index / SAMPLE_RATE
            raw = rng.uniform(-1.0, 1.0)
            high_pass = raw - previous * 0.82
            previous = raw
            mix[first + index] += amplitude * math.exp(-decay * t) * high_pass

    progression = [
        (130.81, 164.81, 196.00),  # C
        (110.00, 130.81, 164.81),  # Am
        (87.31, 130.81, 174.61),   # F
        (98.00, 146.83, 196.00),   # G
    ]
    bass_roots = [65.41, 55.00, 43.65, 49.00]
    melody = [523.25, 659.25, 783.99, 659.25, 587.33, 659.25, 523.25, 493.88]

    for bar in range(BARS):
        bar_start = bar * 4 * BEAT
        chord = progression[bar % len(progression)]
        for frequency in chord:
            add_tone(bar_start, 4 * BEAT, frequency, 0.045, decay=0.45)
            add_tone(bar_start, 4 * BEAT, frequency * 2, 0.018, decay=0.65)
        root = bass_roots[bar % len(bass_roots)]
        for beat_index in range(4):
            beat_start = bar_start + beat_index * BEAT
            add_kick(beat_start)
            add_tone(beat_start, 0.32, root, 0.22, decay=3.5)
            if beat_index in {1, 3}:
                add_noise(beat_start, 0.16, 0.16, 18.0)
            for half in range(2):
                add_noise(beat_start + half * BEAT / 2, 0.045, 0.055, 42.0)
        if bar >= 2:
            for note_index in range(4):
                frequency = melody[(bar * 4 + note_index) % len(melody)]
                add_tone(bar_start + note_index * BEAT, 0.22, frequency, 0.075, decay=4.0)

    peak = max(max(mix), -min(mix), 1e-9)
    fade_samples = int(0.7 * SAMPLE_RATE)
    output = bytearray()
    for index, sample in enumerate(mix):
        gain = 0.90 / peak
        if index < fade_samples:
            gain *= index / fade_samples
        if index >= total - fade_samples:
            gain *= (total - index - 1) / fade_samples
        value = max(-1.0, min(1.0, sample * gain))
        output.extend(struct.pack("<h", int(value * 32767)))

    destination = Path(__file__).resolve().parents[1] / "assets" / "demo_choreography.wav"
    destination.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(destination), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(SAMPLE_RATE)
        wav.writeframes(output)
    print(f"Generated {destination} ({DURATION:.1f}s, {BPM} BPM)")


if __name__ == "__main__":
    main()
