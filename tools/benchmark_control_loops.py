"""Deterministic software-only plant comparison; never imports hardware drivers."""
from pathlib import Path
import json
import math
import sys
import types

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dual_dog.control import feedback, speed_sync
from dual_dog.models import SafetyConfig, Telemetry


def baseline(name):
    sources = json.loads((ROOT / 'diagnostics/control_algorithm_baseline.json').read_text(encoding='utf-8'))
    module = types.ModuleType('dual_dog.baseline_' + name)
    module.__package__ = 'dual_dog'
    sys.modules[module.__name__] = module
    exec(sources[name], module.__dict__)
    return module


def velocity(module):
    results = []
    for axis in range(3):
        for sign in (-1, 1):
            traces = []
            gains = ((0.70, 0.85, 1.0, 1.15, 1.25, 0.90),
                     (0.75, 0.90, 1.0, 1.10, 1.20, 0.85),
                     (0.80, 0.90, 1.0, 1.10, 1.20, 0.95))[axis]
            for gain in gains:
                loop = module.VelocitySynchronizer()
                pulse = speed_sync.ActuationPulse()
                actual = measured = command = 0.0
                trace = []
                for tick in range(160):
                    now = 1.0 + tick * 0.05
                    if tick % 3 == 0:
                        measured = actual + 0.003 * math.sin(tick * 0.7)
                    args = dict(limits=(0.5, 0.5), sample_token=tick // 3) if module is speed_sync else {}
                    if axis == 1:
                        _, command, _ = loop.apply(0.0, sign*.30, (0.0, measured), now, **args)
                    elif axis == 2 and module is not speed_sync:
                        command = sign*.30  # Original controller bypassed PI when turning.
                    else:
                        command, _, _ = loop.apply(sign * 0.30, 0.0, (measured, 0.0), now, **args)
                    command = max(-0.5, min(0.5, command))
                    if axis in (1, 2):
                        minimum = .25 if axis == 1 else .22
                        if module is speed_sync:
                            command = pulse.apply(command, minimum)
                        elif axis == 1 and 0 < abs(command) < minimum:
                            command = math.copysign(minimum, command)
                        if abs(command) < minimum - 1e-9:
                            command = 0.
                    actual += (gain * command - actual) * (1 - math.exp(-0.05 / 0.18))
                    trace.append(actual)
                traces.append(trace)
            spread = sum(max(t[i] for t in traces) - min(t[i] for t in traces) for i in range(80, 160)) / 80
            results.append(dict(axis=('x','y','yaw')[axis], sign=sign, steady_spread=round(spread, 5)))
    return results


def heading(module):
    results = []
    for initial in (5, 15, 45, -45):
        loop = module.StraightLineTracker(SafetyConfig())
        angle, rate = math.radians(initial), 0.0
        measured_angle, measured_rate = angle, rate
        delay = [0.0, 0.0]
        errors = []
        for tick in range(240):
            now = 1.0 + tick * 0.05
            if tick % 3 == 0:
                measured_angle, measured_rate = angle, rate
            data = Telemetry('sim', connected=True, rpy=(0.,0.,measured_angle),
                             corrected_yaw_rate=measured_rate, updated_at=now)
            out = loop.apply(0.,0.,0.,data,now=now,target_yaw=0.)
            delay.append(out.yaw)
            command = delay.pop(0)
            if abs(command) < 0.20:
                command = 0.0
            rate += (command - rate) * (1 - math.exp(-0.05 / 0.12))
            angle += rate * 0.05
            errors.append(abs(math.degrees(angle)))
        settled = next((i * .05 for i in range(len(errors)-20)
                        if max(errors[i:]) <= 1.0), None)
        results.append(dict(initial_deg=initial, settle_1deg_s=settled,
                            final_error_deg=round(errors[-1],3),
                            last_2s_peak_deg=round(max(errors[-40:]),3)))
    return results


if __name__ == '__main__':
    report = {'scope': 'Synthetic gain differences, 150ms telemetry, 100ms yaw command delay; not hardware evidence.',
              'speed_before': velocity(baseline('speed_sync')), 'speed_after': velocity(speed_sync),
              'yaw_before': heading(baseline('feedback')), 'yaw_after': heading(feedback)}
    destination = ROOT / 'diagnostics/control_algorithm_comparison.json'
    destination.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
