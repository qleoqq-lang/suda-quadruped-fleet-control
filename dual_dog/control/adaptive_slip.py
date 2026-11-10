from __future__ import annotations

from dataclasses import dataclass
import math

from dual_dog.models import SafetyConfig


@dataclass(frozen=True)
class AdaptiveSlipResult:
    correction_x: float
    correction_y: float
    direction: str
    filtered_cross_velocity: float | None
    learned_bias: float
    status: str


class AdaptiveSlipCompensator:
    """Learn a robot's cross-axis velocity bias while it moves straight.

    Each RobotWorker owns one instance, so the learned values are inherently
    per-robot.  Forward, backward, left and right are learned independently;
    stopping or turning freezes the estimates without converting one short
    observation into a permanent calibration value.
    """

    DIRECTIONS = ("forward", "backward", "left", "right")
    LABELS = {
        "forward": "前进",
        "backward": "后退",
        "left": "左移",
        "right": "右移",
    }

    def __init__(self, safety: SafetyConfig) -> None:
        self.safety = safety
        self.biases = {direction: 0.0 for direction in self.DIRECTIONS}
        self.filtered_cross = {direction: None for direction in self.DIRECTIONS}
        self.active_direction: str | None = None
        self.active_since: float | None = None
        self.previous_sample_time: float | None = None
        self.previous_apply_time: float | None = None
        self.previous_sample_token: tuple[object, ...] | None = None
        self.output_bias = 0.0

    def reset(self) -> None:
        for direction in self.DIRECTIONS:
            self.biases[direction] = 0.0
            self.filtered_cross[direction] = None
        self.pause()

    def pause(self) -> None:
        self.active_direction = None
        self.active_since = None
        self.previous_sample_time = None
        self.previous_apply_time = None
        self.previous_sample_token = None
        self.output_bias = 0.0

    @staticmethod
    def _direction(x: float, y: float) -> str | None:
        # Online learning is deliberately limited to the four show-control
        # straight directions.  Diagonal choreography remains unchanged.
        if abs(x) >= 1e-6 and abs(y) <= 0.20 * abs(x):
            return "forward" if x > 0.0 else "backward"
        if abs(y) >= 1e-6 and abs(x) <= 0.20 * abs(y):
            return "left" if y > 0.0 else "right"
        return None

    @staticmethod
    def _approach(current: float, target: float, maximum_change: float) -> float:
        return max(current - maximum_change, min(current + maximum_change, target))

    def apply(
        self,
        x_command: float,
        y_command: float,
        measured_velocity,
        now: float,
        sample_token: tuple[object, ...] | None = None,
        support_legs: int = 0,
    ) -> AdaptiveSlipResult:
        direction = self._direction(float(x_command), float(y_command))
        if direction is None or not math.isfinite(now):
            self.pause()
            return AdaptiveSlipResult(0.0, 0.0, "", None, 0.0, "侧滑自适应[暂停]")

        if direction != self.active_direction:
            self.active_direction = direction
            self.active_since = now
            self.previous_sample_time = now
            self.previous_apply_time = now
            self.previous_sample_token = None
            self.output_bias = 0.0
            # Retain the learned bias, but start every new movement bout from
            # a fresh velocity observation so changed floor/battery conditions
            # can be learned instead of inheriting a stale filtered sample.
            self.filtered_cross[direction] = None

        apply_dt = (
            0.05
            if self.previous_apply_time is None
            else max(0.02, min(0.2, now - self.previous_apply_time))
        )
        self.previous_apply_time = now
        filtered = self.filtered_cross[direction]
        measurement_state = "等待腿部速度"
        learning = False

        valid_measurement = measured_velocity is not None and len(measured_velocity) >= 2
        if valid_measurement:
            measured_x = float(measured_velocity[0])
            measured_y = float(measured_velocity[1])
            valid_measurement = math.isfinite(measured_x) and math.isfinite(measured_y)
        if valid_measurement:
            primary = measured_x if direction in {"forward", "backward"} else measured_y
            cross = measured_y if direction in {"forward", "backward"} else measured_x
            expected_sign = 1.0 if direction in {"forward", "left"} else -1.0
            valid_measurement = (
                primary * expected_sign >= self.safety.adaptive_slip_min_speed_m_s
            )

        new_sample = sample_token is None or sample_token != self.previous_sample_token
        if valid_measurement and new_sample:
            sample_dt = (
                0.05
                if self.previous_sample_time is None
                else max(0.02, min(0.25, now - self.previous_sample_time))
            )
            self.previous_sample_time = now
            self.previous_sample_token = sample_token
            alpha = self.safety.adaptive_slip_filter_alpha
            filtered = (
                cross
                if filtered is None
                else alpha * cross + (1.0 - alpha) * filtered
            )
            self.filtered_cross[direction] = filtered
            warmed_up = (
                self.active_since is not None
                and now - self.active_since >= self.safety.adaptive_slip_warmup_s
            )
            if warmed_up:
                effective_cross = math.copysign(
                    max(
                        0.0,
                        abs(filtered) - self.safety.adaptive_slip_deadband_m_s,
                    ),
                    filtered,
                )
                current_bias = self.biases[direction]
                hard_limit = self.safety.adaptive_slip_hard_limit
                # Smoothly slow learning near the safety boundary instead of
                # hitting the old small rigid correction cap.
                headroom = max(0.10, 1.0 - (abs(current_bias) / hard_limit) ** 2)
                current_bias -= (
                    self.safety.adaptive_slip_learning_rate
                    * effective_cross
                    * sample_dt
                    * headroom
                )
                self.biases[direction] = max(
                    -hard_limit, min(hard_limit, current_bias)
                )
                learning = abs(effective_cross) > 0.0
                measurement_state = "学习" if learning else "横速到位"
            else:
                measurement_state = "观测稳定中"
        elif valid_measurement:
            measurement_state = "等待新状态帧"
        else:
            measurement_state = "速度不足，冻结学习"

        target_bias = self.biases[direction]
        self.output_bias = self._approach(
            self.output_bias,
            target_bias,
            self.safety.adaptive_slip_slew_rate * apply_dt,
        )
        correction_x = (
            self.output_bias if direction in {"left", "right"} else 0.0
        )
        correction_y = (
            self.output_bias if direction in {"forward", "backward"} else 0.0
        )
        label = self.LABELS[direction]
        cross_text = "--" if filtered is None else f"{filtered:+.3f}"
        legs_text = "" if support_legs <= 0 else f" {support_legs}腿"
        status = (
            f"侧滑自适应[{label}/{measurement_state}{legs_text}] "
            f"横速={cross_text}m/s 学习值={target_bias:+.3f}m/s "
            f"输出={self.output_bias:+.3f}m/s"
        )
        return AdaptiveSlipResult(
            correction_x=correction_x,
            correction_y=correction_y,
            direction=direction,
            filtered_cross_velocity=filtered,
            learned_bias=target_bias,
            status=status,
        )
