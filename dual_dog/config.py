from __future__ import annotations

import json
from pathlib import Path

from .models import RobotConfig, SafetyConfig


DEFAULT_APPLICATION_NAME = "苏州大学新生开学典礼\n四足机器人表演集群控制平台"


def load_application_name(path: str | Path) -> str:
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    application = data.get("application", {})
    name = str(application.get("name", DEFAULT_APPLICATION_NAME)).strip()
    return name or DEFAULT_APPLICATION_NAME


def load_config(path: str | Path) -> tuple[str, list[RobotConfig], SafetyConfig]:
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    mode = str(data.get("mode", "sim"))
    if mode not in {"sim", "real"}:
        raise ValueError("mode 必须是 sim 或 real")
    robots = [
        RobotConfig(
            robot_id=str(item["id"]),
            name=str(item.get("name", item["id"])),
            ip=str(item["ip"]),
            mac=str(item.get("mac", "")),
            state_port=int(item["state_port"]),
            session_id=int(item["session_id"]),
        )
        for item in data["robots"]
    ]
    if not 1 <= len(robots) <= 20:
        raise ValueError("机器狗数量必须在 1 到 20 台之间")
    if len({r.robot_id for r in robots}) != len(robots):
        raise ValueError("机器狗 ID 必须唯一")
    if len({r.ip for r in robots}) != len(robots):
        raise ValueError("机器狗 IP 必须唯一")
    if len({r.state_port for r in robots}) != len(robots):
        raise ValueError("本机状态 UDP 端口必须唯一")
    if len({r.session_id for r in robots}) != len(robots):
        raise ValueError("机器狗 session_id 必须唯一")
    safety = SafetyConfig(**data.get("safety", {}))
    if min(safety.max_x, safety.max_y, safety.max_yaw) <= 0:
        raise ValueError("速度限值必须为正数")
    if not 0 < safety.min_lateral_actuation <= safety.max_y:
        raise ValueError("纯横移最小起步速度必须大于 0 且不超过横向速度硬限幅")
    if min(safety.yaw_kp, safety.yaw_ki, safety.yaw_kd) < 0:
        raise ValueError("反馈增益不能为负数")
    if safety.yaw_integral_limit < 0:
        raise ValueError("PID 积分限幅不能为负数")
    if not 0 <= safety.yaw_derivative_alpha <= 1:
        raise ValueError("微分滤波系数必须在 0 到 1 之间")
    if not 0 < safety.max_yaw_correction <= safety.max_yaw:
        raise ValueError("反馈 yaw 限幅必须大于 0 且不超过总 yaw 限幅")
    if not 0 < safety.min_yaw_actuation <= safety.max_yaw_correction:
        raise ValueError("最小有效 yaw 速度必须大于 0 且不超过反馈 yaw 限幅")
    if not 0 < safety.feedback_deadband_rad <= 0.1:
        raise ValueError("yaw 到位容差必须在 0 到 0.1 rad 之间")
    if min(safety.lateral_kp, safety.lateral_ki, safety.lateral_kd) < 0:
        raise ValueError("轨迹纠偏增益不能为负数")
    if safety.lateral_integral_limit < 0:
        raise ValueError("横向纠偏积分限幅不能为负数")
    if not 0 <= safety.lateral_derivative_alpha <= 1:
        raise ValueError("横向纠偏微分滤波系数必须在 0 到 1 之间")
    if not 0 <= safety.lateral_deadband_m <= 0.1:
        raise ValueError("横向纠偏死区必须在 0 到 0.1 m 之间")
    if not 0 < safety.max_lateral_correction <= min(safety.max_x, safety.max_y):
        raise ValueError("轨迹补偿限幅必须大于 0 且不超过平移速度硬限幅")
    if min(
        safety.max_lateral_error_m,
        safety.lateral_uncertainty_limit_m,
        safety.lateral_slew_rate,
    ) <= 0:
        raise ValueError("轨迹误差、置信度和变化率限值必须为正数")
    if safety.adaptive_slip_learning_rate <= 0:
        raise ValueError("在线侧滑学习率必须为正数")
    if not 0 < safety.adaptive_slip_filter_alpha <= 1:
        raise ValueError("在线侧滑滤波系数必须在 0 到 1 之间")
    if min(
        safety.adaptive_slip_deadband_m_s,
        safety.adaptive_slip_min_speed_m_s,
        safety.adaptive_slip_warmup_s,
        safety.adaptive_slip_hard_limit,
        safety.adaptive_slip_slew_rate,
    ) <= 0:
        raise ValueError("在线侧滑死区、起步速度、观察时间、限幅和变化率必须为正数")
    if safety.adaptive_slip_hard_limit > min(safety.max_x, safety.max_y):
        raise ValueError("在线侧滑安全限幅不能超过平移速度硬限幅")
    if not 500 <= safety.one_shot_return_ms <= 10000:
        raise ValueError("特技安全锁定时间必须在 500 到 10000 ms 之间")
    if not 50 <= safety.one_shot_trigger_ms <= 500:
        raise ValueError("特技触发脉冲必须在 50 到 500 ms 之间")
    if safety.one_shot_trigger_ms >= safety.one_shot_return_ms:
        raise ValueError("特技触发脉冲必须短于动作安全锁定时间")
    if not 0.5 <= safety.non_flip_action_seconds <= 10.0:
        raise ValueError("非空翻动作持续时间必须在 0.5 到 10 秒之间")
    return mode, robots, safety
