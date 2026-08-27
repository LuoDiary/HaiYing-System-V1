from __future__ import annotations

import math
import subprocess
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Literal

from .config import RobotPrototypeConfig, SafetyConfig


RealArmAction = Literal["inspect", "jog", "dry-run", "execute"]
JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)


@dataclass(frozen=True)
class RealArmRequest:
    action: RealArmAction
    joint: str | None
    delta_deg: float | None
    plan_id: str | None
    confirm_execute: bool


def build_real_arm_command(
    executable: str,
    robot: RobotPrototypeConfig,
    safety: SafetyConfig,
    request: RealArmRequest,
) -> tuple[str, ...]:
    if not executable:
        raise ValueError("实机控制命令不能为空")
    if request.action not in {"inspect", "jog", "dry-run", "execute"}:
        raise ValueError(f"未知实机控制动作：{request.action!r}")
    command = [
        executable,
        f"--mode={request.action}",
        f"--port={robot.port}",
        f"--robot_id={robot.calibration_id}",
    ]
    if request.action == "jog":
        if request.joint not in robot.joint_names:
            raise ValueError(f"jog joint 必须是：{', '.join(robot.joint_names)}")
        if (
            request.delta_deg is None
            or not math.isfinite(request.delta_deg)
            or request.delta_deg == 0.0
            or abs(request.delta_deg) > safety.jog_max_delta_deg
        ):
            raise ValueError(
                f"jog delta_deg 必须非零且不超过 {safety.jog_max_delta_deg:g}°"
            )
        command.extend((f"--joint={request.joint}", f"--delta_deg={request.delta_deg}"))
    elif request.joint is not None or request.delta_deg is not None:
        raise ValueError("joint 和 delta_deg 只允许用于 jog")

    if request.action in {"dry-run", "execute"}:
        if request.plan_id is None or not request.plan_id.isalnum():
            raise ValueError(f"{request.action} 必须提供合法 plan_id")
        command.append(f"--plan_id={request.plan_id}")
    elif request.plan_id is not None:
        raise ValueError("plan_id 只允许用于 dry-run 或 execute")

    if request.action == "execute":
        if not request.confirm_execute:
            raise ValueError("execute 必须显式确认")
        command.append("--confirm_execute=true")
    elif request.confirm_execute:
        raise ValueError("confirm_execute 只允许用于 execute")
    return tuple(command)


def run_real_arm_command(command: Sequence[str]) -> int:
    if not command:
        raise ValueError("实机控制命令不能为空")
    completed = subprocess.run(list(command), check=False, shell=False)
    return completed.returncode
