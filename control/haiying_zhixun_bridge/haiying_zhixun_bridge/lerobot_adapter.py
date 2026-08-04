"""SO-101 MoveIt/URDF 到 LeRobot 的最小实机适配层。

该模块只在 Python 3.12 的 LeRobot 环境中由实机服务导入；ROS 2 节点不会导入
LeRobot，从而避免 ROS 2 Humble Python 3.10 与 LeRobot 环境相互污染。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Protocol, cast

from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS


Mode = Literal["inspect"]
JOINT_NAMES: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
URDF_JOINT_NAMES: tuple[str, ...] = (
    "J1_Rotation",
    "J2_Shoulder_Pitch",
    "J3_Elbow_Pitch",
    "J4_Wrist_Pitch",
    "J5_Wrist_Roll",
)

# 现场装配映射。修改后必须同步 config/arm_bridge.yaml，并重新执行小角度验收。
JOINT_DIRECTIONS: tuple[float, ...] = (1.0, 1.0, -1.0, 1.0, 1.0)
JOINT_OFFSETS_DEG: tuple[float, ...] = (
    -5.406593406593407,
    12.615384615384615,
    0.13186813186813187,
    19.428571428571427,
    17.45054945054945,
)


class RobotLike(Protocol):
    is_connected: bool

    def connect(self, calibrate: bool) -> None: ...

    def disconnect(self) -> None: ...

    def get_observation(self) -> dict[str, object]: ...

    def send_action(self, action: dict[str, float]) -> dict[str, float]: ...


@dataclass
class IKRealConfig:
    mode: Mode
    port: str = "/dev/ttyACM0"
    robot_id: str = "jiebang_follower_arm"
    calibration_dir: Path | None = None


def _calibration_path(config: IKRealConfig) -> Path:
    directory = (
        config.calibration_dir
        if config.calibration_dir is not None
        else HF_LEROBOT_CALIBRATION / ROBOTS / "so101_follower_5dof"
    )
    return directory / f"{config.robot_id}.json"


def validate_calibration_file(calibration_path: Path) -> dict[str, int]:
    if not calibration_path.is_file():
        raise FileNotFoundError(f"找不到五舵机校准文件：{calibration_path}")
    payload = json.loads(calibration_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or set(payload) != set(JOINT_NAMES):
        raise ValueError(f"校准文件必须严格包含五个关节：{', '.join(JOINT_NAMES)}")
    motor_ids: dict[str, int] = {}
    for expected_id, joint_name in enumerate(JOINT_NAMES, start=1):
        entry = payload[joint_name]
        if not isinstance(entry, dict) or entry.get("id") != expected_id:
            raise ValueError(f"校准文件中的 {joint_name} 必须使用舵机 ID {expected_id}")
        motor_ids[joint_name] = expected_id
    return motor_ids


def _make_robot(config: IKRealConfig, max_relative_target_deg: float) -> RobotLike:
    from lerobot.robots.so_follower import SO101Follower5DOF, SO101Follower5DOFConfig

    calibration_path = _calibration_path(config)
    validate_calibration_file(calibration_path)
    robot_config = SO101Follower5DOFConfig(
        port=config.port,
        id=config.robot_id,
        calibration_dir=calibration_path.parent,
        max_relative_target=max_relative_target_deg,
        use_degrees=True,
    )
    return SO101Follower5DOF(robot_config)


def inspect_robot(robot: RobotLike) -> dict[str, float]:
    observation = cast(dict[str, object], robot.get_observation())
    positions: dict[str, float] = {}
    for joint_name in JOINT_NAMES:
        key = f"{joint_name}.pos"
        value = observation.get(key)
        if not isinstance(value, int | float) or not math.isfinite(float(value)):
            raise ValueError(f"实机观测缺少有限角度：{key}")
        positions[joint_name] = float(value)
    return positions


def map_urdf_angles_to_real(urdf_angles_deg: tuple[float, ...]) -> dict[str, float]:
    if len(urdf_angles_deg) != len(JOINT_NAMES) or not all(
        math.isfinite(value) for value in urdf_angles_deg
    ):
        raise ValueError("URDF 角度必须包含五个有限数值")
    mapped_values = tuple(
        direction * angle + offset
        for angle, direction, offset in zip(
            urdf_angles_deg, JOINT_DIRECTIONS, JOINT_OFFSETS_DEG, strict=True
        )
    )
    return dict(zip(JOINT_NAMES, mapped_values, strict=True))
