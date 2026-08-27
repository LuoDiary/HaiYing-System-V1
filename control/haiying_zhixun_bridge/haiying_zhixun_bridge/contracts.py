from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

try:
    from enum import StrEnum
except ImportError:  # Python 3.10 used by ROS 2 Humble
    class StrEnum(str, Enum):
        """Small Python 3.10-compatible subset of enum.StrEnum."""


TARGET_POSE_TOPIC = "/arm/target_pose"
CURRENT_STATE_TOPIC = "/system/current_state"
ACCEPTED_FRAME_ID = "base_footprint"
JOINT_COUNT = 5


class MissionState(StrEnum):
    SEARCHING = "SEARCHING"
    TARGET_FOUND = "TARGET_FOUND"
    APPROACHING = "APPROACHING"
    BRUSHING = "BRUSHING"
    RETURNING = "RETURNING"

    @classmethod
    def parse(cls, value: str) -> MissionState:
        normalized = value.strip().upper()
        try:
            return cls(normalized)
        except ValueError as error:
            allowed = ", ".join(state.value for state in cls)
            raise ValueError(f"未知系统状态 {value!r}，允许值：{allowed}") from error


@dataclass(frozen=True)
class TargetPose:
    frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    def __post_init__(self) -> None:
        if self.frame_id != ACCEPTED_FRAME_ID:
            raise ValueError(f"机械臂目标坐标系必须是 {ACCEPTED_FRAME_ID}，实际为 {self.frame_id!r}")
        values = (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("机械臂目标位置和四元数必须是有限数值")
        quaternion_norm = math.sqrt(self.qx**2 + self.qy**2 + self.qz**2 + self.qw**2)
        if quaternion_norm <= 1e-9:
            raise ValueError("机械臂目标四元数不能为零")
        if not math.isclose(quaternion_norm, 1.0, rel_tol=0.0, abs_tol=1e-3):
            raise ValueError(f"机械臂目标四元数必须归一化，当前模长为 {quaternion_norm:.6f}")

    @property
    def position_m(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class PlanSummary:
    plan_id: str
    target_position_m: tuple[float, float, float]
    reached_position_m: tuple[float, float, float]
    error_m: float
    joint_angles_deg: tuple[float, ...]
    trajectory_frames: int
    collision_free: bool

    def __post_init__(self) -> None:
        if not self.plan_id or not self.plan_id.isalnum():
            raise ValueError("plan_id 必须只包含字母和数字")
        if len(self.target_position_m) != 3 or len(self.reached_position_m) != 3:
            raise ValueError("规划目标位置和到达位置必须各包含 3 个数值")
        if len(self.joint_angles_deg) != JOINT_COUNT:
            raise ValueError(f"规划结果必须包含 {JOINT_COUNT} 个关节角")
        numeric_values = (
            *self.target_position_m,
            *self.reached_position_m,
            self.error_m,
            *self.joint_angles_deg,
        )
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("规划摘要包含非有限数值")
        if self.error_m < 0.0:
            raise ValueError("规划位置误差不能为负数")
        if self.trajectory_frames <= 0:
            raise ValueError("规划轨迹帧数必须为正数")
