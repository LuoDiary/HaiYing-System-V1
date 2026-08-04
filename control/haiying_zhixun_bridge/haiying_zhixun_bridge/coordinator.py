from __future__ import annotations

import math
import threading
from collections.abc import Sequence
from typing import Protocol

from .contracts import JOINT_COUNT, MissionState, PlanSummary, TargetPose


class Planner(Protocol):
    def plan_target(self, target: TargetPose, initial_joint_angles_deg: Sequence[float]) -> PlanSummary: ...


class StateGateError(RuntimeError):
    """当前任务状态不允许机械臂规划。"""


class StalePlanError(StateGateError):
    """规划期间任务状态已变化，结果不得继续使用。"""


class ArmCoordinator:
    def __init__(self, planner: Planner, initial_joint_angles_deg: tuple[float, ...]) -> None:
        if len(initial_joint_angles_deg) != JOINT_COUNT or not all(
            math.isfinite(value) for value in initial_joint_angles_deg
        ):
            raise ValueError(
                f"initial_joint_angles_deg 必须包含 {JOINT_COUNT} 个有限数值"
            )
        self._planner = planner
        self._initial_joint_angles_deg = initial_joint_angles_deg
        self._state = MissionState.SEARCHING
        self._state_revision = 0
        self._lock = threading.Lock()

    @property
    def state(self) -> MissionState:
        with self._lock:
            return self._state

    def update_state(self, value: str) -> MissionState:
        state = MissionState.parse(value)
        with self._lock:
            if state is not self._state:
                self._state = state
                self._state_revision += 1
            return self._state

    def plan_target(self, target: TargetPose) -> PlanSummary:
        with self._lock:
            if self._state is not MissionState.BRUSHING:
                raise StateGateError(
                    f"当前状态 {self._state.value} 不允许机械臂规划，仅 BRUSHING 状态允许"
                )
            revision = self._state_revision
        summary = self._planner.plan_target(target, self._initial_joint_angles_deg)
        with self._lock:
            if self._state is not MissionState.BRUSHING or self._state_revision != revision:
                raise StalePlanError("IK 规划期间系统状态已变化，规划结果已作废")
        return summary
