from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from typing import cast
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .config import IkConfig
from .contracts import JOINT_COUNT, PlanSummary, TargetPose


def _finite_vector(value: object, name: str, length: int) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"IK 响应字段 {name} 必须包含 {length} 个数值")
    if not all(
        not isinstance(item, bool)
        and isinstance(item, int | float)
        and math.isfinite(float(item))
        for item in value
    ):
        raise ValueError(f"IK 响应字段 {name} 包含非法数值")
    return tuple(float(item) for item in value)


def parse_plan_payload(
    payload: Mapping[str, object], expected_frames: int, maximum_position_error_m: float
) -> PlanSummary:
    if payload.get("success") is not True:
        raise ValueError(f"IK 规划失败：{payload.get('message', '未知错误')}")
    if payload.get("collision_free") is not True:
        raise ValueError("IK 规划未通过自碰撞检查")
    plan_id = payload.get("plan_id")
    if not isinstance(plan_id, str) or not plan_id.isalnum():
        raise ValueError("IK 响应缺少合法 plan_id")
    target = _finite_vector(payload.get("target_position_m"), "target_position_m", 3)
    reached = _finite_vector(payload.get("reached_position_m"), "reached_position_m", 3)
    joints = _finite_vector(payload.get("joint_angles_deg"), "joint_angles_deg", JOINT_COUNT)
    error_value = payload.get("error_m")
    if (
        isinstance(error_value, bool)
        or not isinstance(error_value, int | float)
        or not math.isfinite(float(error_value))
    ):
        raise ValueError("IK 响应缺少有限 error_m")
    error_m = float(error_value)
    if error_m < 0.0:
        raise ValueError("IK 位置误差不能为负数")
    if error_m > maximum_position_error_m:
        raise ValueError(
            f"IK 位置误差 {error_m * 1000:.3f} mm 超过限制 "
            f"{maximum_position_error_m * 1000:.3f} mm"
        )
    trajectory = payload.get("trajectory_deg")
    if not isinstance(trajectory, list) or len(trajectory) != expected_frames:
        raise ValueError(f"IK 轨迹必须包含 {expected_frames} 帧")
    for frame_index, frame in enumerate(trajectory, start=1):
        _finite_vector(frame, f"trajectory_deg[{frame_index}]", JOINT_COUNT)
    return PlanSummary(
        plan_id=plan_id,
        target_position_m=cast(tuple[float, float, float], target),
        reached_position_m=cast(tuple[float, float, float], reached),
        error_m=error_m,
        joint_angles_deg=joints,
        trajectory_frames=len(trajectory),
        collision_free=True,
    )


class IkClient:
    def __init__(self, config: IkConfig) -> None:
        self._config = config

    def _request_json(self, path: str, method: str, payload: Mapping[str, object] | None) -> dict[str, object]:
        url = f"{self._config.server_url}{path}"
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {} if data is None else {"Content-Type": "application/json"}
        request = Request(url, data=data, headers=headers, method=method)
        try:
            with urlopen(request, timeout=30) as response:
                value = json.load(response)
        except (HTTPError, URLError, TimeoutError) as error:
            raise RuntimeError(f"无法访问本机 IK 服务 {url}：{error}") from error
        if not isinstance(value, dict):
            raise ValueError("IK 服务响应必须是 JSON 对象")
        return cast(dict[str, object], value)

    def health(self) -> dict[str, object]:
        payload = self._request_json("/api/health", "GET", None)
        if payload.get("status") != "ok":
            raise RuntimeError(f"IK 服务健康检查失败：{payload}")
        return payload

    def plan_target(self, target: TargetPose, initial_joint_angles_deg: Sequence[float]) -> PlanSummary:
        initial_values = tuple(initial_joint_angles_deg)
        if len(initial_values) != JOINT_COUNT or not all(
            not isinstance(value, bool)
            and isinstance(value, int | float)
            and math.isfinite(float(value))
            for value in initial_values
        ):
            raise ValueError(f"initial_joint_angles_deg 必须包含 {JOINT_COUNT} 个有限数值")
        initial = tuple(float(value) for value in initial_values)
        payload = self._request_json(
            "/api/plan",
            "POST",
            {
                "target_position_m": list(target.position_m),
                "initial_joint_angles_deg": list(initial),
            },
        )
        summary = parse_plan_payload(
            payload,
            self._config.expected_trajectory_frames,
            self._config.maximum_position_error_m,
        )
        if any(
            not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9)
            for actual, expected in zip(summary.target_position_m, target.position_m)
        ):
            raise ValueError("IK 响应中的 target_position_m 与请求目标不一致")
        return summary

    def get_plan(self, plan_id: str) -> PlanSummary:
        if not plan_id.isalnum():
            raise ValueError("plan_id 必须只包含字母和数字")
        payload = self._request_json(f"/api/plans/{plan_id}", "GET", None)
        return parse_plan_payload(
            payload,
            self._config.expected_trajectory_frames,
            self._config.maximum_position_error_m,
        )
