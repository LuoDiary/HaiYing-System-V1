from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import cast
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen


URDF_JOINT_NAMES: tuple[str, ...] = (
    "J1_Rotation",
    "J2_Shoulder_Pitch",
    "J3_Elbow_Pitch",
    "J4_Wrist_Pitch",
    "J5_Wrist_Roll",
)


@dataclass(frozen=True)
class MoveItTrajectorySnapshot:
    joint_names: tuple[str, ...]
    positions_rad: tuple[tuple[float, ...], ...]
    times_s: tuple[float, ...]

    @property
    def duration_s(self) -> float:
        return self.times_s[-1]

    @property
    def start_positions_rad(self) -> tuple[float, ...]:
        return self.positions_rad[0]

    @property
    def target_positions_rad(self) -> tuple[float, ...]:
        return self.positions_rad[-1]

    @property
    def target_positions_deg(self) -> tuple[float, ...]:
        return tuple(math.degrees(value) for value in self.target_positions_rad)

    def to_payload(self) -> dict[str, object]:
        return {
            "joint_names": list(self.joint_names),
            "points": [
                {
                    "positions_rad": list(positions),
                    "time_from_start_s": time_s,
                }
                for positions, time_s in zip(self.positions_rad, self.times_s, strict=True)
            ],
        }


def _canonical_values(
    names: list[str] | tuple[str, ...],
    values: list[float] | tuple[float, ...],
    label: str,
) -> tuple[float, ...]:
    if len(names) != len(values) or len(set(names)) != len(names):
        raise ValueError(f"{label} 的关节名称和值不匹配")
    missing = [name for name in URDF_JOINT_NAMES if name not in names]
    if missing:
        raise ValueError(f"{label} 缺少关节：{', '.join(missing)}")
    result = tuple(float(values[names.index(name)]) for name in URDF_JOINT_NAMES)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} 包含非法关节角")
    return result


def build_snapshot(
    trajectory_joint_names: list[str] | tuple[str, ...],
    point_positions_rad: list[list[float]] | tuple[tuple[float, ...], ...],
    point_times_s: list[float] | tuple[float, ...],
    start_joint_names: list[str] | tuple[str, ...],
    start_positions_rad: list[float] | tuple[float, ...],
) -> MoveItTrajectorySnapshot:
    if not point_positions_rad or len(point_positions_rad) != len(point_times_s):
        raise ValueError("MoveIt RobotTrajectory 没有有效轨迹点")
    start = _canonical_values(start_joint_names, start_positions_rad, "trajectory_start")
    positions = [
        _canonical_values(trajectory_joint_names, values, f"轨迹点 {index + 1}")
        for index, values in enumerate(point_positions_rad)
    ]
    times = [float(value) for value in point_times_s]
    if not all(math.isfinite(value) and value >= 0.0 for value in times):
        raise ValueError("MoveIt 轨迹时间包含非法值")
    if times[0] <= 1e-6:
        start_error = max(
            abs(planned - actual) for planned, actual in zip(positions[0], start, strict=True)
        )
        if start_error > math.radians(0.5):
            raise ValueError(
                f"MoveIt 首轨迹点与 trajectory_start 相差 {math.degrees(start_error):.3f}°"
            )
        times[0] = 0.0
        positions[0] = start
    else:
        times.insert(0, 0.0)
        positions.insert(0, start)
    if len(times) < 2 or any(
        current <= previous for previous, current in zip(times[:-1], times[1:], strict=True)
    ):
        raise ValueError("MoveIt 轨迹时间必须严格递增")
    return MoveItTrajectorySnapshot(URDF_JOINT_NAMES, tuple(positions), tuple(times))


def simulation_endpoint_error_deg(
    snapshot: MoveItTrajectorySnapshot,
    joint_state_names: list[str] | tuple[str, ...],
    joint_state_positions_rad: list[float] | tuple[float, ...],
) -> float:
    current = _canonical_values(joint_state_names, joint_state_positions_rad, "仿真 joint_states")
    return math.degrees(
        max(
            abs(actual - target)
            for actual, target in zip(current, snapshot.target_positions_rad, strict=True)
        )
    )


class MoveItRealClient:
    def __init__(self, server_url: str, timeout_s: float = 5.0):
        parsed = urlparse(server_url)
        if parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost"}:
            raise ValueError("MoveIt 实机服务只允许本机 HTTP 地址")
        if parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
            raise ValueError("MoveIt 实机服务地址不能包含路径、查询参数或片段")
        self.server_url = server_url.rstrip("/")
        self.timeout_s = timeout_s

    def _request(self, path: str, payload: dict[str, object] | None = None) -> dict[str, object]:
        request = Request(
            f"{self.server_url}{path}",
            data=None if payload is None else json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="GET" if payload is None else "POST",
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                value = json.load(response)
        except HTTPError as error:
            try:
                detail = json.load(error).get("error", str(error))
            except (json.JSONDecodeError, AttributeError):
                detail = str(error)
            raise RuntimeError(str(detail)) from error
        except URLError as error:
            raise RuntimeError(f"无法连接 MoveIt 实机服务：{error.reason}") from error
        if not isinstance(value, dict):
            raise RuntimeError("MoveIt 实机服务返回值格式错误")
        return cast(dict[str, object], value)

    def health(self) -> dict[str, object]:
        return self._request("/api/health")

    def validate(self, snapshot: MoveItTrajectorySnapshot) -> dict[str, object]:
        return self._request("/api/validate", snapshot.to_payload())

    def execute(self, trajectory_id: str) -> dict[str, object]:
        return self._request(
            "/api/execute",
            {"trajectory_id": trajectory_id, "confirm_execute": True},
        )
