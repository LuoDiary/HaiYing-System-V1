import hashlib
import json
import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Protocol, cast
from urllib.parse import urlparse

import draccus

from .lerobot_adapter import (
    JOINT_DIRECTIONS,
    JOINT_NAMES,
    JOINT_OFFSETS_DEG,
    URDF_JOINT_NAMES,
    IKRealConfig,
    RobotLike,
    _make_robot,
    inspect_robot,
    map_urdf_angles_to_real,
    validate_calibration_file,
)
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
from lerobot.utils.utils import init_logging


MAX_REQUEST_BYTES = 2 * 1024 * 1024
MAX_STORED_TRAJECTORIES = 10
MAX_TRAJECTORY_AGE_S = 15 * 60.0


@dataclass
class MoveItRealServerConfig:
    host: str = "127.0.0.1"
    port: int = 8767
    robot_port: str = "/dev/ttyACM0"
    robot_id: str = "jiebang_follower_arm"
    calibration_dir: Path | None = None
    hardware_execution_enabled: bool = False
    execution_rate_hz: float = 10.0
    maximum_duration_s: float = 100.0
    maximum_joint_speed_deg_s: float = 30.0
    maximum_frame_step_deg: float = 5.0
    maximum_command_clip_deg: float = 0.5
    maximum_start_error_deg: float = 10.0
    maximum_feedback_error_deg: float = 3.0
    joint_limit_deg: float = 89.9

    def __post_init__(self) -> None:
        if self.host not in {"127.0.0.1", "localhost"}:
            raise ValueError("host 只允许 127.0.0.1 或 localhost")
        if not 0 <= self.port <= 65535:
            raise ValueError("port 必须位于 0 到 65535；0 仅用于自动分配测试端口")
        positive = (
            self.execution_rate_hz,
            self.maximum_duration_s,
            self.maximum_joint_speed_deg_s,
            self.maximum_frame_step_deg,
            self.maximum_command_clip_deg,
            self.maximum_start_error_deg,
            self.maximum_feedback_error_deg,
            self.joint_limit_deg,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("所有实机轨迹安全参数必须是正的有限数值")
        if self.maximum_frame_step_deg > 5.0:
            raise ValueError("maximum_frame_step_deg 不得超过 5°")
        if self.maximum_command_clip_deg > 0.5:
            raise ValueError("maximum_command_clip_deg 不得超过 0.5°")
        if self.maximum_start_error_deg > 10.0:
            raise ValueError("maximum_start_error_deg 不得超过 10°")
        if self.maximum_feedback_error_deg > 5.0:
            raise ValueError("maximum_feedback_error_deg 不得超过 5°")
        if self.joint_limit_deg > 90.0:
            raise ValueError("joint_limit_deg 不得超过 URDF 的 90° 限位")


@dataclass(frozen=True)
class ValidatedMoveItTrajectory:
    trajectory_id: str
    joint_names: tuple[str, ...]
    source_points: int
    duration_s: float
    resampled_times_s: tuple[float, ...]
    resampled_positions_deg: tuple[tuple[float, ...], ...]
    start_positions_deg: tuple[float, ...]
    target_positions_deg: tuple[float, ...]
    maximum_speed_deg_s: float
    maximum_frame_step_deg: float
    created_monotonic_s: float


@dataclass(frozen=True)
class MoveItExecutionResult:
    trajectory_id: str
    commanded_frames: int
    duration_s: float
    final_sent_deg: dict[str, float]
    final_feedback_deg: dict[str, float]
    maximum_feedback_error_deg: float
    execution_completed: bool


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"{label} 必须是有限数值")
    return float(value)


def _canonical_points(payload: dict[str, object]) -> tuple[list[float], list[tuple[float, ...]]]:
    names_value = payload.get("joint_names")
    if not isinstance(names_value, list) or len(names_value) != len(URDF_JOINT_NAMES):
        raise ValueError("MoveIt 轨迹 joint_names 必须包含五个关节")
    if not all(isinstance(name, str) for name in names_value):
        raise ValueError("MoveIt 轨迹 joint_names 包含非法名称")
    names = tuple(cast(list[str], names_value))
    if len(set(names)) != len(names) or set(names) != set(URDF_JOINT_NAMES):
        raise ValueError(f"MoveIt 轨迹关节必须严格为：{', '.join(URDF_JOINT_NAMES)}")
    reorder = tuple(names.index(name) for name in URDF_JOINT_NAMES)

    points_value = payload.get("points")
    if not isinstance(points_value, list) or not 2 <= len(points_value) <= 2000:
        raise ValueError("MoveIt 轨迹点数必须位于 2 到 2000")
    times: list[float] = []
    positions: list[tuple[float, ...]] = []
    for index, point_value in enumerate(points_value):
        if not isinstance(point_value, dict):
            raise ValueError(f"MoveIt 第 {index + 1} 个轨迹点格式错误")
        time_s = _finite_number(point_value.get("time_from_start_s"), f"第 {index + 1} 点时间")
        position_value = point_value.get("positions_rad")
        if not isinstance(position_value, list) or len(position_value) != len(URDF_JOINT_NAMES):
            raise ValueError(f"MoveIt 第 {index + 1} 点必须包含五个关节角")
        radians = tuple(
            _finite_number(position_value[source_index], f"第 {index + 1} 点关节角")
            for source_index in reorder
        )
        times.append(time_s)
        positions.append(tuple(math.degrees(value) for value in radians))
    return times, positions


def _resample(
    times: list[float],
    positions: list[tuple[float, ...]],
    rate_hz: float,
) -> tuple[tuple[float, ...], tuple[tuple[float, ...], ...]]:
    duration = times[-1]
    sample_count = max(2, math.ceil(duration * rate_hz) + 1)
    sample_times = [min(index / rate_hz, duration) for index in range(sample_count)]
    if sample_times[-1] < duration:
        sample_times.append(duration)
    else:
        sample_times[-1] = duration

    result: list[tuple[float, ...]] = []
    segment = 0
    for sample_time in sample_times:
        while segment + 1 < len(times) - 1 and sample_time > times[segment + 1]:
            segment += 1
        start_time, end_time = times[segment], times[segment + 1]
        ratio = 0.0 if end_time == start_time else (sample_time - start_time) / (end_time - start_time)
        ratio = max(0.0, min(1.0, ratio))
        result.append(
            tuple(
                start + ratio * (end - start)
                for start, end in zip(positions[segment], positions[segment + 1], strict=True)
            )
        )
    return tuple(sample_times), tuple(result)


def validate_moveit_trajectory(
    payload: dict[str, object],
    config: MoveItRealServerConfig,
) -> ValidatedMoveItTrajectory:
    times, positions = _canonical_points(payload)
    if abs(times[0]) > 1e-6:
        raise ValueError("MoveIt 轨迹必须包含 time_from_start=0 的真实起始状态")
    if any(current <= previous for previous, current in zip(times[:-1], times[1:], strict=True)):
        raise ValueError("MoveIt 轨迹时间必须严格递增")
    duration = times[-1]
    if not 0.05 <= duration <= config.maximum_duration_s:
        raise ValueError(
            f"MoveIt 轨迹时长必须位于 0.05 到 {config.maximum_duration_s:g} 秒"
        )
    maximum_angle = max(abs(angle) for frame in positions for angle in frame)
    if maximum_angle > config.joint_limit_deg:
        raise ValueError(
            f"MoveIt 轨迹关节角 {maximum_angle:.3f}° 超过 ±{config.joint_limit_deg:g}° 限位"
        )

    maximum_speed = 0.0
    for start_time, end_time, start, end in zip(
        times[:-1], times[1:], positions[:-1], positions[1:], strict=True
    ):
        delta_time = end_time - start_time
        maximum_speed = max(
            maximum_speed,
            max(abs(target - source) / delta_time for source, target in zip(start, end, strict=True)),
        )
    if maximum_speed > config.maximum_joint_speed_deg_s:
        raise ValueError(
            f"MoveIt 轨迹最大关节速度 {maximum_speed:.3f}°/s 超过 "
            f"{config.maximum_joint_speed_deg_s:g}°/s"
        )

    sample_times, resampled = _resample(times, positions, config.execution_rate_hz)
    maximum_step = max(
        max(abs(target - source) for source, target in zip(start, end, strict=True))
        for start, end in zip(resampled[:-1], resampled[1:], strict=True)
    )
    if maximum_step > config.maximum_frame_step_deg:
        raise ValueError(
            f"重采样后最大单帧变化 {maximum_step:.3f}° 超过 "
            f"{config.maximum_frame_step_deg:g}°"
        )

    canonical = {
        "joint_names": list(URDF_JOINT_NAMES),
        "times_s": [round(value, 9) for value in times],
        "positions_deg": [[round(value, 9) for value in frame] for frame in positions],
    }
    trajectory_id = hashlib.sha256(
        json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return ValidatedMoveItTrajectory(
        trajectory_id=trajectory_id,
        joint_names=URDF_JOINT_NAMES,
        source_points=len(positions),
        duration_s=duration,
        resampled_times_s=sample_times,
        resampled_positions_deg=resampled,
        start_positions_deg=resampled[0],
        target_positions_deg=resampled[-1],
        maximum_speed_deg_s=maximum_speed,
        maximum_frame_step_deg=maximum_step,
        created_monotonic_s=time.monotonic(),
    )


def trajectory_summary(plan: ValidatedMoveItTrajectory) -> dict[str, object]:
    return {
        "trajectory_id": plan.trajectory_id,
        "joint_names": list(plan.joint_names),
        "source_points": plan.source_points,
        "resampled_frames": len(plan.resampled_positions_deg),
        "duration_s": plan.duration_s,
        "start_positions_deg": list(plan.start_positions_deg),
        "target_positions_deg": list(plan.target_positions_deg),
        "maximum_speed_deg_s": plan.maximum_speed_deg_s,
        "maximum_frame_step_deg": plan.maximum_frame_step_deg,
        "validated": True,
        "hardware_connected": False,
    }


def execute_moveit_trajectory(
    robot: RobotLike,
    plan: ValidatedMoveItTrajectory,
    config: MoveItRealServerConfig,
) -> MoveItExecutionResult:
    real_frames = tuple(map_urdf_angles_to_real(frame) for frame in plan.resampled_positions_deg)
    initial_feedback = inspect_robot(robot)
    initial_target = real_frames[0]
    initial_errors = {
        name: abs(initial_feedback[name] - initial_target[name]) for name in JOINT_NAMES
    }
    worst_initial_joint = max(initial_errors, key=initial_errors.get)
    if initial_errors[worst_initial_joint] > config.maximum_start_error_deg:
        raise RuntimeError(
            f"MoveIt 实机首帧不匹配：{worst_initial_joint} 相差 "
            f"{initial_errors[worst_initial_joint]:.3f}°，未发送动作"
        )

    period_s = 1.0 / config.execution_rate_hz
    alignment_step_deg = min(
        config.maximum_frame_step_deg,
        config.maximum_joint_speed_deg_s / config.execution_rate_hz,
    )
    alignment_frames = math.ceil(initial_errors[worst_initial_joint] / alignment_step_deg)
    command_targets: list[dict[str, float]] = []
    for step in range(1, alignment_frames + 1):
        ratio = step / alignment_frames
        command_targets.append(
            {
                name: initial_feedback[name]
                + (initial_target[name] - initial_feedback[name]) * ratio
                for name in JOINT_NAMES
            }
        )
    command_targets.extend(real_frames[1:])

    maximum_feedback_error = 0.0
    final_sent = dict(initial_feedback)
    final_feedback = dict(initial_feedback)
    commanded_frames = 0
    for frame_index, target in enumerate(command_targets, start=1):
        frame_start = time.perf_counter()
        action = {f"{name}.pos": value for name, value in target.items()}
        sent_value = robot.send_action(action)
        final_sent = {
            name: float(cast(dict[str, object], sent_value)[f"{name}.pos"])
            for name in JOINT_NAMES
        }
        command_errors = {name: abs(final_sent[name] - target[name]) for name in JOINT_NAMES}
        clipped_joint = max(command_errors, key=command_errors.get)
        if command_errors[clipped_joint] > config.maximum_command_clip_deg:
            raise RuntimeError(
                f"MoveIt 实机第 {frame_index}/{len(command_targets)} 帧相对目标被安全裁剪："
                f"{clipped_joint} 裁剪量 {command_errors[clipped_joint]:.3f}°，"
                f"允许 ≤ {config.maximum_command_clip_deg:.3f}°（不是机械关节限位）"
            )
        remaining_s = period_s - (time.perf_counter() - frame_start)
        if remaining_s > 0.0:
            time.sleep(remaining_s)
        final_feedback = inspect_robot(robot)
        feedback_errors = {
            name: abs(final_feedback[name] - final_sent[name]) for name in JOINT_NAMES
        }
        worst_joint = max(feedback_errors, key=feedback_errors.get)
        maximum_feedback_error = max(maximum_feedback_error, feedback_errors[worst_joint])
        commanded_frames += 1
        if feedback_errors[worst_joint] > config.maximum_feedback_error_deg:
            raise RuntimeError(
                f"MoveIt 实机第 {frame_index}/{len(command_targets)} 帧反馈误差过大："
                f"{worst_joint} {feedback_errors[worst_joint]:.3f}°"
            )
    return MoveItExecutionResult(
        trajectory_id=plan.trajectory_id,
        commanded_frames=commanded_frames,
        duration_s=plan.duration_s + alignment_frames * period_s,
        final_sent_deg=final_sent,
        final_feedback_deg=final_feedback,
        maximum_feedback_error_deg=maximum_feedback_error,
        execution_completed=True,
    )


def _calibration_path(config: MoveItRealServerConfig) -> Path:
    directory = (
        config.calibration_dir
        if config.calibration_dir is not None
        else HF_LEROBOT_CALIBRATION / ROBOTS / "so101_follower_5dof"
    )
    return directory / f"{config.robot_id}.json"


class MoveItRealServer(ThreadingHTTPServer):
    config: MoveItRealServerConfig

    def initialize(self, config: MoveItRealServerConfig) -> None:
        self.config = config
        self._plans: dict[str, ValidatedMoveItTrajectory] = {}
        self._order: deque[str] = deque()
        self._lock = threading.Lock()
        self._busy = False

    def health(self) -> dict[str, object]:
        calibration_path = _calibration_path(self.config)
        calibration_valid = False
        calibration_error: str | None = None
        try:
            validate_calibration_file(calibration_path)
            calibration_valid = True
        except (FileNotFoundError, ValueError, json.JSONDecodeError) as error:
            calibration_error = str(error)
        with self._lock:
            busy = self._busy
            stored = len(self._plans)
        return {
            "status": "ok",
            "hardware_connected": False,
            "hardware_execution_enabled": self.config.hardware_execution_enabled,
            "busy": busy,
            "stored_trajectories": stored,
            "robot_port": self.config.robot_port,
            "robot_id": self.config.robot_id,
            "calibration_path": str(calibration_path),
            "calibration_valid": calibration_valid,
            "calibration_error": calibration_error,
            "urdf_joint_names": list(URDF_JOINT_NAMES),
            "real_joint_names": list(JOINT_NAMES),
            "direction_signs": list(JOINT_DIRECTIONS),
            "zero_offsets_deg": list(JOINT_OFFSETS_DEG),
            "execution_rate_hz": self.config.execution_rate_hz,
            "maximum_command_clip_deg": self.config.maximum_command_clip_deg,
        }

    def store(self, plan: ValidatedMoveItTrajectory) -> None:
        with self._lock:
            self._plans[plan.trajectory_id] = plan
            if plan.trajectory_id in self._order:
                self._order.remove(plan.trajectory_id)
            self._order.append(plan.trajectory_id)
            while len(self._order) > MAX_STORED_TRAJECTORIES:
                self._plans.pop(self._order.popleft(), None)

    def begin_execution(self, trajectory_id: str) -> ValidatedMoveItTrajectory:
        with self._lock:
            if self._busy:
                raise RuntimeError("已有实机轨迹正在执行")
            plan = self._plans.get(trajectory_id)
            if plan is None:
                raise ValueError("轨迹不存在或服务已重启，请重新验证")
            if time.monotonic() - plan.created_monotonic_s > MAX_TRAJECTORY_AGE_S:
                self._plans.pop(trajectory_id, None)
                raise ValueError("轨迹验证已超过 15 分钟，请重新验证")
            self._busy = True
            return plan

    def finish_execution(self, trajectory_id: str, completed: bool) -> None:
        with self._lock:
            self._busy = False
            if completed:
                self._plans.pop(trajectory_id, None)
                if trajectory_id in self._order:
                    self._order.remove(trajectory_id)


class MoveItRealHandler(BaseHTTPRequestHandler):
    server_version = "HaiyingMoveItReal/1.0"

    def _send(self, payload: dict[str, object], status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_payload(self) -> dict[str, object]:
        content_length = int(self.headers.get("Content-Length", "0"))
        if not 0 < content_length <= MAX_REQUEST_BYTES:
            raise ValueError("请求大小非法")
        value = json.loads(self.rfile.read(content_length))
        if not isinstance(value, dict):
            raise ValueError("请求必须是 JSON 对象")
        return cast(dict[str, object], value)

    def do_GET(self) -> None:
        if urlparse(self.path).path == "/api/health":
            self._send(self.server.health())
        else:
            self._send({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            payload = self._read_payload()
            if path == "/api/validate":
                plan = validate_moveit_trajectory(payload, self.server.config)
                self.server.store(plan)
                self._send(trajectory_summary(plan))
                return
            if path != "/api/execute":
                self._send({"error": "not found"}, HTTPStatus.NOT_FOUND)
                return
            trajectory_id = payload.get("trajectory_id")
            if not isinstance(trajectory_id, str) or len(trajectory_id) != 64:
                raise ValueError("execute 必须提供 64 位 trajectory_id")
            if payload.get("confirm_execute") is not True:
                raise ValueError("execute 必须显式提供 confirm_execute=true")
            if not self.server.config.hardware_execution_enabled:
                raise ValueError("服务未启用实机执行；启动时必须显式设置 hardware_execution_enabled=true")

            plan = self.server.begin_execution(trajectory_id)
            completed = False
            try:
                calibration_path = _calibration_path(self.server.config)
                validate_calibration_file(calibration_path)
                robot_cfg = IKRealConfig(
                    mode="inspect",
                    port=self.server.config.robot_port,
                    robot_id=self.server.config.robot_id,
                    calibration_dir=calibration_path.parent,
                )
                robot = _make_robot(robot_cfg, self.server.config.maximum_frame_step_deg)
                try:
                    robot.connect(calibrate=False)
                    result = execute_moveit_trajectory(robot, plan, self.server.config)
                    completed = result.execution_completed
                finally:
                    if robot.is_connected:
                        robot.disconnect()
                self._send(cast(dict[str, object], asdict(result)))
            finally:
                self.server.finish_execution(trajectory_id, completed)
        except (ValueError, FileNotFoundError, json.JSONDecodeError) as error:
            self._send({"error": str(error)}, HTTPStatus.BAD_REQUEST)
        except RuntimeError as error:
            self._send({"error": str(error)}, HTTPStatus.CONFLICT)
        except Exception as error:
            self._send({"error": f"实机执行失败：{error}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def log_message(self, format_string: str, *args: object) -> None:
        return


def create_server(config: MoveItRealServerConfig) -> MoveItRealServer:
    server = MoveItRealServer((config.host, config.port), MoveItRealHandler)
    server.initialize(config)
    return server


@draccus.wrap()
def run_server(config: MoveItRealServerConfig) -> None:
    init_logging()
    server = create_server(config)
    print(f"Haiying MoveIt real-arm service: http://{config.host}:{config.port}")
    print("Validation is hardware-free. Only POST /api/execute can connect and move the robot.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("MoveIt real-arm service stopped.")
    finally:
        server.server_close()


def main() -> None:
    run_server()


if __name__ == "__main__":
    main()
