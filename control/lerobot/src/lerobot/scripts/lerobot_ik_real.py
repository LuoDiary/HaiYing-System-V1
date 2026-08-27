import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal, Protocol, cast
from urllib.parse import urlparse
from urllib.request import Request, urlopen

import draccus

from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
from lerobot.utils.utils import init_logging


Mode = Literal["inspect", "jog", "dry-run", "execute"]
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
MAX_JOG_DELTA_DEG = 10.0
EXPECTED_TRAJECTORY_FRAMES = 91
JOINT_DIRECTIONS: tuple[float, ...] = (1.0, 1.0, -1.0, 1.0, 1.0)
JOINT_OFFSETS_DEG: tuple[float, ...] = (
    -6.417582417582418,
    -0.7472527472527473,
    -0.5274725274725275,
    16.967032967032967,
    -6.197802197802198,
)
EXECUTE_MAX_FRAME_STEP_DEG = 1.5
EXECUTE_MAX_POSITION_ERROR_M = 0.00025
EXECUTE_MAX_JOINT_EXCURSION_DEG = 10.0
EXECUTE_MAX_START_ERROR_DEG = 2.0
EXECUTE_MAX_FEEDBACK_ERROR_DEG = 3.0
EXECUTE_RATE_HZ = 10.0
EXECUTE_REFERENCE_TCP_M: tuple[float, float, float] = (
    0.005533999999999999,
    -0.179839003114076,
    0.16121899518123253,
)
EXECUTE_MAX_REFERENCE_TCP_ERROR_M = 0.0005
EXECUTE_MAX_INITIAL_URDF_ANGLE_DEG = 0.1
EXECUTE_MAX_COMMAND_CLIP_DEG = 0.05
EXECUTE_MAX_XY_DISPLACEMENT_M = 0.0005
EXECUTE_MIN_Z_DISPLACEMENT_M = 0.008
EXECUTE_MAX_Z_DISPLACEMENT_M = 0.012


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
    joint: str | None = None
    delta_deg: float | None = None
    settle_s: float = 0.8
    server_url: str = "http://127.0.0.1:8766"
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    plan_id: str | None = None
    max_frame_step_deg: float = 1.0
    confirm_execute: bool = False

    def __post_init__(self) -> None:
        if self.mode == "jog":
            if self.joint not in JOINT_NAMES:
                raise ValueError(f"jog 模式的 joint 必须是：{', '.join(JOINT_NAMES)}")
            if self.delta_deg is None or not math.isfinite(self.delta_deg) or self.delta_deg == 0.0:
                raise ValueError("jog 模式必须提供非零有限数值 delta_deg")
            if abs(self.delta_deg) > MAX_JOG_DELTA_DEG:
                raise ValueError(f"单次点动不得超过 {MAX_JOG_DELTA_DEG:.1f}°")
        elif self.joint is not None or self.delta_deg is not None:
            raise ValueError("joint 和 delta_deg 只允许用于 jog 模式")
        if not math.isfinite(self.settle_s) or self.settle_s < 0.0 or self.settle_s > 5.0:
            raise ValueError("settle_s 必须位于 0 到 5 秒之间")
        if not math.isfinite(self.max_frame_step_deg) or not 0.0 < self.max_frame_step_deg <= 3.0:
            raise ValueError("max_frame_step_deg 必须大于 0 且不超过 3°")
        if self.mode in {"dry-run", "execute"}:
            _validate_local_server_url(self.server_url)
            if self.plan_id is not None and (not self.plan_id.isalnum() or len(self.plan_id) > 64):
                raise ValueError("plan_id 只能包含字母和数字，且长度不得超过 64")
        if self.mode == "dry-run":
            target = (self.target_x, self.target_y, self.target_z)
            if not all(math.isfinite(value) for value in target):
                raise ValueError("dry-run 目标坐标必须是有限数值")
        if self.mode == "execute":
            if self.plan_id is None:
                raise ValueError("execute 模式必须提供 plan_id")
            if not self.confirm_execute:
                raise ValueError("execute 模式必须显式提供 --confirm_execute=true")
        elif self.confirm_execute:
            raise ValueError("confirm_execute 只允许用于 execute 模式")


@dataclass(frozen=True)
class JogResult:
    joint: str
    delta_deg: float
    before_deg: dict[str, float]
    sent_deg: dict[str, float]
    feedback_deg: dict[str, float]


@dataclass(frozen=True)
class DryRunResult:
    target_position_m: tuple[float, float, float]
    joint_names: tuple[str, ...]
    joint_angles_deg: tuple[float, ...]
    error_m: float
    trajectory_frames: int
    maximum_frame_step_deg: float
    collision_free: bool
    cartesian_execution_locked: bool


@dataclass(frozen=True)
class ExecutePlan:
    plan_id: str
    urdf_trajectory_deg: tuple[tuple[float, ...], ...]
    tcp_start_m: tuple[float, float, float]
    tcp_end_m: tuple[float, float, float]


@dataclass(frozen=True)
class ExecuteResult:
    plan_id: str
    verified_initial_frame: bool
    commanded_frames: int
    tcp_start_m: tuple[float, float, float]
    tcp_end_m: tuple[float, float, float]
    final_sent_deg: dict[str, float]
    final_feedback_deg: dict[str, float]
    maximum_feedback_error_deg: float
    execution_completed: bool


def _calibration_path(cfg: IKRealConfig) -> Path:
    directory = (
        cfg.calibration_dir
        if cfg.calibration_dir is not None
        else HF_LEROBOT_CALIBRATION / ROBOTS / "so101_follower_5dof"
    )
    return directory / f"{cfg.robot_id}.json"


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


def _make_robot(cfg: IKRealConfig, max_relative_target_deg: float) -> RobotLike:
    from lerobot.robots.so_follower import SO101Follower5DOF, SO101Follower5DOFConfig

    calibration_path = _calibration_path(cfg)
    validate_calibration_file(calibration_path)
    robot_config = SO101Follower5DOFConfig(
        port=cfg.port,
        id=cfg.robot_id,
        calibration_dir=calibration_path.parent,
        max_relative_target=max_relative_target_deg,
        use_degrees=True,
    )
    return SO101Follower5DOF(robot_config)


def _positions(observation: dict[str, object]) -> dict[str, float]:
    positions: dict[str, float] = {}
    for joint_name in JOINT_NAMES:
        key = f"{joint_name}.pos"
        value = observation.get(key)
        if not isinstance(value, int | float) or not math.isfinite(float(value)):
            raise ValueError(f"实机观测缺少有限角度：{key}")
        positions[joint_name] = float(value)
    return positions


def inspect_robot(robot: RobotLike) -> dict[str, float]:
    return _positions(cast(dict[str, object], robot.get_observation()))


def jog_robot(
    robot: RobotLike,
    joint_name: str,
    delta_deg: float,
    settle_s: float,
) -> JogResult:
    if joint_name not in JOINT_NAMES:
        raise ValueError(f"未知关节：{joint_name}")
    if not math.isfinite(delta_deg) or delta_deg == 0.0 or abs(delta_deg) > MAX_JOG_DELTA_DEG:
        raise ValueError(f"点动增量必须非零且不超过 {MAX_JOG_DELTA_DEG:.1f}°")
    before = inspect_robot(robot)
    requested = {f"{name}.pos": angle for name, angle in before.items()}
    requested[f"{joint_name}.pos"] += delta_deg
    sent = _positions(cast(dict[str, object], robot.send_action(requested)))
    if settle_s > 0.0:
        time.sleep(settle_s)
    feedback = inspect_robot(robot)
    return JogResult(joint_name, delta_deg, before, sent, feedback)


def _validate_local_server_url(server_url: str) -> str:
    parsed = urlparse(server_url)
    if parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost"}:
        raise ValueError("IK server_url 只允许本机 http://127.0.0.1 或 http://localhost")
    if parsed.username is not None or parsed.password is not None or parsed.query or parsed.fragment:
        raise ValueError("IK server_url 不允许认证信息、查询参数或片段")
    return server_url.rstrip("/")


def _request_plan(
    server_url: str,
    target_position_m: tuple[float, float, float],
) -> dict[str, object]:
    url = f"{_validate_local_server_url(server_url)}/api/plan"
    body = json.dumps(
        {"target_position_m": list(target_position_m), "initial_joint_angles_deg": [0.0] * 5}
    ).encode("utf-8")
    request = Request(url, data=body, headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(request, timeout=30) as response:
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise ValueError("IK 服务器返回值不是 JSON 对象")
    return cast(dict[str, object], payload)


def _fetch_plan(server_url: str, plan_id: str) -> dict[str, object]:
    url = f"{_validate_local_server_url(server_url)}/api/plans/{plan_id}"
    with urlopen(url, timeout=30) as response:
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise ValueError("IK 服务器返回值不是 JSON 对象")
    return cast(dict[str, object], payload)


def validate_plan_for_dry_run(
    payload: dict[str, object],
    target_position_m: tuple[float, float, float],
    max_frame_step_deg: float,
) -> DryRunResult:
    if payload.get("success") is not True:
        raise ValueError(f"IK 规划失败：{payload.get('message', '未知错误')}")
    if payload.get("collision_free") is not True:
        raise ValueError("IK 轨迹没有通过自碰撞检查")
    trajectory_value = payload.get("trajectory_deg")
    joint_angles_value = payload.get("joint_angles_deg")
    error_value = payload.get("error_m")
    if not isinstance(trajectory_value, list) or len(trajectory_value) != EXPECTED_TRAJECTORY_FRAMES:
        raise ValueError(f"IK 轨迹必须包含 {EXPECTED_TRAJECTORY_FRAMES} 帧")
    if not isinstance(joint_angles_value, list) or len(joint_angles_value) != len(JOINT_NAMES):
        raise ValueError("IK 结果必须包含五个目标关节角")
    if not isinstance(error_value, int | float) or not math.isfinite(float(error_value)):
        raise ValueError("IK 结果缺少有限位置误差")

    trajectory: list[tuple[float, ...]] = []
    for frame_index, frame_value in enumerate(trajectory_value):
        if not isinstance(frame_value, list) or len(frame_value) != len(JOINT_NAMES):
            raise ValueError(f"IK 第 {frame_index + 1} 帧不是五关节角")
        if not all(isinstance(value, int | float) and math.isfinite(float(value)) for value in frame_value):
            raise ValueError(f"IK 第 {frame_index + 1} 帧包含非法角度")
        trajectory.append(tuple(float(value) for value in frame_value))

    maximum_step = 0.0
    for previous, current in zip(trajectory[:-1], trajectory[1:], strict=True):
        maximum_step = max(maximum_step, max(abs(end - start) for start, end in zip(previous, current, strict=True)))
    if maximum_step > max_frame_step_deg:
        raise ValueError(
            f"IK 最大单帧变化 {maximum_step:.3f}°，超过 dry-run 限制 {max_frame_step_deg:.3f}°"
        )
    joint_angles = tuple(float(value) for value in joint_angles_value)
    if not all(math.isfinite(value) for value in joint_angles):
        raise ValueError("IK 目标关节角包含非法数值")
    return DryRunResult(
        target_position_m=target_position_m,
        joint_names=URDF_JOINT_NAMES,
        joint_angles_deg=joint_angles,
        error_m=float(error_value),
        trajectory_frames=len(trajectory),
        maximum_frame_step_deg=maximum_step,
        collision_free=True,
        cartesian_execution_locked=True,
    )


def _trajectory_from_payload(payload: dict[str, object]) -> tuple[tuple[float, ...], ...]:
    trajectory_value = payload.get("trajectory_deg")
    if not isinstance(trajectory_value, list) or len(trajectory_value) != EXPECTED_TRAJECTORY_FRAMES:
        raise ValueError(f"IK 轨迹必须包含 {EXPECTED_TRAJECTORY_FRAMES} 帧")
    trajectory: list[tuple[float, ...]] = []
    for frame_index, frame_value in enumerate(trajectory_value):
        if not isinstance(frame_value, list) or len(frame_value) != len(JOINT_NAMES):
            raise ValueError(f"IK 第 {frame_index + 1} 帧不是五关节角")
        if not all(isinstance(value, int | float) and math.isfinite(float(value)) for value in frame_value):
            raise ValueError(f"IK 第 {frame_index + 1} 帧包含非法角度")
        trajectory.append(tuple(float(value) for value in frame_value))
    return tuple(trajectory)


def _tcp_path_from_payload(payload: dict[str, object]) -> tuple[tuple[float, float, float], ...]:
    frames_value = payload.get("trajectory_points_m")
    if not isinstance(frames_value, list) or len(frames_value) != EXPECTED_TRAJECTORY_FRAMES:
        raise ValueError(f"IK TCP 轨迹必须包含 {EXPECTED_TRAJECTORY_FRAMES} 帧")
    tcp_path: list[tuple[float, float, float]] = []
    for frame_index, points_value in enumerate(frames_value):
        if not isinstance(points_value, list) or not points_value:
            raise ValueError(f"IK 第 {frame_index + 1} 帧缺少 TCP 坐标")
        tcp_value = points_value[-1]
        if not isinstance(tcp_value, list) or len(tcp_value) != 3:
            raise ValueError(f"IK 第 {frame_index + 1} 帧 TCP 坐标格式错误")
        if not all(isinstance(value, int | float) and math.isfinite(float(value)) for value in tcp_value):
            raise ValueError(f"IK 第 {frame_index + 1} 帧 TCP 坐标包含非法数值")
        tcp_path.append(tuple(float(value) for value in tcp_value))
    return tuple(tcp_path)


def validate_plan_for_execute(payload: dict[str, object], plan_id: str) -> ExecutePlan:
    if payload.get("plan_id") != plan_id:
        raise ValueError("IK 规划编号与请求的 plan_id 不一致")
    target_value = payload.get("target_position_m")
    if (
        not isinstance(target_value, list)
        or len(target_value) != 3
        or not all(isinstance(value, int | float) and math.isfinite(float(value)) for value in target_value)
    ):
        raise ValueError("IK 规划缺少三维目标坐标")
    target = tuple(float(value) for value in target_value)
    validate_plan_for_dry_run(payload, target, EXECUTE_MAX_FRAME_STEP_DEG)
    error_value = payload.get("error_m")
    if not isinstance(error_value, int | float) or float(error_value) > EXECUTE_MAX_POSITION_ERROR_M:
        raise ValueError(
            f"execute IK 位置误差必须不超过 {EXECUTE_MAX_POSITION_ERROR_M * 1000:.3f} mm"
        )
    trajectory = _trajectory_from_payload(payload)
    tcp_path = _tcp_path_from_payload(payload)
    start_tcp = tcp_path[0]
    end_tcp = tcp_path[-1]
    maximum_initial_angle = max(abs(angle) for angle in trajectory[0])
    if maximum_initial_angle > EXECUTE_MAX_INITIAL_URDF_ANGLE_DEG:
        raise ValueError(
            f"execute 轨迹必须从 URDF 全零参考姿态开始，首帧最大角为 {maximum_initial_angle:.3f}°"
        )
    reference_tcp_error = math.dist(start_tcp, EXECUTE_REFERENCE_TCP_M)
    if reference_tcp_error > EXECUTE_MAX_REFERENCE_TCP_ERROR_M:
        raise ValueError(
            f"execute TCP 起点偏离参考点 {reference_tcp_error * 1000:.3f} mm，超过限制"
        )
    displacement = tuple(end - start for start, end in zip(start_tcp, end_tcp, strict=True))
    if abs(displacement[0]) > EXECUTE_MAX_XY_DISPLACEMENT_M:
        raise ValueError(f"execute 的 X 位移 {displacement[0] * 1000:.3f} mm 超过限制")
    if abs(displacement[1]) > EXECUTE_MAX_XY_DISPLACEMENT_M:
        raise ValueError(f"execute 的 Y 位移 {displacement[1] * 1000:.3f} mm 超过限制")
    if not EXECUTE_MIN_Z_DISPLACEMENT_M <= displacement[2] <= EXECUTE_MAX_Z_DISPLACEMENT_M:
        raise ValueError(
            f"execute 的 Z 位移必须为 +{EXECUTE_MIN_Z_DISPLACEMENT_M * 1000:.1f} 至 "
            f"+{EXECUTE_MAX_Z_DISPLACEMENT_M * 1000:.1f} mm，实际为 {displacement[2] * 1000:.3f} mm"
        )
    start_angles = trajectory[0]
    maximum_excursion = max(
        abs(angle - initial)
        for frame in trajectory
        for angle, initial in zip(frame, start_angles, strict=True)
    )
    if maximum_excursion > EXECUTE_MAX_JOINT_EXCURSION_DEG:
        raise ValueError(
            f"execute 最大关节总变化 {maximum_excursion:.3f}°，超过限制 "
            f"{EXECUTE_MAX_JOINT_EXCURSION_DEG:.1f}°"
        )
    return ExecutePlan(plan_id, trajectory, start_tcp, end_tcp)


def map_urdf_angles_to_real(urdf_angles_deg: tuple[float, ...]) -> dict[str, float]:
    if len(urdf_angles_deg) != len(JOINT_NAMES) or not all(math.isfinite(value) for value in urdf_angles_deg):
        raise ValueError("URDF 角度必须包含五个有限数值")
    mapped_values = tuple(
        direction * angle + offset
        for angle, direction, offset in zip(
            urdf_angles_deg, JOINT_DIRECTIONS, JOINT_OFFSETS_DEG, strict=True
        )
    )
    return dict(zip(JOINT_NAMES, mapped_values, strict=True))


def execute_plan(robot: RobotLike, plan: ExecutePlan, rate_hz: float) -> ExecuteResult:
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("execute rate_hz 必须为正的有限数值")
    real_trajectory = tuple(map_urdf_angles_to_real(frame) for frame in plan.urdf_trajectory_deg)
    initial_feedback = inspect_robot(robot)
    initial_target = real_trajectory[0]
    initial_errors = {
        name: abs(initial_feedback[name] - initial_target[name]) for name in JOINT_NAMES
    }
    worst_initial_joint = max(initial_errors, key=initial_errors.get)
    if initial_errors[worst_initial_joint] > EXECUTE_MAX_START_ERROR_DEG:
        raise RuntimeError(
            f"execute 首帧不匹配：{worst_initial_joint} 相差 "
            f"{initial_errors[worst_initial_joint]:.3f}°，未发送动作"
        )

    period_s = 1.0 / rate_hz
    maximum_feedback_error = 0.0
    final_sent = dict(initial_target)
    final_feedback = dict(initial_feedback)
    commanded_frames = 0
    for frame_index, target in enumerate(real_trajectory[1:], start=2):
        frame_start = time.perf_counter()
        action = {f"{name}.pos": value for name, value in target.items()}
        final_sent = _positions(cast(dict[str, object], robot.send_action(action)))
        command_errors = {name: abs(final_sent[name] - target[name]) for name in JOINT_NAMES}
        clipped_joint = max(command_errors, key=command_errors.get)
        if command_errors[clipped_joint] > EXECUTE_MAX_COMMAND_CLIP_DEG:
            raise RuntimeError(
                f"execute 在第 {frame_index}/{EXPECTED_TRAJECTORY_FRAMES} 帧中止："
                f"{clipped_joint} 命令被限幅 {command_errors[clipped_joint]:.3f}°"
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
        if feedback_errors[worst_joint] > EXECUTE_MAX_FEEDBACK_ERROR_DEG:
            raise RuntimeError(
                f"execute 在第 {frame_index}/{EXPECTED_TRAJECTORY_FRAMES} 帧中止："
                f"{worst_joint} 反馈误差 {feedback_errors[worst_joint]:.3f}°"
            )
    return ExecuteResult(
        plan_id=plan.plan_id,
        verified_initial_frame=True,
        commanded_frames=commanded_frames,
        tcp_start_m=plan.tcp_start_m,
        tcp_end_m=plan.tcp_end_m,
        final_sent_deg=final_sent,
        final_feedback_deg=final_feedback,
        maximum_feedback_error_deg=maximum_feedback_error,
        execution_completed=True,
    )


def run_inspect(cfg: IKRealConfig) -> dict[str, float]:
    robot = _make_robot(cfg, MAX_JOG_DELTA_DEG)
    try:
        robot.connect(calibrate=False)
        return inspect_robot(robot)
    finally:
        if robot.is_connected:
            robot.disconnect()


def run_jog(cfg: IKRealConfig) -> JogResult:
    if cfg.joint is None or cfg.delta_deg is None:
        raise ValueError("jog 模式缺少 joint 或 delta_deg")
    robot = _make_robot(cfg, MAX_JOG_DELTA_DEG)
    try:
        robot.connect(calibrate=False)
        return jog_robot(robot, cfg.joint, cfg.delta_deg, cfg.settle_s)
    finally:
        if robot.is_connected:
            robot.disconnect()


def run_dry_run(cfg: IKRealConfig) -> DryRunResult:
    target = (cfg.target_x, cfg.target_y, cfg.target_z)
    payload = (
        _fetch_plan(cfg.server_url, cfg.plan_id)
        if cfg.plan_id is not None
        else _request_plan(cfg.server_url, target)
    )
    if cfg.plan_id is not None:
        payload_target = payload.get("target_position_m")
        if not isinstance(payload_target, list) or len(payload_target) != 3:
            raise ValueError("已保存的 IK 规划缺少三维目标坐标")
        target = tuple(float(value) for value in payload_target)
    return validate_plan_for_dry_run(payload, target, cfg.max_frame_step_deg)


def run_execute(cfg: IKRealConfig) -> ExecuteResult:
    if cfg.plan_id is None:
        raise ValueError("execute 模式缺少 plan_id")
    payload = _fetch_plan(cfg.server_url, cfg.plan_id)
    plan = validate_plan_for_execute(payload, cfg.plan_id)
    robot = _make_robot(cfg, EXECUTE_MAX_FRAME_STEP_DEG)
    try:
        robot.connect(calibrate=False)
        return execute_plan(robot, plan, EXECUTE_RATE_HZ)
    finally:
        if robot.is_connected:
            robot.disconnect()


@draccus.wrap()
def run(cfg: IKRealConfig) -> None:
    init_logging()
    if cfg.mode == "inspect":
        result: object = run_inspect(cfg)
    elif cfg.mode == "jog":
        result = asdict(run_jog(cfg))
    elif cfg.mode == "dry-run":
        result = asdict(run_dry_run(cfg))
    else:
        result = asdict(run_execute(cfg))
    print(json.dumps(result, ensure_ascii=False, indent=2))


def main() -> None:
    run()


if __name__ == "__main__":
    main()
