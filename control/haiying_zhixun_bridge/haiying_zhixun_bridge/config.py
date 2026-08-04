from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlparse

import yaml

from .contracts import ACCEPTED_FRAME_ID, CURRENT_STATE_TOPIC, JOINT_COUNT, TARGET_POSE_TOPIC


@dataclass(frozen=True)
class RosInterfaceConfig:
    target_pose_topic: str
    current_state_topic: str
    accepted_frame_id: str
    allowed_state: str
    orientation_supported: bool


@dataclass(frozen=True)
class IkConfig:
    server_url: str
    expected_trajectory_frames: int
    maximum_position_error_m: float


@dataclass(frozen=True)
class MoveItRealConfig:
    server_url: str
    hardware_execution_enabled: bool
    display_trajectory_topic: str
    joint_states_topic: str
    simulation_tolerance_deg: float
    joint_state_timeout_s: float


@dataclass(frozen=True)
class RobotPrototypeConfig:
    robot_type: str
    port: str
    calibration_id: str
    joint_names: tuple[str, ...]


@dataclass(frozen=True)
class GeometryConfig:
    dimensions_mm: tuple[float, ...]
    urdf_source: str
    end_effector: str


@dataclass(frozen=True)
class MappingConfig:
    direction_signs: tuple[float, ...]
    zero_offsets_deg: tuple[float, ...]


@dataclass(frozen=True)
class SafetyConfig:
    simulation_only: bool
    jog_max_delta_deg: float
    dry_run_max_frame_step_deg: float
    restricted_execute_max_frame_step_deg: float
    restricted_execute_start_error_deg: float
    restricted_execute_feedback_error_deg: float
    restricted_execute_positive_z_min_m: float
    restricted_execute_positive_z_max_m: float


@dataclass(frozen=True)
class BridgeConfig:
    ros2: RosInterfaceConfig
    ik: IkConfig
    moveit_real: MoveItRealConfig
    robot: RobotPrototypeConfig
    geometry: GeometryConfig
    mapping: MappingConfig
    safety: SafetyConfig


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"配置项 {name} 必须是映射")
    return value


def _text(mapping: Mapping[str, object], key: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"配置项 {key} 必须是非空字符串")
    return value


def _number(mapping: Mapping[str, object], key: str) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"配置项 {key} 必须是有限数值")
    return float(value)


def _numbers(value: object, name: str, length: int) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"配置项 {name} 必须包含 {length} 个数值")
    if not all(
        not isinstance(item, bool)
        and isinstance(item, int | float)
        and math.isfinite(float(item))
        for item in value
    ):
        raise ValueError(f"配置项 {name} 包含非法数值")
    return tuple(float(item) for item in value)


def _local_server_url(value: str) -> str:
    parsed = urlparse(value)
    if parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost"}:
        raise ValueError("IK server_url 只允许本机 HTTP 地址")
    if (
        parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or parsed.path not in {"", "/"}
    ):
        raise ValueError("IK server_url 不允许认证信息、查询参数或片段")
    try:
        parsed.port
    except ValueError as error:
        raise ValueError("IK server_url 端口非法") from error
    return value.rstrip("/")


def load_bridge_config(path: Path) -> BridgeConfig:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    root = _mapping(payload, "root")
    ros2 = _mapping(root.get("ros2"), "ros2")
    ik = _mapping(root.get("ik"), "ik")
    moveit_real = _mapping(root.get("moveit_real"), "moveit_real")
    robot = _mapping(root.get("robot"), "robot")
    geometry = _mapping(root.get("geometry"), "geometry")
    dimensions = _mapping(geometry.get("dimensions_mm"), "geometry.dimensions_mm")
    mapping = _mapping(root.get("mapping"), "mapping")
    safety = _mapping(root.get("safety"), "safety")

    orientation_supported = ros2.get("orientation_supported")
    if not isinstance(orientation_supported, bool):
        raise ValueError("ros2.orientation_supported 必须是布尔值")
    ros_config = RosInterfaceConfig(
        target_pose_topic=_text(ros2, "target_pose_topic"),
        current_state_topic=_text(ros2, "current_state_topic"),
        accepted_frame_id=_text(ros2, "accepted_frame_id"),
        allowed_state=_text(ros2, "allowed_state"),
        orientation_supported=orientation_supported,
    )
    if ros_config.target_pose_topic != TARGET_POSE_TOPIC:
        raise ValueError(f"target_pose_topic 必须固定为 {TARGET_POSE_TOPIC}")
    if ros_config.current_state_topic != CURRENT_STATE_TOPIC:
        raise ValueError(f"current_state_topic 必须固定为 {CURRENT_STATE_TOPIC}")
    if ros_config.accepted_frame_id != ACCEPTED_FRAME_ID:
        raise ValueError(f"accepted_frame_id 必须固定为 {ACCEPTED_FRAME_ID}")
    if ros_config.allowed_state != "BRUSHING":
        raise ValueError("allowed_state 必须固定为 BRUSHING")
    if ros_config.orientation_supported:
        raise ValueError("当前五自由度桥接不支持姿态约束")

    server_url = _local_server_url(_text(ik, "server_url"))
    expected_frames_value = _number(ik, "expected_trajectory_frames")
    expected_frames = int(expected_frames_value)
    if expected_frames <= 0 or expected_frames != expected_frames_value:
        raise ValueError("expected_trajectory_frames 必须为正整数")
    maximum_position_error_m = _number(ik, "maximum_position_error_m")
    if maximum_position_error_m <= 0.0:
        raise ValueError("maximum_position_error_m 必须为正数")
    ik_config = IkConfig(
        server_url=server_url,
        expected_trajectory_frames=expected_frames,
        maximum_position_error_m=maximum_position_error_m,
    )

    hardware_execution_enabled = moveit_real.get("hardware_execution_enabled")
    if not isinstance(hardware_execution_enabled, bool):
        raise ValueError("moveit_real.hardware_execution_enabled 必须是布尔值")
    moveit_real_config = MoveItRealConfig(
        server_url=_local_server_url(_text(moveit_real, "server_url")),
        hardware_execution_enabled=hardware_execution_enabled,
        display_trajectory_topic=_text(moveit_real, "display_trajectory_topic"),
        joint_states_topic=_text(moveit_real, "joint_states_topic"),
        simulation_tolerance_deg=_number(moveit_real, "simulation_tolerance_deg"),
        joint_state_timeout_s=_number(moveit_real, "joint_state_timeout_s"),
    )
    if moveit_real_config.display_trajectory_topic != "/display_planned_path":
        raise ValueError("MoveIt display_trajectory_topic 必须是 /display_planned_path")
    if moveit_real_config.joint_states_topic != "/joint_states":
        raise ValueError("MoveIt joint_states_topic 必须是 /joint_states")
    if not 0.0 < moveit_real_config.simulation_tolerance_deg <= 3.0:
        raise ValueError("simulation_tolerance_deg 必须大于 0 且不超过 3°")
    if not 0.1 <= moveit_real_config.joint_state_timeout_s <= 10.0:
        raise ValueError("joint_state_timeout_s 必须位于 0.1 到 10 秒")

    joint_names_value = robot.get("joint_names")
    if not isinstance(joint_names_value, list) or len(joint_names_value) != JOINT_COUNT:
        raise ValueError(f"robot.joint_names 必须包含 {JOINT_COUNT} 个关节")
    if not all(isinstance(name, str) and name for name in joint_names_value):
        raise ValueError("robot.joint_names 包含非法名称")
    robot_config = RobotPrototypeConfig(
        robot_type=_text(robot, "type"),
        port=_text(robot, "port"),
        calibration_id=_text(robot, "calibration_id"),
        joint_names=tuple(joint_names_value),
    )
    expected_joint_names = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    )
    if robot_config.robot_type != "so101_follower_5dof":
        raise ValueError("当前原型 robot.type 必须是 so101_follower_5dof")
    if robot_config.joint_names != expected_joint_names:
        raise ValueError("robot.joint_names 必须保持已标定的五轴顺序")

    dimension_keys = ("j1_to_j2", "j2_to_j3", "j3_to_j4", "j4_to_j5", "j5_to_tcp")
    geometry_config = GeometryConfig(
        dimensions_mm=tuple(_number(dimensions, key) for key in dimension_keys),
        urdf_source=_text(geometry, "urdf_source"),
        end_effector=_text(geometry, "end_effector"),
    )
    if any(dimension <= 0.0 for dimension in geometry_config.dimensions_mm):
        raise ValueError("geometry.dimensions_mm 中的尺寸必须为正数")
    if not geometry_config.urdf_source.startswith("package://"):
        raise ValueError("geometry.urdf_source 必须使用可移植的 package:// URI")
    if geometry_config.end_effector != "卡扣式可替换毛刷模块":
        raise ValueError("项目末端执行器必须固定为卡扣式可替换毛刷模块")

    mapping_config = MappingConfig(
        direction_signs=_numbers(mapping.get("direction_signs"), "mapping.direction_signs", JOINT_COUNT),
        zero_offsets_deg=_numbers(mapping.get("zero_offsets_deg"), "mapping.zero_offsets_deg", JOINT_COUNT),
    )
    if any(sign not in {-1.0, 1.0} for sign in mapping_config.direction_signs):
        raise ValueError("mapping.direction_signs 只能包含 -1 或 +1")

    simulation_only = safety.get("simulation_only")
    if not isinstance(simulation_only, bool):
        raise ValueError("safety.simulation_only 必须是布尔值")
    safety_config = SafetyConfig(
        simulation_only=simulation_only,
        jog_max_delta_deg=_number(safety, "jog_max_delta_deg"),
        dry_run_max_frame_step_deg=_number(safety, "dry_run_max_frame_step_deg"),
        restricted_execute_max_frame_step_deg=_number(
            safety, "restricted_execute_max_frame_step_deg"
        ),
        restricted_execute_start_error_deg=_number(safety, "restricted_execute_start_error_deg"),
        restricted_execute_feedback_error_deg=_number(
            safety, "restricted_execute_feedback_error_deg"
        ),
        restricted_execute_positive_z_min_m=_number(
            safety, "restricted_execute_positive_z_min_m"
        ),
        restricted_execute_positive_z_max_m=_number(
            safety, "restricted_execute_positive_z_max_m"
        ),
    )
    positive_safety_values = (
        safety_config.jog_max_delta_deg,
        safety_config.dry_run_max_frame_step_deg,
        safety_config.restricted_execute_max_frame_step_deg,
        safety_config.restricted_execute_start_error_deg,
        safety_config.restricted_execute_feedback_error_deg,
        safety_config.restricted_execute_positive_z_min_m,
        safety_config.restricted_execute_positive_z_max_m,
    )
    if any(value <= 0.0 for value in positive_safety_values):
        raise ValueError("所有 safety 数值限制必须为正数")
    if safety_config.restricted_execute_positive_z_min_m >= safety_config.restricted_execute_positive_z_max_m:
        raise ValueError("restricted execute 的 Z 位移上下限顺序错误")

    return BridgeConfig(
        ros2=ros_config,
        ik=ik_config,
        moveit_real=moveit_real_config,
        robot=robot_config,
        geometry=geometry_config,
        mapping=mapping_config,
        safety=safety_config,
    )
