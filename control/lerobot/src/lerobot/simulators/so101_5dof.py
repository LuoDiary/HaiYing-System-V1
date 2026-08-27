from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from struct import unpack_from
from typing import Sequence

import numpy as np
from numpy.typing import NDArray


# The migrated runtime lives at <workspace>/vendor/lerobot. Keep the model
# location portable inside the workspace while still allowing an override.
DEFAULT_MODEL_ROOT = Path(
    os.environ.get(
        "HAIYING_ARM_URDF_ROOT",
        Path(__file__).resolve().parents[5] / "src" / "arm_urdf",
    )
)
DEFAULT_MODEL_FILE = "so101_arm.urdf.xacro"
DEFAULT_INITIAL_JOINT_ANGLES_DEG: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)
POSITION_TOLERANCE_M = 0.002
PREFERRED_POSITION_ERROR_M = 0.00025
COLLISION_PAIR_MARGIN_M = 0.002
SAT_EPSILON = 1e-9


@dataclass(frozen=True)
class VisualSpec:
    mesh_relative_path: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    scale: tuple[float, float, float]
    color_rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class CollisionSpec:
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    local_center: tuple[float, float, float]
    half_extents: tuple[float, float, float]


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower_rad: float
    upper_rad: float


@dataclass(frozen=True)
class RobotModel:
    model_root: Path
    source_path: Path
    root_link: str
    tcp_link: str
    link_names: tuple[str, ...]
    chain_joints: tuple[JointSpec, ...]
    movable_joints: tuple[JointSpec, ...]
    visuals_by_link: dict[str, tuple[VisualSpec, ...]]
    collisions_by_link: dict[str, tuple[CollisionSpec, ...]]
    adjacent_link_pairs: frozenset[frozenset[str]]


@dataclass(frozen=True)
class KinematicState:
    end_effector_transform: NDArray[np.float64]
    points: NDArray[np.float64]
    joint_axes: NDArray[np.float64]
    joint_positions: NDArray[np.float64]
    link_transforms: dict[str, NDArray[np.float64]]


@dataclass(frozen=True)
class PlanResult:
    success: bool
    message: str
    target_position_m: list[float]
    reached_position_m: list[float]
    error_m: float
    joint_angles_deg: list[float]
    trajectory_deg: list[list[float]]
    trajectory_points_m: list[list[list[float]]]
    trajectory_link_matrices: list[dict[str, list[float]]]
    iterations: int
    collision_free: bool
    first_collision_frame: int | None
    collision_link_pairs: list[list[str]]
    collision_joint_angles_deg: list[float]


@dataclass(frozen=True)
class OrientedBox:
    center: NDArray[np.float64]
    axes: NDArray[np.float64]
    half_extents: NDArray[np.float64]


@dataclass(frozen=True)
class TrajectoryCollision:
    frame_index: int
    link_pairs: tuple[tuple[str, str], ...]
    joint_angles_deg: tuple[float, ...]


def _vector(text: str | None, length: int) -> tuple[float, ...]:
    values = tuple(float(value) for value in (text or " ".join("0" for _ in range(length))).split())
    if len(values) != length:
        raise ValueError(f"Expected {length} values, got {len(values)}")
    return values


def _rpy_rotation(rpy: Sequence[float]) -> NDArray[np.float64]:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _axis_rotation(axis: Sequence[float], angle_rad: float) -> NDArray[np.float64]:
    unit_axis = np.asarray(axis, dtype=float)
    unit_axis /= np.linalg.norm(unit_axis)
    x, y, z = unit_axis
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    d = 1.0 - c
    return np.array(
        [
            [c + x * x * d, x * y * d - z * s, x * z * d + y * s],
            [y * x * d + z * s, c + y * y * d, y * z * d - x * s],
            [z * x * d - y * s, z * y * d + x * s, c + z * z * d],
        ],
        dtype=float,
    )


def _transform(xyz: Sequence[float], rotation: NDArray[np.float64]) -> NDArray[np.float64]:
    result = np.eye(4, dtype=float)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(xyz, dtype=float)
    return result


def _origin(element: ET.Element | None) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if element is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    return _vector(element.get("xyz"), 3), _vector(element.get("rpy"), 3)


def _mesh_relative_path(uri: str) -> str:
    prefix = "package://arm_urdf/"
    if not uri.startswith(prefix):
        raise ValueError(f"Only {prefix} mesh URIs are supported: {uri}")
    return uri.removeprefix(prefix).replace("\\", "/")


def _stl_vertices(mesh_path: Path) -> NDArray[np.float64]:
    data = mesh_path.read_bytes()
    if len(data) >= 84:
        triangle_count = unpack_from("<I", data, 80)[0]
        if len(data) == 84 + triangle_count * 50:
            triangle_dtype = np.dtype(
                [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
            )
            triangles = np.frombuffer(data, dtype=triangle_dtype, count=triangle_count, offset=84)
            return triangles["vertices"].reshape(-1, 3).astype(np.float64)

    vertices: list[tuple[float, float, float]] = []
    for line in data.decode("utf-8", errors="ignore").splitlines():
        fields = line.strip().split()
        if len(fields) == 4 and fields[0].lower() == "vertex":
            vertices.append((float(fields[1]), float(fields[2]), float(fields[3])))
    if not vertices:
        raise ValueError(f"无法读取 STL 顶点：{mesh_path}")
    return np.asarray(vertices, dtype=np.float64)


def _collision_spec(
    root_path: Path,
    collision: ET.Element,
) -> CollisionSpec | None:
    mesh = collision.find("geometry/mesh")
    if mesh is None:
        return None
    relative_path = _mesh_relative_path(mesh.get("filename", ""))
    mesh_path = (root_path / Path(relative_path)).resolve()
    if not mesh_path.is_relative_to(root_path) or not mesh_path.is_file():
        raise FileNotFoundError(f"模型引用的碰撞网格不存在：{mesh_path}")
    scale = np.asarray(_vector(mesh.get("scale"), 3) if mesh.get("scale") else (1.0, 1.0, 1.0))
    vertices = _stl_vertices(mesh_path) * scale
    lower = vertices.min(axis=0)
    upper = vertices.max(axis=0)
    xyz, rpy = _origin(collision.find("origin"))
    return CollisionSpec(
        origin_xyz=xyz,
        origin_rpy=rpy,
        local_center=tuple(((lower + upper) / 2.0).tolist()),
        half_extents=tuple(((upper - lower) / 2.0).tolist()),
    )


def load_robot_model(model_root: Path, model_file: str) -> RobotModel:
    root_path = model_root.expanduser().resolve()
    source_path = (root_path / "urdf" / model_file).resolve()
    if not source_path.is_file():
        raise FileNotFoundError(f"找不到无相机机械臂模型：{source_path}")
    xml_root = ET.parse(source_path).getroot()
    if xml_root.tag != "robot":
        raise ValueError(f"模型根节点不是 robot：{source_path}")

    link_elements = {element.get("name", ""): element for element in xml_root.findall("link")}
    material_colors: dict[str, tuple[float, float, float, float]] = {}
    for material in xml_root.findall("material"):
        color = material.find("color")
        if material.get("name") and color is not None:
            material_colors[material.get("name", "")] = _vector(color.get("rgba"), 4)

    joints: list[JointSpec] = []
    child_links: set[str] = set()
    for element in xml_root.findall("joint"):
        parent_element = element.find("parent")
        child_element = element.find("child")
        if parent_element is None or child_element is None:
            raise ValueError(f"关节缺少 parent/child：{element.get('name')}")
        parent = parent_element.get("link", "")
        child = child_element.get("link", "")
        xyz, rpy = _origin(element.find("origin"))
        axis_element = element.find("axis")
        axis = _vector(axis_element.get("xyz"), 3) if axis_element is not None else (0.0, 0.0, 1.0)
        limit = element.find("limit")
        joint_type = element.get("type", "fixed")
        lower = float(limit.get("lower", "0")) if limit is not None else 0.0
        upper = float(limit.get("upper", "0")) if limit is not None else 0.0
        joints.append(
            JointSpec(element.get("name", ""), joint_type, parent, child, xyz, rpy, axis, lower, upper)
        )
        child_links.add(child)

    roots = set(link_elements) - child_links
    if len(roots) != 1:
        raise ValueError(f"模型必须只有一个根 link，实际为：{sorted(roots)}")
    root_link = roots.pop()
    tcp_link = "end_effector"
    if tcp_link not in link_elements:
        raise ValueError(f"模型缺少 TCP link：{tcp_link}")

    by_parent = {joint.parent: joint for joint in joints}
    chain: list[JointSpec] = []
    current = root_link
    visited = {current}
    while current != tcp_link:
        joint = by_parent.get(current)
        if joint is None or joint.child in visited:
            raise ValueError(f"从 {root_link} 到 {tcp_link} 的运动链不完整")
        chain.append(joint)
        current = joint.child
        visited.add(current)
    movable = tuple(joint for joint in chain if joint.joint_type in {"revolute", "continuous"})
    if len(movable) != 5:
        raise ValueError(f"无相机模型应包含 5 个旋转关节，实际为 {len(movable)}")

    visuals: dict[str, tuple[VisualSpec, ...]] = {}
    collisions: dict[str, tuple[CollisionSpec, ...]] = {}
    for link_name, link_element in link_elements.items():
        link_visuals: list[VisualSpec] = []
        for visual in link_element.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            relative_path = _mesh_relative_path(mesh.get("filename", ""))
            mesh_path = (root_path / Path(relative_path)).resolve()
            if not mesh_path.is_relative_to(root_path) or not mesh_path.is_file():
                raise FileNotFoundError(f"模型引用的网格不存在：{mesh_path}")
            xyz, rpy = _origin(visual.find("origin"))
            scale = _vector(mesh.get("scale"), 3) if mesh.get("scale") else (1.0, 1.0, 1.0)
            material = visual.find("material")
            color = (0.55, 0.62, 0.72, 1.0)
            if material is not None:
                inline_color = material.find("color")
                if inline_color is not None:
                    color = _vector(inline_color.get("rgba"), 4)
                elif material.get("name") in material_colors:
                    color = material_colors[material.get("name", "")]
            link_visuals.append(VisualSpec(relative_path, xyz, rpy, scale, color))
        visuals[link_name] = tuple(link_visuals)
        link_collisions = tuple(
            spec
            for collision in link_element.findall("collision")
            if (spec := _collision_spec(root_path, collision)) is not None
        )
        collisions[link_name] = link_collisions

    adjacent_pairs = frozenset(frozenset((joint.parent, joint.child)) for joint in chain)

    return RobotModel(
        model_root=root_path,
        source_path=source_path,
        root_link=root_link,
        tcp_link=tcp_link,
        link_names=tuple(link_elements),
        chain_joints=tuple(chain),
        movable_joints=movable,
        visuals_by_link=visuals,
        collisions_by_link=collisions,
        adjacent_link_pairs=adjacent_pairs,
    )


def _validate_joint_vector(model: RobotModel, joint_angles_rad: Sequence[float]) -> NDArray[np.float64]:
    joints = np.asarray(joint_angles_rad, dtype=float)
    if joints.shape != (len(model.movable_joints),) or not np.all(np.isfinite(joints)):
        raise ValueError("joint_angles_rad 必须包含 5 个有限数值")
    return joints


def forward_kinematics(model: RobotModel, joint_angles_rad: Sequence[float]) -> KinematicState:
    angles = _validate_joint_vector(model, joint_angles_rad)
    transform = np.eye(4, dtype=float)
    link_transforms = {model.root_link: transform.copy()}
    points = [transform[:3, 3].copy()]
    joint_axes: list[NDArray[np.float64]] = []
    joint_positions: list[NDArray[np.float64]] = []
    movable_index = 0
    for joint in model.chain_joints:
        transform = transform @ _transform(joint.origin_xyz, _rpy_rotation(joint.origin_rpy))
        if joint.joint_type in {"revolute", "continuous"}:
            joint_positions.append(transform[:3, 3].copy())
            joint_axes.append(transform[:3, :3] @ np.asarray(joint.axis, dtype=float))
            transform = transform @ _transform((0.0, 0.0, 0.0), _axis_rotation(joint.axis, angles[movable_index]))
            points.append(transform[:3, 3].copy())
            movable_index += 1
        link_transforms[joint.child] = transform.copy()
    if not np.allclose(points[-1], transform[:3, 3]):
        points.append(transform[:3, 3].copy())
    else:
        points.append(transform[:3, 3].copy())
    return KinematicState(
        end_effector_transform=transform,
        points=np.asarray(points, dtype=float),
        joint_axes=np.asarray(joint_axes, dtype=float),
        joint_positions=np.asarray(joint_positions, dtype=float),
        link_transforms=link_transforms,
    )


def _world_collision_boxes(model: RobotModel, state: KinematicState) -> dict[str, tuple[OrientedBox, ...]]:
    boxes: dict[str, tuple[OrientedBox, ...]] = {}
    inflation = COLLISION_PAIR_MARGIN_M / 2.0
    for link_name, collision_specs in model.collisions_by_link.items():
        link_transform = state.link_transforms.get(link_name)
        if link_transform is None:
            continue
        link_boxes: list[OrientedBox] = []
        for spec in collision_specs:
            collision_transform = link_transform @ _transform(
                spec.origin_xyz, _rpy_rotation(spec.origin_rpy)
            )
            local_center = np.asarray(spec.local_center, dtype=float)
            center = collision_transform[:3, :3] @ local_center + collision_transform[:3, 3]
            link_boxes.append(
                OrientedBox(
                    center=center,
                    axes=collision_transform[:3, :3],
                    half_extents=np.asarray(spec.half_extents, dtype=float) + inflation,
                )
            )
        if link_boxes:
            boxes[link_name] = tuple(link_boxes)
    return boxes


def _oriented_boxes_overlap(first: OrientedBox, second: OrientedBox) -> bool:
    rotation = first.axes.T @ second.axes
    absolute_rotation = np.abs(rotation) + SAT_EPSILON
    center_delta = first.axes.T @ (second.center - first.center)
    first_extent = first.half_extents
    second_extent = second.half_extents

    for axis_index in range(3):
        first_radius = first_extent[axis_index]
        second_radius = float(second_extent @ absolute_rotation[axis_index, :])
        if abs(center_delta[axis_index]) > first_radius + second_radius:
            return False

    for axis_index in range(3):
        first_radius = float(first_extent @ absolute_rotation[:, axis_index])
        second_radius = second_extent[axis_index]
        if abs(center_delta @ rotation[:, axis_index]) > first_radius + second_radius:
            return False

    for first_axis in range(3):
        first_next = (first_axis + 1) % 3
        first_other = (first_axis + 2) % 3
        for second_axis in range(3):
            second_next = (second_axis + 1) % 3
            second_other = (second_axis + 2) % 3
            first_radius = (
                first_extent[first_next] * absolute_rotation[first_other, second_axis]
                + first_extent[first_other] * absolute_rotation[first_next, second_axis]
            )
            second_radius = (
                second_extent[second_next] * absolute_rotation[first_axis, second_other]
                + second_extent[second_other] * absolute_rotation[first_axis, second_next]
            )
            projected_distance = abs(
                center_delta[first_other] * rotation[first_next, second_axis]
                - center_delta[first_next] * rotation[first_other, second_axis]
            )
            if projected_distance > first_radius + second_radius:
                return False
    return True


def find_self_collisions(model: RobotModel, state: KinematicState) -> tuple[tuple[str, str], ...]:
    boxes = _world_collision_boxes(model, state)
    link_names = [name for name in model.link_names if name in boxes]
    collisions: list[tuple[str, str]] = []
    for first_index, first_name in enumerate(link_names):
        for second_name in link_names[first_index + 1 :]:
            if frozenset((first_name, second_name)) in model.adjacent_link_pairs:
                continue
            if any(
                _oriented_boxes_overlap(first_box, second_box)
                for first_box in boxes[first_name]
                for second_box in boxes[second_name]
            ):
                collisions.append((first_name, second_name))
    return tuple(collisions)


def _first_trajectory_collision(
    model: RobotModel,
    trajectory_rad: NDArray[np.float64],
    states: Sequence[KinematicState],
) -> TrajectoryCollision | None:
    for frame_index, (joint_angles, state) in enumerate(zip(trajectory_rad, states, strict=True)):
        link_pairs = find_self_collisions(model, state)
        if link_pairs:
            return TrajectoryCollision(
                frame_index=frame_index,
                link_pairs=link_pairs,
                joint_angles_deg=tuple(np.rad2deg(joint_angles).tolist()),
            )
    return None


def _position_jacobian(state: KinematicState) -> NDArray[np.float64]:
    endpoint = state.end_effector_transform[:3, 3]
    return np.column_stack(
        [
            np.cross(axis, endpoint - position)
            for axis, position in zip(state.joint_axes, state.joint_positions, strict=True)
        ]
    )


def _solve_from_seed(
    model: RobotModel,
    target: NDArray[np.float64],
    seed: NDArray[np.float64],
    reference: NDArray[np.float64],
    max_iterations: int,
) -> tuple[NDArray[np.float64], float, int]:
    lower = np.array([joint.lower_rad for joint in model.movable_joints])
    upper = np.array([joint.upper_rad for joint in model.movable_joints])
    center = (lower + upper) / 2.0
    span = upper - lower
    angles = np.clip(seed, lower, upper)
    damping = 0.035
    for iteration in range(1, max_iterations + 1):
        state = forward_kinematics(model, angles)
        error = target - state.end_effector_transform[:3, 3]
        error_norm = float(np.linalg.norm(error))
        if error_norm <= POSITION_TOLERANCE_M * 0.45:
            return angles, error_norm, iteration
        jacobian = _position_jacobian(state)
        regularized = jacobian @ jacobian.T + damping**2 * np.eye(3)
        pseudo_inverse = jacobian.T @ np.linalg.solve(regularized, np.eye(3))
        delta = pseudo_inverse @ error
        null_space = np.eye(len(angles)) - pseudo_inverse @ jacobian
        delta += null_space @ (0.025 * (reference - angles) + 0.008 * (center - angles) / span)
        largest = float(np.max(np.abs(delta)))
        if largest > 0.14:
            delta *= 0.14 / largest
        candidate = np.clip(angles + delta, lower, upper)
        candidate_error = float(
            np.linalg.norm(target - forward_kinematics(model, candidate).end_effector_transform[:3, 3])
        )
        if candidate_error <= error_norm:
            angles = candidate
            damping = max(damping * 0.85, 0.006)
        else:
            damping = min(damping * 2.0, 0.4)
    final = forward_kinematics(model, angles).end_effector_transform[:3, 3]
    return angles, float(np.linalg.norm(target - final)), max_iterations


def _ik_seeds(model: RobotModel, reference: NDArray[np.float64]) -> list[NDArray[np.float64]]:
    lower = np.array([joint.lower_rad for joint in model.movable_joints])
    upper = np.array([joint.upper_rad for joint in model.movable_joints])
    center = (lower + upper) / 2.0
    seeds = [reference.copy(), center]
    for pan in (-1.2, 0.0, 1.2):
        for shoulder in (-1.1, 0.0, 1.1):
            for elbow in (-1.1, 0.0, 1.1):
                seed = center.copy()
                seed[0] = pan
                seed[1] = shoulder
                seed[2] = elbow
                seed[3] = -0.5 * (shoulder + elbow)
                seeds.append(np.clip(seed, lower, upper))

    unique_seeds: list[NDArray[np.float64]] = []
    seen: set[tuple[float, ...]] = set()
    for seed in seeds:
        key = tuple(np.round(seed, decimals=8).tolist())
        if key not in seen:
            seen.add(key)
            unique_seeds.append(seed)
    return unique_seeds


def _smooth_trajectory(
    start_rad: NDArray[np.float64], target_rad: NDArray[np.float64], frame_count: int
) -> NDArray[np.float64]:
    time_values = np.linspace(0.0, 1.0, frame_count)
    scale = 10.0 * time_values**3 - 15.0 * time_values**4 + 6.0 * time_values**5
    return start_rad[None, :] + scale[:, None] * (target_rad - start_rad)[None, :]


def _matrix_payload(matrix: NDArray[np.float64]) -> list[float]:
    return matrix.T.reshape(-1).tolist()


def _link_matrix_payload(state: KinematicState) -> dict[str, list[float]]:
    return {name: _matrix_payload(matrix) for name, matrix in state.link_transforms.items()}


def plan_cartesian_target(
    model: RobotModel,
    target_position_m: Sequence[float],
    initial_joint_angles_deg: Sequence[float],
) -> PlanResult:
    target = np.asarray(target_position_m, dtype=float)
    initial_deg = np.asarray(initial_joint_angles_deg, dtype=float)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("target_position_m 必须包含 3 个有限数值")
    if initial_deg.shape != (5,) or not np.all(np.isfinite(initial_deg)):
        raise ValueError("initial_joint_angles_deg 必须包含 5 个有限数值")
    lower = np.array([joint.lower_rad for joint in model.movable_joints])
    upper = np.array([joint.upper_rad for joint in model.movable_joints])
    initial_rad = np.clip(np.deg2rad(initial_deg), lower, upper)
    candidates = [_solve_from_seed(model, target, seed, initial_rad, 350) for seed in _ik_seeds(model, initial_rad)]
    span = upper - lower
    best_solution, best_error_m, best_iterations = min(
        candidates,
        key=lambda candidate: (
            candidate[1] > POSITION_TOLERANCE_M,
            candidate[1],
            float(np.linalg.norm((candidate[0] - initial_rad) / span)),
        ),
    )
    if best_error_m > POSITION_TOLERANCE_M:
        final_state = forward_kinematics(model, best_solution)
        return PlanResult(
            False,
            f"目标不可达或 IK 未收敛，最小误差 {best_error_m * 1000:.1f} mm。",
            target.tolist(),
            final_state.end_effector_transform[:3, 3].tolist(),
            best_error_m,
            np.rad2deg(best_solution).tolist(),
            [],
            [],
            [],
            best_iterations,
            False,
            None,
            [],
            [],
        )

    evaluated_candidates: list[
        tuple[
            NDArray[np.float64],
            float,
            int,
            NDArray[np.float64],
            list[KinematicState],
            TrajectoryCollision | None,
        ]
    ] = []
    for solution, error_m, iterations in candidates:
        if error_m > POSITION_TOLERANCE_M:
            continue
        trajectory_rad = _smooth_trajectory(initial_rad, solution, 91)
        states = [forward_kinematics(model, frame) for frame in trajectory_rad]
        collision = _first_trajectory_collision(model, trajectory_rad, states)
        evaluated_candidates.append((solution, error_m, iterations, trajectory_rad, states, collision))

    safe_candidates = [candidate for candidate in evaluated_candidates if candidate[5] is None]
    if not safe_candidates:
        collision_candidate = min(
            evaluated_candidates,
            key=lambda candidate: (
                candidate[1],
                float(np.linalg.norm((candidate[0] - initial_rad) / span)),
            ),
        )
        solution, error_m, iterations, _trajectory_rad, states, collision = collision_candidate
        if collision is None:
            raise RuntimeError("碰撞候选缺少碰撞报告")
        final_state = states[-1]
        pair_text = "、".join(f"{first}/{second}" for first, second in collision.link_pairs)
        return PlanResult(
            False,
            f"目标位置可达，但轨迹在第 {collision.frame_index + 1} 帧发生自碰撞：{pair_text}。",
            target.tolist(),
            final_state.end_effector_transform[:3, 3].tolist(),
            error_m,
            np.rad2deg(solution).tolist(),
            [],
            [],
            [],
            iterations,
            False,
            collision.frame_index,
            [list(pair) for pair in collision.link_pairs],
            list(collision.joint_angles_deg),
        )

    preferred_candidates = [
        candidate for candidate in safe_candidates if candidate[1] <= PREFERRED_POSITION_ERROR_M
    ]
    selection_pool = preferred_candidates if preferred_candidates else safe_candidates
    solution, error_m, iterations, trajectory_rad, states, _collision = min(
        selection_pool,
        key=lambda candidate: (
            float(np.linalg.norm((candidate[0] - initial_rad) / span)),
            candidate[1],
        ),
    )
    final_state = states[-1]
    return PlanResult(
        True,
        "规划成功，整段轨迹无自碰撞。",
        target.tolist(),
        final_state.end_effector_transform[:3, 3].tolist(),
        error_m,
        np.rad2deg(solution).tolist(),
        np.rad2deg(trajectory_rad).tolist(),
        [state.points.tolist() for state in states],
        [_link_matrix_payload(state) for state in states],
        iterations,
        True,
        None,
        [],
        [],
    )


def make_model_payload(model: RobotModel) -> dict[str, object]:
    initial_deg = np.asarray(DEFAULT_INITIAL_JOINT_ANGLES_DEG)
    initial_state = forward_kinematics(model, np.deg2rad(initial_deg))
    visuals = {
        link: [
            {
                "mesh_url": f"/model/{visual.mesh_relative_path}",
                "origin_matrix": _matrix_payload(_transform(visual.origin_xyz, _rpy_rotation(visual.origin_rpy))),
                "scale": list(visual.scale),
                "color_rgba": list(visual.color_rgba),
            }
            for visual in link_visuals
        ]
        for link, link_visuals in model.visuals_by_link.items()
    }
    return {
        "model_name": "so101_arm（用户提供，无相机）",
        "model_source": str(model.source_path),
        "root_link": model.root_link,
        "tcp_link": model.tcp_link,
        "link_names": list(model.link_names),
        "joint_names": [joint.name for joint in model.movable_joints],
        "joint_limits_deg": [
            [float(np.rad2deg(joint.lower_rad)), float(np.rad2deg(joint.upper_rad))]
            for joint in model.movable_joints
        ],
        "coordinate_frame": {"origin": "base_footprint", "x": "+X", "y": "+Y", "z": "+Z", "unit": "m"},
        "visuals": visuals,
        "initial_frame": {
            "joint_angles_deg": initial_deg.tolist(),
            "points": initial_state.points.tolist(),
            "link_matrices": _link_matrix_payload(initial_state),
            "end_effector_position_m": initial_state.end_effector_transform[:3, 3].tolist(),
        },
    }
