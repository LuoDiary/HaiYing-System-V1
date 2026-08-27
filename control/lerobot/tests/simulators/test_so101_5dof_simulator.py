import json
import threading
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from typing import Iterator
from urllib.request import Request, urlopen

import numpy as np
import pytest

from lerobot.scripts.lerobot_ik_simulator import create_server
from lerobot.simulators.so101_5dof import (
    DEFAULT_MODEL_FILE,
    DEFAULT_MODEL_ROOT,
    find_self_collisions,
    forward_kinematics,
    load_robot_model,
    make_model_payload,
    plan_cartesian_target,
)


MODEL = load_robot_model(DEFAULT_MODEL_ROOT, DEFAULT_MODEL_FILE)


def test_forward_kinematics_returns_full_five_joint_chain():
    state = forward_kinematics(MODEL, np.zeros(5, dtype=float))

    assert state.end_effector_transform.shape == (4, 4)
    assert state.points.shape == (7, 3)
    np.testing.assert_allclose(state.end_effector_transform[3], [0.0, 0.0, 0.0, 1.0])


def test_plan_reaches_position_generated_by_forward_kinematics():
    expected_joints_rad = np.deg2rad([10.0, -15.0, 20.0, -10.0, 5.0])
    target = forward_kinematics(MODEL, expected_joints_rad).end_effector_transform[:3, 3]

    result = plan_cartesian_target(MODEL, target, [0.0] * 5)

    assert result.success, result.message
    assert result.error_m <= 0.002
    assert len(result.trajectory_deg) == 91
    assert result.collision_free
    np.testing.assert_allclose(result.trajectory_deg[0], np.zeros(5), atol=1e-9)
    np.testing.assert_allclose(result.trajectory_deg[-1], result.joint_angles_deg, atol=1e-9)
    assert all(
        not find_self_collisions(MODEL, forward_kinematics(MODEL, np.deg2rad(frame)))
        for frame in result.trajectory_deg
    )


def test_nearby_positive_z_target_prefers_near_pose_solution():
    initial_deg = np.zeros(5, dtype=float)
    start = forward_kinematics(MODEL, np.deg2rad(initial_deg)).end_effector_transform[:3, 3]
    target = start + np.array([0.0, 0.0, 0.01])

    result = plan_cartesian_target(MODEL, target, initial_deg)

    assert result.success, result.message
    assert result.collision_free
    assert result.error_m <= 0.00025
    assert max(abs(angle) for angle in result.joint_angles_deg) <= 10.0
    np.testing.assert_allclose(result.trajectory_deg[0], initial_deg, atol=1e-9)


def test_known_overlapping_pose_detects_base_collisions():
    state = forward_kinematics(MODEL, np.deg2rad([37.65, -48.31, 60.95, -2.70, 0.0]))

    collisions = set(find_self_collisions(MODEL, state))

    assert ("base_link", "arm_upper") in collisions
    assert ("base_link", "arm_lower") in collisions


def test_known_problem_target_never_returns_colliding_trajectory():
    result = plan_cartesian_target(MODEL, [0.09, 0.105, 0.29], [0.0] * 5)

    if result.success:
        assert result.collision_free
        assert all(
            not find_self_collisions(MODEL, forward_kinematics(MODEL, np.deg2rad(frame)))
            for frame in result.trajectory_deg
        )
    else:
        assert "碰撞" in result.message
        assert not result.collision_free
        assert result.first_collision_frame is not None
        assert result.collision_link_pairs


@pytest.mark.parametrize(
    "source_joint_angles_deg",
    [
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [10.0, -15.0, 20.0, -10.0, 5.0],
        [-8.0, 5.0, 4.0, -4.0, 0.0],
    ],
)
def test_workspace_samples_only_return_collision_free_successes(source_joint_angles_deg: list[float]):
    target = forward_kinematics(MODEL, np.deg2rad(source_joint_angles_deg)).end_effector_transform[:3, 3]

    result = plan_cartesian_target(MODEL, target, [0.0] * 5)

    assert result.success, result.message
    assert result.collision_free
    for frame in result.trajectory_deg:
        assert not find_self_collisions(MODEL, forward_kinematics(MODEL, np.deg2rad(frame)))


def test_plan_rejects_unreachable_target():
    result = plan_cartesian_target(MODEL, [2.0, 0.0, 0.0], [0.0] * 5)

    assert not result.success
    assert not result.collision_free
    assert result.error_m > 0.002
    assert result.trajectory_deg == []


@pytest.mark.parametrize(
    "target",
    [
        [float("nan"), 0.0, 0.1],
        [0.1, float("inf"), 0.1],
        [0.1, 0.2],
    ],
)
def test_plan_rejects_invalid_target(target: list[float]):
    with pytest.raises(ValueError, match="target"):
        plan_cartesian_target(MODEL, target, [0.0] * 5)


def test_model_payload_exposes_joint_names_limits_and_initial_frame():
    payload = make_model_payload(MODEL)

    assert payload["joint_names"] == [
        "J1_Rotation",
        "J2_Shoulder_Pitch",
        "J3_Elbow_Pitch",
        "J4_Wrist_Pitch",
        "J5_Wrist_Roll",
    ]
    assert len(payload["joint_limits_deg"]) == 5
    assert len(payload["initial_frame"]["points"]) == 7
    assert len(payload["initial_frame"]["joint_angles_deg"]) == 5
    assert payload["model_source"].endswith("so101_arm.urdf.xacro")
    assert sum(len(items) for items in payload["visuals"].values()) == 12


@contextmanager
def running_server() -> Iterator[str]:
    server: ThreadingHTTPServer = create_server("127.0.0.1", 0, DEFAULT_MODEL_ROOT, DEFAULT_MODEL_FILE)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address
        yield f"http://{host}:{port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_http_server_serves_threejs_ui_and_hardware_free_health():
    with running_server() as base_url:
        with urlopen(f"{base_url}/", timeout=3) as response:
            html = response.read().decode("utf-8")
        with urlopen(f"{base_url}/static/three/three.module.js", timeout=3) as response:
            three_module = response.read().decode("utf-8")
            three_content_type = response.headers.get_content_type()
        with urlopen(f"{base_url}/static/three/three.core.js", timeout=3) as response:
            three_core = response.read().decode("utf-8")
        with urlopen(
            f"{base_url}/static/three/addons/loaders/STLLoader.js", timeout=3
        ) as response:
            stl_loader = response.read().decode("utf-8")
        with urlopen(f"{base_url}/api/health", timeout=3) as response:
            health = json.load(response)

    assert '"three": "/static/three/three.module.js"' in html
    assert "cdn.jsdelivr.net" not in html
    assert "stlLoader.parse(await response.arrayBuffer())" in html
    assert "stlLoader.load(" not in html
    assert "WebGLRenderer" in html
    assert "WebGLRenderer" in three_module
    assert "const REVISION = '185'" in three_core
    assert three_content_type == "text/javascript"
    assert "class STLLoader" in stl_loader
    assert health == {"status": "ok", "hardware_connected": False}


def test_http_plan_endpoint_returns_trajectory():
    target = forward_kinematics(MODEL, np.deg2rad([10.0, -15.0, 20.0, -10.0, 5.0]))
    payload = json.dumps(
        {
            "target_position_m": target.end_effector_transform[:3, 3].tolist(),
            "initial_joint_angles_deg": [0.0] * 5,
        }
    ).encode("utf-8")

    with running_server() as base_url:
        request = Request(
            f"{base_url}/api/plan",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request, timeout=10) as response:
            result = json.load(response)

    assert result["success"]
    assert result["collision_free"]
    assert result["error_m"] <= 0.002
    assert len(result["plan_id"]) == 32
    assert len(result["trajectory_points_m"]) == 91
    assert len(result["trajectory_link_matrices"]) == 91


def test_http_plan_endpoint_stores_retrievable_plan():
    target = forward_kinematics(MODEL, np.deg2rad([10.0, -15.0, 20.0, -10.0, 5.0]))
    payload = json.dumps(
        {
            "target_position_m": target.end_effector_transform[:3, 3].tolist(),
            "initial_joint_angles_deg": [0.0] * 5,
        }
    ).encode("utf-8")

    with running_server() as base_url:
        request = Request(
            f"{base_url}/api/plan",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request, timeout=10) as response:
            planned = json.load(response)
        with urlopen(f"{base_url}/api/plans/{planned['plan_id']}", timeout=3) as response:
            fetched = json.load(response)

    assert fetched == planned


def test_http_server_serves_original_stl_mesh():
    with running_server() as base_url:
        with urlopen(
            f"{base_url}/model/meshes/body/part_001_Wiring_holder_v1.stl", timeout=3
        ) as response:
            body = response.read()

    assert len(body) > 84
