from __future__ import annotations

import io
import json
import threading
from contextlib import contextmanager
from dataclasses import replace
from http.server import ThreadingHTTPServer
from pathlib import Path
from typing import Iterator
from unittest.mock import patch

import pytest
import yaml

from haiying_zhixun_bridge.config import load_bridge_config
from haiying_zhixun_bridge.contracts import (
    ACCEPTED_FRAME_ID,
    CURRENT_STATE_TOPIC,
    TARGET_POSE_TOPIC,
    MissionState,
    PlanSummary,
    TargetPose,
)
from haiying_zhixun_bridge.coordinator import ArmCoordinator, StateGateError, StalePlanError
from haiying_zhixun_bridge.ik_client import IkClient, parse_plan_payload
from haiying_zhixun_bridge.real_arm import RealArmRequest, build_real_arm_command

try:
    from lerobot.scripts.lerobot_ik_simulator import create_server
    from lerobot.simulators.so101_5dof import DEFAULT_MODEL_FILE, DEFAULT_MODEL_ROOT

    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False


BRIDGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = BRIDGE_ROOT / "config" / "arm_bridge.yaml"


class FakePlanner:
    def __init__(self) -> None:
        self.calls: list[tuple[TargetPose, tuple[float, ...]]] = []

    def plan_target(
        self, target: TargetPose, initial_joint_angles_deg: tuple[float, ...]
    ) -> PlanSummary:
        self.calls.append((target, initial_joint_angles_deg))
        return PlanSummary(
            plan_id="fakeplan001",
            target_position_m=target.position_m,
            reached_position_m=target.position_m,
            error_m=0.0,
            joint_angles_deg=(0.0,) * 5,
            trajectory_frames=91,
            collision_free=True,
        )


def _target() -> TargetPose:
    return TargetPose(ACCEPTED_FRAME_ID, 0.005534, -0.179839, 0.171219, 0.0, 0.0, 0.0, 1.0)


def _plan_payload() -> dict[str, object]:
    trajectory = [[0.0] * 5 for _ in range(91)]
    return {
        "success": True,
        "collision_free": True,
        "plan_id": "plan001",
        "target_position_m": [0.005534, -0.179839, 0.171219],
        "reached_position_m": [0.005534, -0.179839, 0.1711],
        "error_m": 0.000119,
        "joint_angles_deg": [0.0] * 5,
        "trajectory_deg": trajectory,
    }


@contextmanager
def _running_ik_server() -> Iterator[str]:
    if not LEROBOT_AVAILABLE:
        raise RuntimeError("当前 Python 环境没有安装可选的 LeRobot IK 服务")
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


def test_config_preserves_fixed_topics_and_measured_mapping():
    config = load_bridge_config(CONFIG_PATH)

    assert config.ros2.target_pose_topic == TARGET_POSE_TOPIC
    assert config.ros2.current_state_topic == CURRENT_STATE_TOPIC
    assert config.ros2.allowed_state == MissionState.BRUSHING.value
    assert config.geometry.dimensions_mm == (55.0, 135.0, 135.0, 55.0, 10.0)
    assert config.geometry.urdf_source == "package://arm_urdf/urdf/so101_arm.urdf.xacro"
    assert config.mapping.direction_signs == (1.0, 1.0, -1.0, 1.0, 1.0)
    assert config.mapping.zero_offsets_deg == pytest.approx(
        (-6.417582417582418, -0.7472527472527473, -0.5274725274725275, 16.967032967032967, -6.197802197802198)
    )
    assert config.moveit_real.server_url == "http://127.0.0.1:8767"
    assert config.moveit_real.hardware_execution_enabled is True
    assert config.moveit_real.display_trajectory_topic == "/display_planned_path"
    assert config.moveit_real.joint_states_topic == "/joint_states"
    assert config.robot.port == "/dev/ttyACM0"
    assert config.safety.simulation_only


def test_config_rejects_private_topic_change(tmp_path: Path):
    payload = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    payload["ros2"]["target_pose_topic"] = "/private/arm_target"
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(payload, allow_unicode=True), encoding="utf-8")

    with pytest.raises(ValueError, match="target_pose_topic"):
        load_bridge_config(path)


@pytest.mark.parametrize("state", list(MissionState))
def test_mission_state_round_trip(state: MissionState):
    assert MissionState.parse(state.value.lower()) is state


def test_mission_state_rejects_unknown_value():
    with pytest.raises(ValueError, match="未知系统状态"):
        MissionState.parse("FLYING")


def test_target_pose_requires_base_frame_and_valid_quaternion():
    with pytest.raises(ValueError, match="base_footprint"):
        TargetPose("map", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    with pytest.raises(ValueError, match="四元数不能为零"):
        TargetPose(ACCEPTED_FRAME_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="归一化"):
        TargetPose(ACCEPTED_FRAME_ID, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0)


def test_coordinator_accepts_target_only_while_brushing():
    planner = FakePlanner()
    coordinator = ArmCoordinator(planner, (0.0,) * 5)

    with pytest.raises(StateGateError, match="SEARCHING"):
        coordinator.plan_target(_target())

    coordinator.update_state("BRUSHING")
    summary = coordinator.plan_target(_target())

    assert summary.plan_id == "fakeplan001"
    assert len(planner.calls) == 1


def test_coordinator_discards_plan_if_state_changes_while_planning():
    started = threading.Event()
    release = threading.Event()

    class BlockingPlanner(FakePlanner):
        def plan_target(
            self, target: TargetPose, initial_joint_angles_deg: tuple[float, ...]
        ) -> PlanSummary:
            started.set()
            assert release.wait(timeout=2)
            return super().plan_target(target, initial_joint_angles_deg)

    coordinator = ArmCoordinator(BlockingPlanner(), (0.0,) * 5)
    coordinator.update_state("BRUSHING")
    errors: list[BaseException] = []

    def plan() -> None:
        try:
            coordinator.plan_target(_target())
        except BaseException as error:
            errors.append(error)

    thread = threading.Thread(target=plan)
    thread.start()
    assert started.wait(timeout=2)
    coordinator.update_state("RETURNING")
    release.set()
    thread.join(timeout=2)

    assert not thread.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], StalePlanError)


def test_parse_plan_payload_requires_safe_complete_trajectory():
    summary = parse_plan_payload(_plan_payload(), 91, 0.002)

    assert summary.collision_free
    assert summary.trajectory_frames == 91

    collision = _plan_payload()
    collision["collision_free"] = False
    with pytest.raises(ValueError, match="碰撞"):
        parse_plan_payload(collision, 91, 0.002)

    negative_error = _plan_payload()
    negative_error["error_m"] = -0.001
    with pytest.raises(ValueError, match="不能为负数"):
        parse_plan_payload(negative_error, 91, 0.002)

    boolean_joint = _plan_payload()
    boolean_joint["joint_angles_deg"] = [True, 0.0, 0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="非法数值"):
        parse_plan_payload(boolean_joint, 91, 0.002)


def test_ik_client_health_and_plan_use_json_http_boundary():
    config = load_bridge_config(CONFIG_PATH)
    responses = [
        io.BytesIO(json.dumps({"status": "ok", "hardware_connected": False}).encode("utf-8")),
        io.BytesIO(json.dumps(_plan_payload()).encode("utf-8")),
    ]
    client = IkClient(config.ik)

    with patch("haiying_zhixun_bridge.ik_client.urlopen", side_effect=responses) as urlopen:
        health = client.health()
        summary = client.plan_target(_target(), (0.0,) * 5)

    assert health["hardware_connected"] is False
    assert summary.plan_id == "plan001"
    assert urlopen.call_count == 2


def test_ik_client_rejects_response_for_a_different_target():
    config = load_bridge_config(CONFIG_PATH)
    payload = _plan_payload()
    payload["target_position_m"] = [0.1, 0.2, 0.3]
    client = IkClient(config.ik)

    with patch(
        "haiying_zhixun_bridge.ik_client.urlopen",
        return_value=io.BytesIO(json.dumps(payload).encode("utf-8")),
    ):
        with pytest.raises(ValueError, match="与请求目标不一致"):
            client.plan_target(_target(), (0.0,) * 5)


@pytest.mark.skipif(not LEROBOT_AVAILABLE, reason="可选 LeRobot IK 包未安装")
def test_bridge_integrates_with_real_local_ik_server():
    config = load_bridge_config(CONFIG_PATH)
    with _running_ik_server() as server_url:
        client = IkClient(replace(config.ik, server_url=server_url))
        assert client.health()["status"] == "ok"
        summary = client.plan_target(_target(), (0.0,) * 5)

    assert summary.collision_free
    assert summary.trajectory_frames == 91
    assert max(abs(angle) for angle in summary.joint_angles_deg) <= 10.0


def test_real_arm_command_wraps_existing_cli_without_shell_text():
    config = load_bridge_config(CONFIG_PATH)
    request = RealArmRequest("jog", "shoulder_pan", 5.0, None, False)

    command = build_real_arm_command("lerobot-ik-real", config.robot, config.safety, request)

    assert command == (
        "lerobot-ik-real",
        "--mode=jog",
        "--port=/dev/ttyACM0",
        "--robot_id=jiebang_follower_arm",
        "--joint=shoulder_pan",
        "--delta_deg=5.0",
    )


def test_real_arm_execute_requires_explicit_confirmation():
    config = load_bridge_config(CONFIG_PATH)
    request = RealArmRequest("execute", None, None, "abc123", False)

    with pytest.raises(ValueError, match="显式确认"):
        build_real_arm_command("lerobot-ik-real", config.robot, config.safety, request)

    confirmed = RealArmRequest("execute", None, None, "abc123", True)
    command = build_real_arm_command("lerobot-ik-real", config.robot, config.safety, confirmed)
    assert command[-1] == "--confirm_execute=true"


def test_real_arm_jog_uses_configured_limit():
    config = load_bridge_config(CONFIG_PATH)
    safety = replace(config.safety, jog_max_delta_deg=2.0)
    request = RealArmRequest("jog", "shoulder_pan", 2.1, None, False)

    with pytest.raises(ValueError, match="不超过 2°"):
        build_real_arm_command("lerobot-ik-real", config.robot, safety, request)


def test_real_arm_rejects_unknown_runtime_action():
    config = load_bridge_config(CONFIG_PATH)
    request = RealArmRequest("stop", None, None, None, False)  # type: ignore[arg-type]

    with pytest.raises(ValueError, match="未知实机控制动作"):
        build_real_arm_command("lerobot-ik-real", config.robot, config.safety, request)
