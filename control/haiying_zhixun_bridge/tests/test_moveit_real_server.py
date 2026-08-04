import json
import threading
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from unittest.mock import MagicMock, patch

import pytest

from haiying_zhixun_bridge.lerobot_adapter import (
    JOINT_NAMES,
    URDF_JOINT_NAMES,
    map_urdf_angles_to_real,
)
from haiying_zhixun_bridge.moveit_real_server import (
    MoveItRealServerConfig,
    create_server,
    execute_moveit_trajectory,
    validate_moveit_trajectory,
)


def _payload(target_deg: tuple[float, ...] = (4.0, -3.0, 2.0, -1.0, 0.5)) -> dict[str, object]:
    return {
        "joint_names": list(URDF_JOINT_NAMES),
        "points": [
            {"positions_rad": [0.0] * 5, "time_from_start_s": 0.0},
            {
                "positions_rad": [value * 3.141592653589793 / 180.0 for value in target_deg],
                "time_from_start_s": 2.0,
            },
        ],
    }


def _calibration(directory: Path, robot_id: str = "test_arm") -> None:
    directory.mkdir(parents=True)
    payload = {
        name: {"id": index}
        for index, name in enumerate(JOINT_NAMES, start=1)
    }
    (directory / f"{robot_id}.json").write_text(json.dumps(payload), encoding="utf-8")


def _observation(positions: dict[str, float]) -> dict[str, float]:
    return {f"{name}.pos": value for name, value in positions.items()}


def test_validate_moveit_trajectory_resamples_and_hashes_deterministically():
    config = MoveItRealServerConfig(port=0, execution_rate_hz=20.0)

    first = validate_moveit_trajectory(_payload(), config)
    second = validate_moveit_trajectory(_payload(), config)

    assert first.trajectory_id == second.trajectory_id
    assert len(first.trajectory_id) == 64
    assert first.joint_names == URDF_JOINT_NAMES
    assert first.resampled_times_s[0] == 0.0
    assert first.resampled_times_s[-1] == 2.0
    assert len(first.resampled_positions_deg) == 41
    assert first.target_positions_deg == pytest.approx((4.0, -3.0, 2.0, -1.0, 0.5))
    assert first.maximum_frame_step_deg <= config.maximum_frame_step_deg


def test_validate_moveit_trajectory_reorders_joint_names():
    payload = _payload()
    payload["joint_names"] = list(reversed(URDF_JOINT_NAMES))
    points = payload["points"]
    assert isinstance(points, list)
    for point in points:
        assert isinstance(point, dict)
        point["positions_rad"] = list(reversed(point["positions_rad"]))

    plan = validate_moveit_trajectory(payload, MoveItRealServerConfig(port=0))

    assert plan.target_positions_deg == pytest.approx((4.0, -3.0, 2.0, -1.0, 0.5))


def test_validate_moveit_trajectory_requires_start_state_at_zero_time():
    payload = _payload()
    points = payload["points"]
    assert isinstance(points, list) and isinstance(points[0], dict)
    points[0]["time_from_start_s"] = 0.1

    with pytest.raises(ValueError, match="真实起始状态"):
        validate_moveit_trajectory(payload, MoveItRealServerConfig(port=0))


def test_validate_moveit_trajectory_rejects_speed_and_joint_limit():
    with pytest.raises(ValueError, match="速度"):
        validate_moveit_trajectory(
            _payload((80.0, 0.0, 0.0, 0.0, 0.0)),
            MoveItRealServerConfig(port=0, maximum_joint_speed_deg_s=20.0),
        )
    with pytest.raises(ValueError, match="限位"):
        validate_moveit_trajectory(
            _payload((90.0, 0.0, 0.0, 0.0, 0.0)),
            MoveItRealServerConfig(port=0),
        )


def test_execute_moveit_trajectory_checks_start_then_tracks_feedback():
    config = MoveItRealServerConfig(port=0, execution_rate_hz=10.0)
    plan = validate_moveit_trajectory(_payload(), config)
    real_frames = [map_urdf_angles_to_real(frame) for frame in plan.resampled_positions_deg]
    robot = MagicMock()
    robot.get_observation.side_effect = [_observation(frame) for frame in real_frames]
    robot.send_action.side_effect = lambda action: action

    with patch("haiying_zhixun_bridge.moveit_real_server.time.sleep"):
        result = execute_moveit_trajectory(robot, plan, config)

    assert result.execution_completed
    assert result.commanded_frames == len(real_frames) - 1
    assert robot.send_action.call_count == len(real_frames) - 1


def test_execute_moveit_trajectory_rejects_start_mismatch_without_motion():
    config = MoveItRealServerConfig(port=0)
    plan = validate_moveit_trajectory(_payload(), config)
    initial = map_urdf_angles_to_real(plan.start_positions_deg)
    initial["shoulder_pan"] += 10.1
    robot = MagicMock()
    robot.get_observation.return_value = _observation(initial)

    with pytest.raises(RuntimeError, match="首帧不匹配"):
        execute_moveit_trajectory(robot, plan, config)

    robot.send_action.assert_not_called()


def test_execute_moveit_trajectory_safely_aligns_start_within_ten_degrees():
    config = MoveItRealServerConfig(port=0, execution_rate_hz=20.0)
    plan = validate_moveit_trajectory(_payload(), config)
    planned_start = map_urdf_angles_to_real(plan.start_positions_deg)
    state = dict(planned_start)
    state["shoulder_lift"] += 3.61
    robot = MagicMock()
    robot.get_observation.side_effect = lambda: _observation(state)

    def send_action(action: dict[str, float]) -> dict[str, float]:
        for name in JOINT_NAMES:
            state[name] = action[f"{name}.pos"]
        return action

    robot.send_action.side_effect = send_action
    with patch("haiying_zhixun_bridge.moveit_real_server.time.sleep"):
        result = execute_moveit_trajectory(robot, plan, config)

    normal_frames = len(plan.resampled_positions_deg) - 1
    assert result.commanded_frames == normal_frames + 3
    assert result.duration_s == pytest.approx(plan.duration_s + 3 / config.execution_rate_hz)
    first_action = robot.send_action.call_args_list[0].args[0]
    assert abs(first_action["shoulder_lift.pos"] - (planned_start["shoulder_lift"] + 3.61)) <= 1.5
    assert result.execution_completed


def test_execute_moveit_trajectory_accepts_small_relative_target_clipping():
    config = MoveItRealServerConfig(port=0)
    plan = validate_moveit_trajectory(_payload(), config)
    state = map_urdf_angles_to_real(plan.start_positions_deg)
    robot = MagicMock()
    robot.get_observation.side_effect = lambda: _observation(state)

    def send_action(action: dict[str, float]) -> dict[str, float]:
        sent = dict(action)
        sent["elbow_flex.pos"] -= 0.052
        for name in JOINT_NAMES:
            state[name] = sent[f"{name}.pos"]
        return sent

    robot.send_action.side_effect = send_action
    with patch("haiying_zhixun_bridge.moveit_real_server.time.sleep"):
        result = execute_moveit_trajectory(robot, plan, config)

    assert result.execution_completed


def test_start_error_safety_limit_cannot_exceed_ten_degrees():
    with pytest.raises(ValueError, match="不得超过 10°"):
        MoveItRealServerConfig(port=0, maximum_start_error_deg=10.1)


def test_command_clip_safety_limit_cannot_exceed_half_degree():
    with pytest.raises(ValueError, match="不得超过 0.5°"):
        MoveItRealServerConfig(port=0, maximum_command_clip_deg=0.51)


@contextmanager
def _server(tmp_path: Path) -> Iterator[str]:
    calibration_dir = tmp_path / "calibration"
    _calibration(calibration_dir)
    config = MoveItRealServerConfig(
        port=0,
        robot_id="test_arm",
        calibration_dir=calibration_dir,
    )
    server = create_server(config)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address
        yield f"http://{host}:{port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_http_health_and_validate_are_hardware_free(tmp_path: Path):
    with _server(tmp_path) as base_url:
        with urlopen(f"{base_url}/api/health") as response:
            health = json.load(response)
        request = Request(
            f"{base_url}/api/validate",
            data=json.dumps(_payload()).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request) as response:
            validated = json.load(response)

    assert health["calibration_valid"]
    assert not health["hardware_connected"]
    assert validated["validated"]
    assert not validated["hardware_connected"]


def test_http_execute_requires_explicit_confirmation(tmp_path: Path):
    with _server(tmp_path) as base_url:
        validate_request = Request(
            f"{base_url}/api/validate",
            data=json.dumps(_payload()).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(validate_request) as response:
            trajectory_id = json.load(response)["trajectory_id"]
        execute_request = Request(
            f"{base_url}/api/execute",
            data=json.dumps({"trajectory_id": trajectory_id}).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with pytest.raises(HTTPError) as error:
            urlopen(execute_request)

    assert error.value.code == 400


def test_http_execute_rejects_when_hardware_execution_is_disabled(tmp_path: Path):
    with _server(tmp_path) as base_url:
        validate_request = Request(
            f"{base_url}/api/validate",
            data=json.dumps(_payload()).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(validate_request) as response:
            trajectory_id = json.load(response)["trajectory_id"]
        execute_request = Request(
            f"{base_url}/api/execute",
            data=json.dumps(
                {"trajectory_id": trajectory_id, "confirm_execute": True}
            ).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with patch("haiying_zhixun_bridge.moveit_real_server._make_robot") as make_robot:
            with pytest.raises(HTTPError) as error:
                urlopen(execute_request)

    assert error.value.code == 400
    make_robot.assert_not_called()
