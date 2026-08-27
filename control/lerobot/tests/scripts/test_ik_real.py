from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from lerobot.scripts.lerobot_ik_real import (
    EXECUTE_RATE_HZ,
    EXPECTED_TRAJECTORY_FRAMES,
    JOINT_OFFSETS_DEG,
    ExecutePlan,
    IKRealConfig,
    JOINT_NAMES,
    execute_plan,
    inspect_robot,
    jog_robot,
    map_urdf_angles_to_real,
    run_dry_run,
    run_execute,
    validate_calibration_file,
    validate_plan_for_execute,
    validate_plan_for_dry_run,
)


def _positions(offset: float) -> dict[str, float]:
    return {f"{name}.pos": float(index * 10) + offset for index, name in enumerate(JOINT_NAMES)}


def _safe_plan_payload() -> dict[str, object]:
    trajectory = [
        [frame_index / (EXPECTED_TRAJECTORY_FRAMES - 1) * (joint_index + 1) for joint_index in range(5)]
        for frame_index in range(EXPECTED_TRAJECTORY_FRAMES)
    ]
    return {
        "success": True,
        "collision_free": True,
        "message": "规划成功",
        "error_m": 0.0001,
        "joint_angles_deg": trajectory[-1],
        "trajectory_deg": trajectory,
    }


def _execute_plan_payload() -> dict[str, object]:
    final_angles = (-0.15, 0.81, 0.94, -4.07, 0.0)
    trajectory = [
        [frame_index / (EXPECTED_TRAJECTORY_FRAMES - 1) * angle for angle in final_angles]
        for frame_index in range(EXPECTED_TRAJECTORY_FRAMES)
    ]
    tcp_path = [
        [[0.005534, -0.179839, 0.161219 + frame_index / (EXPECTED_TRAJECTORY_FRAMES - 1) * 0.01]]
        for frame_index in range(EXPECTED_TRAJECTORY_FRAMES)
    ]
    return {
        "plan_id": "plan123",
        "success": True,
        "collision_free": True,
        "message": "规划成功",
        "target_position_m": tcp_path[-1][-1],
        "error_m": 0.00018,
        "joint_angles_deg": trajectory[-1],
        "trajectory_deg": trajectory,
        "trajectory_points_m": tcp_path,
    }


def _observation(positions: dict[str, float]) -> dict[str, float]:
    return {f"{name}.pos": value for name, value in positions.items()}


def test_validate_calibration_file_requires_exact_ids(tmp_path: Path):
    path = tmp_path / "calibration.json"
    path.write_text(
        "{" + ",".join(f'\"{name}\":{{\"id\":{index}}}' for index, name in enumerate(JOINT_NAMES, 1)) + "}",
        encoding="utf-8",
    )

    assert validate_calibration_file(path) == dict(zip(JOINT_NAMES, range(1, 6), strict=True))


@pytest.mark.parametrize("delta", [-10.01, 0.0, 10.01])
def test_jog_config_rejects_zero_or_excessive_delta(delta: float):
    with pytest.raises(ValueError):
        IKRealConfig(mode="jog", joint="shoulder_pan", delta_deg=delta)


@pytest.mark.parametrize("delta", [-10.0, 5.0, 10.0])
def test_jog_config_accepts_delta_up_to_ten_degrees(delta: float):
    cfg = IKRealConfig(mode="jog", joint="shoulder_pan", delta_deg=delta)

    assert cfg.delta_deg == delta


def test_execute_config_requires_plan_id_and_explicit_confirmation():
    with pytest.raises(ValueError, match="plan_id"):
        IKRealConfig(mode="execute", confirm_execute=True)
    with pytest.raises(ValueError, match="confirm_execute"):
        IKRealConfig(mode="execute", plan_id="plan123")

    cfg = IKRealConfig(mode="execute", plan_id="plan123", confirm_execute=True)

    assert cfg.plan_id == "plan123"


def test_inspect_only_reads_observation():
    robot = MagicMock()
    robot.get_observation.return_value = _positions(0.0)

    result = inspect_robot(robot)

    assert result == {name: float(index * 10) for index, name in enumerate(JOINT_NAMES)}
    robot.get_observation.assert_called_once_with()
    robot.send_action.assert_not_called()


def test_jog_changes_only_selected_joint_and_reads_feedback():
    robot = MagicMock()
    before = _positions(0.0)
    feedback = _positions(0.0)
    feedback["elbow_flex.pos"] += 10.0
    robot.get_observation.side_effect = [before, feedback]
    robot.send_action.side_effect = lambda action: action

    result = jog_robot(robot, "elbow_flex", 10.0, 0.0)

    sent = robot.send_action.call_args.args[0]
    assert sent["elbow_flex.pos"] == before["elbow_flex.pos"] + 10.0
    assert all(
        sent[f"{name}.pos"] == before[f"{name}.pos"] for name in JOINT_NAMES if name != "elbow_flex"
    )
    assert result.feedback_deg["elbow_flex"] == feedback["elbow_flex.pos"]


def test_validate_plan_for_dry_run_accepts_safe_91_frame_plan():
    result = validate_plan_for_dry_run(_safe_plan_payload(), (0.1, 0.0, 0.2), 1.0)

    assert result.collision_free
    assert result.cartesian_execution_locked
    assert result.trajectory_frames == EXPECTED_TRAJECTORY_FRAMES
    assert result.maximum_frame_step_deg <= 1.0


def test_validate_plan_for_dry_run_rejects_collision():
    payload = _safe_plan_payload()
    payload["collision_free"] = False

    with pytest.raises(ValueError, match="碰撞"):
        validate_plan_for_dry_run(payload, (0.1, 0.0, 0.2), 1.0)


def test_validate_plan_for_dry_run_rejects_large_frame_step():
    payload = _safe_plan_payload()
    trajectory = payload["trajectory_deg"]
    assert isinstance(trajectory, list)
    trajectory[45] = [20.0] * 5

    with pytest.raises(ValueError, match="单帧"):
        validate_plan_for_dry_run(payload, (0.1, 0.0, 0.2), 1.0)


def test_map_urdf_angles_to_real_uses_measured_offsets():
    mapped = map_urdf_angles_to_real((0.0, 0.0, 0.0, 0.0, 0.0))

    assert tuple(mapped.values()) == JOINT_OFFSETS_DEG


def test_validate_execute_plan_accepts_positive_z_ten_mm_near_pose_plan():
    plan = validate_plan_for_execute(_execute_plan_payload(), "plan123")

    assert plan.plan_id == "plan123"
    assert plan.tcp_end_m[2] - plan.tcp_start_m[2] == pytest.approx(0.01)


def test_validate_execute_plan_rejects_wrong_motion_direction():
    payload = _execute_plan_payload()
    points = payload["trajectory_points_m"]
    assert isinstance(points, list)
    points[-1] = [[0.005534, -0.179839, 0.151219]]

    with pytest.raises(ValueError, match="Z 位移"):
        validate_plan_for_execute(payload, "plan123")


def test_validate_execute_plan_rejects_nonzero_initial_pose():
    payload = _execute_plan_payload()
    trajectory = payload["trajectory_deg"]
    assert isinstance(trajectory, list)
    trajectory[0] = [0.2, 0.0, 0.0, 0.0, 0.0]

    with pytest.raises(ValueError, match="全零参考姿态"):
        validate_plan_for_execute(payload, "plan123")


def test_validate_execute_plan_rejects_excessive_position_error():
    payload = _execute_plan_payload()
    payload["error_m"] = 0.000251

    with pytest.raises(ValueError, match="位置误差"):
        validate_plan_for_execute(payload, "plan123")


def test_execute_plan_sends_frames_and_monitors_feedback():
    payload = _execute_plan_payload()
    plan = validate_plan_for_execute(payload, "plan123")
    real_frames = [map_urdf_angles_to_real(frame) for frame in plan.urdf_trajectory_deg]
    robot = MagicMock()
    robot.get_observation.side_effect = [_observation(frame) for frame in real_frames]
    robot.send_action.side_effect = lambda action: action

    with patch("lerobot.scripts.lerobot_ik_real.time.sleep") as sleep:
        result = execute_plan(robot, plan, EXECUTE_RATE_HZ)

    assert result.execution_completed
    assert result.commanded_frames == EXPECTED_TRAJECTORY_FRAMES - 1
    assert robot.send_action.call_count == EXPECTED_TRAJECTORY_FRAMES - 1
    assert robot.get_observation.call_count == EXPECTED_TRAJECTORY_FRAMES
    assert sleep.call_count == EXPECTED_TRAJECTORY_FRAMES - 1


def test_execute_plan_rejects_initial_mismatch_without_sending():
    plan = validate_plan_for_execute(_execute_plan_payload(), "plan123")
    robot = MagicMock()
    initial = map_urdf_angles_to_real(plan.urdf_trajectory_deg[0])
    initial["shoulder_pan"] += 2.1
    robot.get_observation.return_value = _observation(initial)

    with pytest.raises(RuntimeError, match="首帧不匹配"):
        execute_plan(robot, plan, EXECUTE_RATE_HZ)

    robot.send_action.assert_not_called()


def test_execute_plan_aborts_on_feedback_error():
    plan = validate_plan_for_execute(_execute_plan_payload(), "plan123")
    initial = map_urdf_angles_to_real(plan.urdf_trajectory_deg[0])
    first_target = map_urdf_angles_to_real(plan.urdf_trajectory_deg[1])
    bad_feedback = dict(first_target)
    bad_feedback["elbow_flex"] += 3.1
    robot = MagicMock()
    robot.get_observation.side_effect = [_observation(initial), _observation(bad_feedback)]
    robot.send_action.side_effect = lambda action: action

    with (
        patch("lerobot.scripts.lerobot_ik_real.time.sleep"),
        pytest.raises(RuntimeError, match="第 2/91 帧中止"),
    ):
        execute_plan(robot, plan, EXECUTE_RATE_HZ)

    robot.send_action.assert_called_once()


def test_execute_plan_aborts_if_robot_clips_command():
    plan = validate_plan_for_execute(_execute_plan_payload(), "plan123")
    initial = map_urdf_angles_to_real(plan.urdf_trajectory_deg[0])
    robot = MagicMock()
    robot.get_observation.return_value = _observation(initial)

    def clipped_action(action: dict[str, float]) -> dict[str, float]:
        clipped = dict(action)
        clipped["wrist_flex.pos"] -= 0.1
        return clipped

    robot.send_action.side_effect = clipped_action

    with pytest.raises(RuntimeError, match="命令被限幅"):
        execute_plan(robot, plan, EXECUTE_RATE_HZ)

    robot.send_action.assert_called_once()


def test_dry_run_requests_plan_without_constructing_robot():
    cfg = IKRealConfig(mode="dry-run", target_x=0.1, target_y=0.0, target_z=0.2)

    with (
        patch("lerobot.scripts.lerobot_ik_real._request_plan", return_value=_safe_plan_payload()),
        patch("lerobot.scripts.lerobot_ik_real._make_robot") as make_robot,
    ):
        result = run_dry_run(cfg)

    assert result.cartesian_execution_locked
    make_robot.assert_not_called()


def test_dry_run_fetches_saved_browser_plan():
    cfg = IKRealConfig(mode="dry-run", plan_id="abc123")
    payload = _safe_plan_payload()
    payload["target_position_m"] = [0.03, -0.18, 0.176]

    with (
        patch("lerobot.scripts.lerobot_ik_real._fetch_plan", return_value=payload) as fetch_plan,
        patch("lerobot.scripts.lerobot_ik_real._request_plan") as request_plan,
        patch("lerobot.scripts.lerobot_ik_real._make_robot") as make_robot,
    ):
        result = run_dry_run(cfg)

    assert result.target_position_m == (0.03, -0.18, 0.176)
    fetch_plan.assert_called_once_with(cfg.server_url, "abc123")
    request_plan.assert_not_called()
    make_robot.assert_not_called()


def test_run_execute_validates_plan_before_constructing_robot():
    cfg = IKRealConfig(mode="execute", plan_id="plan123", confirm_execute=True)
    payload = _execute_plan_payload()
    points = payload["trajectory_points_m"]
    assert isinstance(points, list)
    points[-1] = [[0.005534, -0.179839, 0.181219]]

    with (
        patch("lerobot.scripts.lerobot_ik_real._fetch_plan", return_value=payload),
        patch("lerobot.scripts.lerobot_ik_real._make_robot") as make_robot,
        pytest.raises(ValueError, match="Z 位移"),
    ):
        run_execute(cfg)

    make_robot.assert_not_called()


def test_run_execute_disconnects_after_execution_error():
    cfg = IKRealConfig(mode="execute", plan_id="plan123", confirm_execute=True)
    robot = MagicMock()
    robot.is_connected = True

    with (
        patch("lerobot.scripts.lerobot_ik_real._fetch_plan", return_value=_execute_plan_payload()),
        patch("lerobot.scripts.lerobot_ik_real._make_robot", return_value=robot),
        patch("lerobot.scripts.lerobot_ik_real.execute_plan", side_effect=RuntimeError("反馈超差")),
        pytest.raises(RuntimeError, match="反馈超差"),
    ):
        run_execute(cfg)

    robot.connect.assert_called_once_with(calibrate=False)
    robot.disconnect.assert_called_once_with()


def test_server_url_must_be_localhost():
    with pytest.raises(ValueError, match="本机"):
        IKRealConfig(mode="dry-run", server_url="http://example.com:8766")
