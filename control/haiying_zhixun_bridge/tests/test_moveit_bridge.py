import math

import pytest

from haiying_zhixun_bridge.moveit_bridge import (
    URDF_JOINT_NAMES,
    MoveItRealClient,
    build_snapshot,
    simulation_endpoint_error_deg,
)


def test_build_snapshot_prepends_moveit_trajectory_start():
    snapshot = build_snapshot(
        list(URDF_JOINT_NAMES),
        [[0.1, -0.2, 0.3, -0.1, 0.0], [0.2, -0.3, 0.4, -0.2, 0.1]],
        [1.0, 2.0],
        list(URDF_JOINT_NAMES),
        [0.0] * 5,
    )

    assert snapshot.times_s == (0.0, 1.0, 2.0)
    assert snapshot.start_positions_rad == (0.0,) * 5
    assert snapshot.target_positions_rad == pytest.approx((0.2, -0.3, 0.4, -0.2, 0.1))
    assert snapshot.to_payload()["joint_names"] == list(URDF_JOINT_NAMES)


def test_build_snapshot_reorders_moveit_joint_order():
    reversed_names = list(reversed(URDF_JOINT_NAMES))
    snapshot = build_snapshot(
        reversed_names,
        [[0.5, 0.4, 0.3, 0.2, 0.1]],
        [1.0],
        reversed_names,
        [0.0] * 5,
    )

    assert snapshot.target_positions_rad == pytest.approx((0.1, 0.2, 0.3, 0.4, 0.5))


def test_build_snapshot_rejects_zero_time_point_that_disagrees_with_start():
    with pytest.raises(ValueError, match="trajectory_start"):
        build_snapshot(
            list(URDF_JOINT_NAMES),
            [[math.radians(1.0), 0.0, 0.0, 0.0, 0.0], [0.0] * 5],
            [0.0, 1.0],
            list(URDF_JOINT_NAMES),
            [0.0] * 5,
        )


def test_simulation_endpoint_error_uses_canonical_joint_order():
    snapshot = build_snapshot(
        list(URDF_JOINT_NAMES),
        [[math.radians(value) for value in (1.0, 2.0, 3.0, 4.0, 5.0)]],
        [1.0],
        list(URDF_JOINT_NAMES),
        [0.0] * 5,
    )
    reversed_names = list(reversed(URDF_JOINT_NAMES))
    reversed_positions = list(reversed(snapshot.target_positions_rad))

    assert simulation_endpoint_error_deg(snapshot, reversed_names, reversed_positions) == pytest.approx(0.0)
    reversed_positions[0] += math.radians(1.25)
    assert simulation_endpoint_error_deg(snapshot, reversed_names, reversed_positions) == pytest.approx(1.25)


def test_moveit_real_client_rejects_nonlocal_server():
    with pytest.raises(ValueError, match="本机"):
        MoveItRealClient("http://192.168.1.20:8767")
