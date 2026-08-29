#!/usr/bin/env python3

import math

import pytest

try:
    import rclpy
    from std_msgs.msg import Float32MultiArray
    from geometry_msgs.msg import Twist
    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False

from attitude_cmd.cmd_vel_to_attitude import CmdVelToAttitude, _finite_cmd, _finite_vel


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def advance(self, dt):
        self.t += dt

    def __call__(self):
        return self.t


def _twist(vals):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.linear.z = vals[0], vals[1], vals[2]
    msg.angular.x, msg.angular.y, msg.angular.z = vals[3], vals[4], vals[5]
    return msg


def _vel(data):
    msg = Float32MultiArray()
    msg.data = [float(v) for v in data]
    return msg


def _make_node():
    rclpy.init()
    clock = FakeClock()
    node = CmdVelToAttitude(clock=clock)
    return node, clock


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_valid_cmd_activates_and_recovers_after_nan():
    node, clock = _make_node()
    try:
        node._on_velocity(_vel([0.0, 0.0, 0.0]))
        node._on_cmd_vel(_twist([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        clock.advance(0.02)
        node._update()
        assert node._state == 'ACTIVE'
        assert not node._cmd_invalid

        node._on_cmd_vel(_twist([float('nan'), 0.0, 0.0, 0.0, 0.0, 0.0]))
        assert node._cmd_invalid
        assert node._err_published
        clock.advance(0.02)
        node._update()
        assert node._state == 'HOLD'

        node._on_cmd_vel(_twist([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        assert not node._cmd_invalid
        assert not node._err_published
        clock.advance(0.02)
        node._update()
        assert node._state == 'ACTIVE'
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
@pytest.mark.parametrize('bad', [
    float('nan'), float('inf'), float('-inf'),
])
@pytest.mark.parametrize('field', range(6))
def test_nonfinite_cmd_rejected_and_timestamp_frozen(bad, field):
    node, clock = _make_node()
    try:
        node._on_cmd_vel(_twist([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        clock.advance(1.0)
        t_before = node._last_cmd
        vals = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        vals[field] = bad
        node._on_cmd_vel(_twist(vals))
        assert node._cmd_invalid
        assert node._last_cmd == t_before
        assert node._err_published
        clock.advance(0.02)
        node._update()
        assert node._state == 'HOLD'
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_nonfinite_cmd_outputs_finite_hold_setpoint():
    node, clock = _make_node()
    try:
        node._on_cmd_vel(_twist([float('inf'), 0.0, 0.0, 0.0, 0.0, 0.0]))
        clock.advance(0.02)
        node._update()
        sp = node._hold_setpoint()
        assert all(math.isfinite(v) for v in sp)
        assert sp == [1.0, 0.0, 0.0, 0.0, node._hover]
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
@pytest.mark.parametrize('bad_vel', [
    [float('nan'), 0.0, 0.0],
    [0.0, float('inf'), 0.0],
    [0.0, 0.0, float('-inf')],
    [0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
])
def test_bad_velocity_rejected(bad_vel):
    node, clock = _make_node()
    try:
        node._on_velocity(_vel([0.1, 0.2, 0.3]))
        est_before = list(node._vel_est)
        t_before = node._vel_est_time
        node._on_velocity(_vel(bad_vel))
        assert node._vel_est == est_before
        assert node._vel_est_time == t_before
        clock.advance(2.0)
        node._update()
        assert node._state == 'FAULT'
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_time_regression_safe():
    node, clock = _make_node()
    try:
        node._on_cmd_vel(_twist([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        node._on_velocity(_vel([0.0, 0.0, 0.0]))
        clock.advance(0.02)
        node._update()
        assert node._state == 'ACTIVE'
        t = clock.t
        clock.t = t - 10.0
        clock.advance(0.02)
        node._update()
        assert node._state == 'ACTIVE'
        assert node._yaw == 0.0
        assert node._last_tick >= clock.t
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pure_validators():
    assert _finite_cmd([1.0, 2.0, 3.0, 0.0, 0.0, 0.0])
    assert not _finite_cmd([float('nan'), 0.0, 0.0, 0.0, 0.0, 0.0])
    assert not _finite_cmd([0.0, float('inf'), 0.0, 0.0, 0.0, 0.0])
    assert not _finite_cmd([0.0, 0.0, 0.0, 0.0, 0.0, float('-inf')])
    assert _finite_vel([0.0, 0.0, 0.0])
    assert not _finite_vel([float('nan'), 0.0, 0.0])
    assert not _finite_vel([0.0, 0.0])
    assert not _finite_vel([0.0, 0.0, 0.0, 0.0])