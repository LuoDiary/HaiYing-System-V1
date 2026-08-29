#!/usr/bin/env python3

import math
import tempfile

import pytest

try:
    import rclpy
    from std_msgs.msg import Float32MultiArray
    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False

from attitude_cmd.mavlink_link_node import (
    SAFETY_HOLD, SAFETY_NORMAL,
    AttitudeCmdNode, _finite_lpned, _finite_quat,
)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def advance(self, dt):
        self.t += dt

    def __call__(self):
        return self.t


class FakeMav:
    class _Mav:
        def __init__(self, fail_send=False):
            self.fail_send = fail_send
            self.sent = []

        def set_attitude_target_send(self, *args):
            if self.fail_send:
                raise RuntimeError('simulated send failure')
            self.sent.append(args)

        def heartbeat_send(self, *args):
            pass

        def command_long_send(self, *args):
            self.sent.append(args)

    def __init__(self, fail_send=False):
        self.mav = self._Mav(fail_send=fail_send)
        self.closed = False

    def recv_match(self, blocking=True, timeout=1.0):
        return None

    def close(self):
        self.closed = True


def _make_node(fail_send=False):
    rclpy.init()
    clock = FakeClock()
    mav = FakeMav(fail_send=fail_send)
    tmp = tempfile.mkdtemp(prefix='px4_viz_test_')
    node = AttitudeCmdNode(clock_fn=clock, mav_conn=mav, data_dir=tmp)
    return node, clock, mav


def _sp(data):
    msg = Float32MultiArray()
    msg.data = [float(v) for v in data]
    return msg


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_fault_topic_is_independent():
    node, _, _ = _make_node()
    try:
        assert node._sys_state_pub.topic_name == '/uav/flight_fault'
        assert node._sys_state_pub.topic_name != '/system/current_state'
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_setpoint_wrong_length_rejects_and_holds():
    node, _, _ = _make_node()
    try:
        node._on_setpoint(_sp([1.0, 0.0, 0.0, 0.0]))
        assert node._safety == SAFETY_HOLD
        assert node._err_published
        assert node._setpoint == [1.0, 0.0, 0.0, 0.0, 0.0]
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_setpoint_nonfinite_rejects_and_keeps_last_valid():
    node, _, _ = _make_node()
    try:
        node._on_setpoint(_sp([1.0, 0.0, 0.0, 0.0, 0.5]))
        node._err_published = False
        node._safety = SAFETY_NORMAL
        node._on_setpoint(_sp([0.7, 0.0, 0.0, 0.7, float('nan')]))
        assert node._err_published
        assert node._setpoint == [1.0, 0.0, 0.0, 0.0, 0.5]
        node._err_published = False
        node._on_setpoint(_sp([0.7, 0.0, 0.0, 0.7, float('inf')]))
        assert node._setpoint == [1.0, 0.0, 0.0, 0.0, 0.5]
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_send_failure_enters_hold_immediately():
    node, _, _ = _make_node(fail_send=True)
    try:
        node._on_setpoint(_sp([1.0, 0.0, 0.0, 0.0, 0.5]))
        node._send_setpoint()
        assert node._err_published
        assert node._safety == SAFETY_HOLD
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_send_success_keeps_normal():
    node, _, mav = _make_node(fail_send=False)
    try:
        node._on_setpoint(_sp([1.0, 0.0, 0.0, 0.0, 0.5]))
        node._send_setpoint()
        assert node._safety == SAFETY_NORMAL
        assert not node._err_published
        assert len(mav.mav.sent) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_bad_telemetry_rejected():
    node, _, _ = _make_node()
    try:
        quat_before = list(node._att_quat)
        node._handle_attitude_quaternion([float('nan'), 0.0, 0.0, 0.0])
        assert node._att_quat == quat_before
        assert node._err_published

        node._err_published = False
        node._handle_attitude_quaternion([1.0, 0.0, 0.0, 0.0])
        assert node._att_quat == [1.0, 0.0, 0.0, 0.0]

        node._err_published = False
        pos_before = node._last_pos
        node._handle_local_position_ned([0.0, 0.0, -1.0, 0.0, float('inf'), 0.0], 0)
        assert node._last_pos == pos_before
        assert node._err_published

        node._err_published = False
        node._handle_local_position_ned([0.0, 0.0, -1.0, 0.0, 0.0, 0.0], 0)
        assert node._last_pos >= 0.0
        assert not node._err_published
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_time_regression_safe():
    node, clock, _ = _make_node()
    try:
        clock.advance(1.0)
        node._last_heartbeat = clock.t
        clock.advance(4.0)
        node._safety_check()
        assert node._err_published

        node._err_published = False
        node._safety = SAFETY_NORMAL
        t = clock.t
        clock.t = t - 30.0
        node._safety_check()
        assert not node._err_published

        clock.t = t
        clock.advance(0.5)
        node._last_heartbeat = clock.t
        clock.advance(4.0)
        node._safety_check()
        assert node._err_published
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pure_validators():
    assert _finite_quat([1.0, 0.0, 0.0, 0.0])
    assert not _finite_quat([1.0, float('nan'), 0.0, 0.0])
    assert not _finite_quat([1.0, 0.0, float('inf'), 0.0])
    assert _finite_lpned([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])
    assert not _finite_lpned([0.0, 0.0, -1.0, 0.0, float('-inf'), 0.0])
    assert not _finite_lpned([float('nan'), 0.0, -1.0, 0.0, 0.0, 0.0])