#!/usr/bin/env python3

import tempfile

import pytest

try:
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False

from attitude_cmd.mavlink_link_node import (
    SAFETY_HOLD, SAFETY_NORMAL, STARTUP_TIMEOUT,
    AttitudeCmdNode,
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
        def set_attitude_target_send(self, *args):
            pass

        def heartbeat_send(self, *args):
            pass

        def command_long_send(self, *args):
            pass

    def __init__(self):
        self.mav = self._Mav()

    def recv_match(self, blocking=True, timeout=1.0):
        return None

    def close(self):
        pass


def _make_node():
    rclpy.init()
    clock = FakeClock()
    node = AttitudeCmdNode(clock_fn=clock, mav_conn=FakeMav(),
                           data_dir=tempfile.mkdtemp(prefix='px4_viz_test_'))
    return node, clock


def _state(node, data):
    msg = String()
    msg.data = data
    return msg


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_startup_without_telemetry_reports_and_latches_hold():
    node, clock = _make_node()
    try:
        clock.advance(STARTUP_TIMEOUT + 1.0)
        node._safety_check()
        assert node._err_published
        assert node._safety == SAFETY_HOLD
        assert node._hold_latched
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_latched_hold_not_released_by_brushing():
    node, clock = _make_node()
    try:
        clock.advance(STARTUP_TIMEOUT + 1.0)
        node._safety_check()
        assert node._hold_latched
        node._on_system_state(_state(node, 'BRUSHING'))
        assert node._safety == SAFETY_HOLD
        node._on_system_state(_state(node, 'TARGET_FOUND'))
        assert node._safety == SAFETY_HOLD
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_fsm_hold_released_by_normal_state():
    node, clock = _make_node()
    try:
        clock.advance(1.0)
        node._on_system_state(_state(node, 'ERROR'))
        assert node._safety == SAFETY_HOLD
        assert not node._hold_latched
        node._on_system_state(_state(node, 'BRUSHING'))
        assert node._safety == SAFETY_NORMAL
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_command_ack_rejection_latches_hold():
    node, clock = _make_node()
    try:
        clock.advance(1.0)
        node._handle_command_ack(176, 1)
        assert node._err_published
        assert node._safety == SAFETY_HOLD
        assert node._hold_latched
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(not _HAS_RCLPY, reason='rclpy not available')
def test_safety_reset_requires_healthy_link():
    node, clock = _make_node()
    try:
        clock.advance(STARTUP_TIMEOUT + 1.0)
        node._safety_check()
        assert node._hold_latched

        resp = Trigger.Response()
        node._on_safety_reset(Trigger.Request(), resp)
        assert not resp.success

        node._last_heartbeat = clock.t
        node._last_pos = clock.t
        clock.advance(0.2)
        resp = Trigger.Response()
        node._on_safety_reset(Trigger.Request(), resp)
        assert resp.success
        assert node._safety == SAFETY_NORMAL
        assert not node._hold_latched
        assert not node._err_published
    finally:
        node.destroy_node()
        rclpy.shutdown()