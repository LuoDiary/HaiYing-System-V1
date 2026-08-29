#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

G = 9.81


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _quat_from_matrix(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return [0.25 * s,
                (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s]
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return [(m[2][1] - m[1][2]) / s,
                0.25 * s,
                (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s]
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return [(m[0][2] - m[2][0]) / s,
                (m[0][1] + m[1][0]) / s,
                0.25 * s,
                (m[1][2] + m[2][1]) / s]
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return [(m[1][0] - m[0][1]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s]


def bodyz_to_quat(body_z, yaw):
    n = _norm(body_z)
    if n < 1e-9:
        body_z = [0.0, 0.0, 1.0]
    else:
        body_z = [c / n for c in body_z]
    y_c = [-math.sin(yaw), math.cos(yaw), 0.0]
    body_x = _cross(y_c, body_z)
    if body_z[2] < 0.0:
        body_x = [-c for c in body_x]
    if abs(body_z[2]) < 1e-6:
        body_x = [0.0, 0.0, 1.0]
    nx = _norm(body_x)
    body_x = [c / nx for c in body_x]
    body_y = _cross(body_z, body_x)
    m = [[body_x[0], body_y[0], body_z[0]],
         [body_x[1], body_y[1], body_z[1]],
         [body_x[2], body_y[2], body_z[2]]]
    return _quat_from_matrix(m)


def _finite_cmd(vals):
    return all(math.isfinite(v) for v in vals)


def _finite_vel(data):
    return len(data) == 3 and all(math.isfinite(float(v)) for v in data)


class CmdVelToAttitude(Node):

    def __init__(self, clock=None):
        super().__init__('cmd_vel_to_attitude')

        self.declare_parameter('rate', 50)
        self.declare_parameter('cmd_vel_timeout', 3.0)
        self.declare_parameter('vel_gain', 1.8)
        self.declare_parameter('max_accel', 2.0)
        self.declare_parameter('max_tilt', 0.5)
        self.declare_parameter('hover_thrust', 0.5)
        self.declare_parameter('thrust_min', 0.1)
        self.declare_parameter('thrust_max', 0.9)
        self.declare_parameter('vel_feedback_timeout', 1.0)

        self._rate = self.get_parameter('rate').value
        self._cmd_timeout = self.get_parameter('cmd_vel_timeout').value
        self._gain = self.get_parameter('vel_gain').value
        self._max_accel = self.get_parameter('max_accel').value
        self._max_tilt = self.get_parameter('max_tilt').value
        self._hover = self.get_parameter('hover_thrust').value
        self._thr_min = self.get_parameter('thrust_min').value
        self._thr_max = self.get_parameter('thrust_max').value
        self._vel_fb_timeout = self.get_parameter('vel_feedback_timeout').value

        self._time_fn = clock if clock is not None else time.monotonic

        self._vel_cmd = [0.0, 0.0, 0.0]
        self._yaw_rate = 0.0
        self._last_cmd = 0.0
        self._vel_est = [0.0, 0.0, 0.0]
        self._vel_est_time = 0.0
        self._yaw = 0.0
        self._last_tick = self._time_fn()
        self._state = 'HOLD'
        self._fb_warned = False
        self._cmd_invalid = False
        self._err_published = False

        self._cmd_sub = self.create_subscription(Twist, 'uav/cmd_vel', self._on_cmd_vel, 1)
        self._vel_sub = self.create_subscription(
            Float32MultiArray, 'vehicle_velocity', self._on_velocity, 1)

        self._sp_pub = self.create_publisher(Float32MultiArray, 'attitude_setpoint', 1)
        self._state_pub = self.create_publisher(String, 'uav/cmd_state', 1)
        self._sys_state_pub = self.create_publisher(String, 'system/current_state', 10)

        self._timer = self.create_timer(1.0 / self._rate, self._update)

    def _report_error(self):
        if not self._err_published:
            self._sys_state_pub.publish(String(data='ERROR'))
            self._err_published = True
            self.get_logger().error('invalid /uav/cmd_vel rejected, '
                                    'published /system/current_state=ERROR')

    def _on_cmd_vel(self, msg):
        vals = [msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z]
        if not _finite_cmd(vals):
            self._cmd_invalid = True
            self._report_error()
            return
        self._cmd_invalid = False
        self._err_published = False
        self._vel_cmd = [float(msg.linear.x), float(msg.linear.y), float(msg.linear.z)]
        self._yaw_rate = float(msg.angular.z)
        self._last_cmd = self._time_fn()

    def _on_velocity(self, msg):
        data = msg.data
        if not _finite_vel(data):
            return
        self._vel_est = [float(data[0]), float(data[1]), float(data[2])]
        self._vel_est_time = self._time_fn()

    def _hold_setpoint(self):
        return [1.0, 0.0, 0.0, 0.0, self._hover]

    def _update(self):
        now = self._time_fn()
        dt = max(now - self._last_tick, 0.0)
        self._last_tick = now

        if self._cmd_invalid:
            self._state = 'HOLD'
        elif now - self._last_cmd > self._cmd_timeout:
            self._state = 'HOLD'
        elif now - self._vel_est_time > self._vel_fb_timeout:
            if not self._fb_warned:
                self.get_logger().warn(
                    'velocity feedback stale, entering FAULT')
                self._fb_warned = True
            self._state = 'FAULT'
        else:
            self._state = 'ACTIVE'

        if self._state != 'ACTIVE':
            setpoint = self._hold_setpoint()
        else:
            v_est = self._vel_est

            a_des = [
                self._gain * (self._vel_cmd[0] - v_est[0]),
                self._gain * (self._vel_cmd[1] - v_est[1]),
                self._gain * (self._vel_cmd[2] - v_est[2]),
            ]
            norm_xy = math.hypot(a_des[0], a_des[1])
            if norm_xy > self._max_accel:
                scale = self._max_accel / norm_xy
                a_des[0] *= scale
                a_des[1] *= scale
            a_des[2] = max(-self._max_accel, min(self._max_accel, a_des[2]))

            body_z = [-a_des[0], -a_des[1], G - a_des[2]]
            n = _norm(body_z)
            body_z = [c / n for c in body_z]
            cos_max = math.cos(self._max_tilt)
            if body_z[2] < cos_max:
                xy_scale = math.sqrt(max(1.0 - cos_max * cos_max, 0.0)) / math.hypot(body_z[0], body_z[1])
                body_z = [body_z[0] * xy_scale, body_z[1] * xy_scale, cos_max]
                n = _norm(body_z)
                body_z = [c / n for c in body_z]

            self._yaw = (self._yaw + self._yaw_rate * dt) % (2.0 * math.pi)
            if self._yaw > math.pi:
                self._yaw -= 2.0 * math.pi

            q = bodyz_to_quat(body_z, self._yaw)
            thrust = (G - a_des[2]) * self._hover / G
            thrust = max(self._thr_min, min(self._thr_max, thrust))
            setpoint = q + [thrust]

        self._sp_pub.publish(Float32MultiArray(data=setpoint))
        self._state_pub.publish(String(data=self._state))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAttitude()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()