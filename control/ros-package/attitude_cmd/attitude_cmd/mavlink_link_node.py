#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Quaternion
from std_srvs.srv import SetBool

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink

PX4_CUSTOM_MAIN_MODE_CUSTOM = 1
PX4_CUSTOM_SUB_MODE_AUTO_OFFBOARD = 6

PX4_MODES = {
    1: 'MANUAL',
    2: 'ALTCTL',
    3: 'POSCTL',
    4: 'AUTO',
    6: 'OFFBOARD',
}

TYPEMASK_IGNORE_RATES = 0b00000111
TYPEMASK_IGNORE_THROTTLE = 0b01000000


class AttitudeCmdNode(Node):

    def __init__(self):
        super().__init__('attitude_cmd_node')

        self.declare_parameter('port', '/dev/ttyS2')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('rate', 50)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self._sysid = self.get_parameter('target_system').value
        self._compid = self.get_parameter('target_component').value
        self._rate = self.get_parameter('rate').value

        if port.startswith('udpin:') and port.count(':') == 1:
            port = 'udpin:0.0.0.0:' + port.split(':')[1]
        elif port.startswith('udpout:') and port.count(':') == 1:
            port = 'udpout:127.0.0.1:' + port.split(':')[1]

        self._mav = mavutil.mavlink_connection(port, baud=baud)
        self.get_logger().info(f'opened {port} @ {baud} baud')

        self._setpoint = [1.0, 0.0, 0.0, 0.0, 0.0]
        self._armed = False
        self._mode = 'unknown'
        self._running = True

        self._sp_sub = self.create_subscription(
            Float32MultiArray, 'attitude_setpoint', self._on_setpoint, 1)
        self._arm_srv = self.create_service(
            SetBool, 'mavlink/arm', self._on_arm)
        self._offboard_srv = self.create_service(
            SetBool, 'mavlink/offboard', self._on_offboard)

        self._att_pub = self.create_publisher(Quaternion, 'vehicle_attitude', 1)
        self._state_pub = self.create_publisher(String, 'vehicle_state', 1)

        self._setpoint_timer = self.create_timer(1.0 / self._rate, self._send_setpoint)
        self._hb_timer = self.create_timer(0.5, self._send_heartbeat)

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _send_heartbeat(self):
        self._mav.mav.heartbeat_send(
            mavlink.MAV_TYPE_GCS,
            mavlink.MAV_AUTOPILOT_INVALID,
            0, 0,
            mavlink.MAV_STATE_ACTIVE)

    def _on_setpoint(self, msg):
        data = msg.data
        if len(data) != 5:
            self.get_logger().warn(
                f'attitude_setpoint needs 5 floats [qw qx qy qz thrust], got {len(data)}')
            return
        self._setpoint = list(data)

    def _send_setpoint(self):
        q = self._setpoint[0:4]
        thrust = float(self._setpoint[4])
        self._mav.mav.set_attitude_target_send(
            int(time.time() * 1000) % 2**32,
            self._sysid,
            self._compid,
            TYPEMASK_IGNORE_RATES,
            q,
            0.0, 0.0, 0.0,
            thrust)

    def _on_arm(self, request, response):
        self._send_command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                           [1 if request.data else 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        response.success = True
        response.message = 'arm/disarm command sent'
        return response

    def _on_offboard(self, request, response):
        if request.data:
            self._send_command(mavlink.MAV_CMD_DO_SET_MODE,
                               [PX4_CUSTOM_MAIN_MODE_CUSTOM,
                                PX4_CUSTOM_SUB_MODE_AUTO_OFFBOARD,
                                0.0, 0.0, 0.0, 0.0, 0.0])
            response.success = True
            response.message = 'OFFBOARD mode command sent'
        else:
            self._send_command(mavlink.MAV_CMD_DO_SET_MODE,
                               [PX4_CUSTOM_MAIN_MODE_CUSTOM, 1.0,
                                0.0, 0.0, 0.0, 0.0, 0.0])
            response.success = True
            response.message = 'MANUAL mode command sent'
        return response

    def _send_command(self, command, params):
        self._mav.mav.command_long_send(
            self._sysid, self._compid, command, 0,
            params[0], params[1], params[2], params[3],
            params[4], params[5], params[6])

    def _rx_loop(self):
        while self._running and rclpy.ok():
            msg = self._mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                self._armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._mode = PX4_MODES.get(msg.custom_mode >> 16, 'unknown')
                state = f'armed={self._armed} mode={self._mode}'
                self._state_pub.publish(String(data=state))
            elif mtype == 'ATTITUDE_QUATERNION':
                self._att_pub.publish(Quaternion(
                    x=float(msg.q2), y=float(msg.q3),
                    z=float(msg.q4), w=float(msg.q1)))
            elif mtype == 'COMMAND_ACK':
                self.get_logger().info(
                    f'COMMAND_ACK cmd={msg.command} result={msg.result}')

    def destroy_node(self):
        self._running = False
        self._mav.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
