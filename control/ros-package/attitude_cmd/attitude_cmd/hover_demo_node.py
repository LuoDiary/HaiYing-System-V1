#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class HoverDemoNode(Node):

    def __init__(self):
        super().__init__('hover_demo_node')

        self.declare_parameter('rate', 50)
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('thrust', 0.5)
        self.declare_parameter('roll_amplitude', 0.15)
        self.declare_parameter('roll_frequency', 0.1)

        self._rate = self.get_parameter('rate').value
        self._duration = self.get_parameter('duration').value
        self._thrust = self.get_parameter('thrust').value
        self._roll_amp = self.get_parameter('roll_amplitude').value
        self._roll_freq = self.get_parameter('roll_frequency').value

        self._pub = self.create_publisher(Float32MultiArray, 'attitude_setpoint', 1)
        self._timer = self.create_timer(1.0 / self._rate, self._publish)
        self._start = time.monotonic()

        self.get_logger().info(
            f'publishing hover setpoint for {self._duration}s '
            f'(thrust={self._thrust}, roll amp={self._roll_amp} rad)')

    def _publish(self):
        t = time.monotonic() - self._start
        if t > self._duration:
            self.get_logger().info('demo finished, publishing level-zero setpoint')
            msg = Float32MultiArray(data=[1.0, 0.0, 0.0, 0.0, 0.0])
            self._pub.publish(msg)
            self._timer.cancel()
            return

        roll = self._roll_amp * math.sin(2.0 * math.pi * self._roll_freq * t)
        qw = math.cos(roll / 2.0)
        qx = math.sin(roll / 2.0)

        msg = Float32MultiArray(data=[qw, qx, 0.0, 0.0, self._thrust])
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HoverDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
