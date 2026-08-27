#!/usr/bin/env python3
"""
Gazebo位姿真值桥接器 - 发布无人机Ground Truth位姿
仿真验证用: 当MAVROS EKF不可靠时提供可靠位姿源
"""
import os
import sys
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

MODEL_NAME = os.environ.get('GZ_MODEL_NAME', 'x500_depth_custom_0')
GZ_POSE_TOPIC = '/world/default/pose/info'


class PoseGtBridge(Node):
    def __init__(self):
        super().__init__('pose_gt_bridge')
        self.pub = self.create_publisher(PoseStamped, '/drone/pose_gt', 10)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, GZ_POSE_TOPIC, self._on_pose_v)
        self.get_logger().info(
            f'位姿真值桥接: {GZ_POSE_TOPIC} → /drone/pose_gt (模型: {MODEL_NAME})')

    def _on_pose_v(self, msg):
        for pose in msg.pose:
            if pose.name == MODEL_NAME:
                out = PoseStamped()
                out.header.stamp = self.get_clock().now().to_msg()
                out.header.frame_id = 'world'
                out.pose.position.x = pose.position.x
                out.pose.position.y = pose.position.y
                out.pose.position.z = pose.position.z
                out.pose.orientation.x = pose.orientation.x
                out.pose.orientation.y = pose.orientation.y
                out.pose.orientation.z = pose.orientation.z
                out.pose.orientation.w = pose.orientation.w
                self.pub.publish(out)
                break


def main():
    rclpy.init()
    node = PoseGtBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
