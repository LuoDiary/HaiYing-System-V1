#!/usr/bin/env python3
"""
发布TF变换链: world → drone_base_link → camera_frame
从MAVROS位姿获取world→drone, 从模型参数获取drone→camera
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
import math

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        # 静态变换: drone_base_link → camera_frame
        # 相机: base_link前方15cm, 居中, 下方8cm
        self.camera_tf = TransformStamped()
        self.camera_tf.header.frame_id = 'drone_base_link'
        self.camera_tf.child_frame_id = 'camera_frame'
        self.camera_tf.transform.translation.x = 0.15
        self.camera_tf.transform.translation.y = 0.0
        self.camera_tf.transform.translation.z = -0.08
        self.camera_tf.transform.rotation.w = 1.0  # camera faces +X

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 订阅MAVROS位姿 → 发布world→drone_base_link TF
        # MAVROS使用RELIABLE QoS，必须匹配
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos)

        # 定时发布静态TF (10Hz)
        self.timer = self.create_timer(0.1, self.publish_tf)

        self.latest_pose = None
        self.get_logger().info('TF发布器已启动: world → drone_base_link → camera_frame')

    def pose_cb(self, msg):
        self.latest_pose = msg

    def publish_tf(self):
        now = self.get_clock().now().to_msg()

        # 发布 camera_frame 静态变换
        self.camera_tf.header.stamp = now
        self.tf_broadcaster.sendTransform(self.camera_tf)

        # 发布 world → drone_base_link (来自MAVROS)
        if self.latest_pose is not None:
            drone_tf = TransformStamped()
            drone_tf.header.stamp = now
            drone_tf.header.frame_id = 'world'
            drone_tf.child_frame_id = 'drone_base_link'
            p = self.latest_pose.pose.position
            o = self.latest_pose.pose.orientation
            drone_tf.transform.translation.x = p.x
            drone_tf.transform.translation.y = p.y
            drone_tf.transform.translation.z = p.z
            drone_tf.transform.rotation = o
            self.tf_broadcaster.sendTransform(drone_tf)


def main():
    rclpy.init()
    node = TfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
