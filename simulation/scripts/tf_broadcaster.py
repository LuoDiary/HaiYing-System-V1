#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tfs()
        self.get_logger().info('TF树节点已启动')

    def publish_static_tfs(self):
        # ============================================================
        # 1. base_link -> camera_link（深度相机位置）
        # 注意：以下 x/y/z 为临时占位值，待硬件组确认后修改
        # ============================================================
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_link'
        camera_tf.transform.translation.x = 0.15   # 无人机前方 15cm（待确认）
        camera_tf.transform.translation.y = 0.0    # 居中（待确认）
        camera_tf.transform.translation.z = -0.05  # 无人机下方 5cm（待确认）
        camera_tf.transform.rotation.w = 1.0       # 无旋转
        self.static_broadcaster.sendTransform(camera_tf)

        # ============================================================
        # 2. base_link -> arm_base_link（机械臂基座位置）
        # 注意：以下 x/y/z 为临时占位值，待曹圆圆提供URDF后修改
        # ============================================================
        arm_tf = TransformStamped()
        arm_tf.header.stamp = self.get_clock().now().to_msg()
        arm_tf.header.frame_id = 'base_link'
        arm_tf.child_frame_id = 'arm_base_link'
        arm_tf.transform.translation.x = 0.0
        arm_tf.transform.translation.y = 0.0
        arm_tf.transform.translation.z = -0.1   # 无人机下方 10cm（待确认）
        arm_tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(arm_tf)

        self.get_logger().info('静态TF已发布: camera_link 和 arm_base_link')

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
