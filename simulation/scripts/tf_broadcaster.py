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
        # 0. map -> world（身份变换，使视觉组的 world 坐标系与 MAVROS 的 map 对齐）
        # ============================================================
        map_to_world = TransformStamped()
        map_to_world.header.stamp = self.get_clock().now().to_msg()
        map_to_world.header.frame_id = 'map'
        map_to_world.child_frame_id = 'world'
        map_to_world.transform.translation.x = 0.0
        map_to_world.transform.translation.y = 0.0
        map_to_world.transform.translation.z = 0.0
        map_to_world.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(map_to_world)
        self.get_logger().info('发布 map -> world 静态变换')

        # ============================================================
        # 1. base_link -> camera_frame（深度相机位置）
        # ============================================================
        
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_frame'
        camera_tf.transform.translation.x = 0.15   # 无人机前方 15cm（确认）
        camera_tf.transform.translation.y = 0.0    # 居中（确认）
        camera_tf.transform.translation.z = -0.08  # 无人机下方 8cm（确认）
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
        arm_tf.transform.translation.z = -0.045   # 无人机下方 4.5cm（确认）
        arm_tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(arm_tf)

        self.get_logger().info('静态TF已发布: camera_link 和 arm_base_link')

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
