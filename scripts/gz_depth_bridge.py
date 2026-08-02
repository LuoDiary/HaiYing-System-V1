#!/usr/bin/env python3
"""
Gazebo 深度相机 → ROS2 桥接器 (保留float32深度值)
用法: export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
      python3 gz_depth_bridge.py
"""
import os
import sys
import time
import struct
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage

GZ_DEPTH_TOPIC = os.environ.get('GZ_DEPTH_TOPIC', '/depth_camera')
ROS_DEPTH_TOPIC = os.environ.get('ROS_DEPTH_TOPIC', '/drone/camera/depth_raw')
ROS_DEPTH_INFO = os.environ.get('ROS_DEPTH_INFO', '/drone/camera/depth_info')
ROS_POINTS_TOPIC = os.environ.get('ROS_POINTS_TOPIC', '/drone/camera/depth_points')


class GzDepthBridge(Node):
    def __init__(self):
        super().__init__('gz_depth_bridge')
        self.image_pub = self.create_publisher(Image, ROS_DEPTH_TOPIC, 10)
        self.info_pub = self.create_publisher(CameraInfo, ROS_DEPTH_INFO, 10)
        self.gz_node = GzNode()
        self.frame_count = 0
        self.start_time = time.time()

        self.gz_node.subscribe(GzImage, GZ_DEPTH_TOPIC, self._on_depth)
        self.get_logger().info(f'深度桥接: {GZ_DEPTH_TOPIC} → {ROS_DEPTH_TOPIC}')

    def _on_depth(self, msg):
        try:
            w, h = msg.width, msg.height
            data = msg.data

            # R_FLOAT32: 每个像素4字节
            expected = w * h * 4
            if len(data) < expected:
                return

            # 解析float32深度值
            floats = np.frombuffer(data, dtype=np.float32, count=w*h)
            depth = floats.reshape((h, w)).copy()  # 单位: 米

            # 发布深度图像 (32FC1 = 32-bit float, 1 channel, meters)
            ros_img = Image()
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = 'depth_camera_frame'
            ros_img.height = h
            ros_img.width = w
            ros_img.encoding = '32FC1'
            ros_img.is_bigendian = 0
            ros_img.step = w * 4
            ros_img.data = depth.tobytes()
            self.image_pub.publish(ros_img)

            # 发布CameraInfo (StereoOV7251 参数)
            caminfo = CameraInfo()
            caminfo.header = ros_img.header
            caminfo.height = h
            caminfo.width = w
            # StereoOV7251: H-FOV=1.274 rad, 640x480
            fx = (w / 2) / np.tan(1.274 / 2)
            fy = fx  # 方形像素
            cx, cy = w / 2, h / 2
            caminfo.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            self.info_pub.publish(caminfo)

            self.frame_count += 1
            if self.frame_count % 100 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                valid = np.sum(depth > 0)
                self.get_logger().info(
                    f'深度帧 {self.frame_count} | {fps:.1f} FPS | '
                    f'{w}x{h} | 有效深度点: {valid} | '
                    f'范围: [{depth[depth>0].min():.2f}, {depth[depth>0].max():.2f}]m')

        except Exception as e:
            import traceback
            self.get_logger().error(f'深度处理错误: {e}\n{traceback.format_exc()}')


def main():
    rclpy.init()
    node = GzDepthBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
