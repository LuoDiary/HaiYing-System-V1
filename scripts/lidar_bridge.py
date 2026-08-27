#!/usr/bin/env python3
"""
Gazebo LiDAR → ROS2 PointCloud2 桥接器
用法:
    export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
    python3 lidar_bridge.py

环境变量:
    GZ_LIDAR_TOPIC:  Gazebo LiDAR话题 (默认 /lidar)
    ROS_LIDAR_TOPIC: ROS2输出话题 (默认 /drone/lidar/points)
"""
import os
import time
import struct
import ctypes
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

from gz.transport13 import Node as GzNode
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked

GZ_LIDAR_TOPIC = os.environ.get('GZ_LIDAR_TOPIC', '/lidar')
ROS_LIDAR_TOPIC = os.environ.get('ROS_LIDAR_TOPIC', '/drone/lidar/points')


class GzLidarBridge(Node):
    """Gazebo LiDAR → ROS2 PointCloud2"""

    def __init__(self):
        super().__init__('lidar_bridge')
        self.pc_pub = self.create_publisher(PointCloud2, ROS_LIDAR_TOPIC, 10)
        self.gz_node = GzNode()
        self.frame_count = 0
        self.start_time = time.time()

        # 尝试订阅点云 (优先Packed格式)
        try:
            from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
            self.gz_node.subscribe(PointCloudPacked, GZ_LIDAR_TOPIC, self._on_pointcloud)
            self.get_logger().info(f'订阅Gazebo点云: {GZ_LIDAR_TOPIC}')
        except Exception:
            self.get_logger().warn('PointCloudPacked不可用，尝试LaserScan...')
            try:
                from gz.msgs10.laserscan_pb2 import LaserScan
                self.gz_node.subscribe(LaserScan, GZ_LIDAR_TOPIC, self._on_laserscan)
                self.get_logger().info(f'订阅Gazebo激光扫描: {GZ_LIDAR_TOPIC}')
            except Exception as e:
                self.get_logger().error(f'无法订阅LiDAR话题: {e}')

        self.get_logger().info(f'LiDAR桥接器已启动: {GZ_LIDAR_TOPIC} → {ROS_LIDAR_TOPIC}')

    def _on_pointcloud(self, msg):
        """PointCloudPacked → PointCloud2"""
        try:
            # 解析packed点云数据
            data = msg.data
            height = msg.height
            width = msg.width
            n_points = height * width if height > 0 else len(data) // msg.point_step

            if n_points == 0:
                return

            # 构建PointCloud2
            pc2 = PointCloud2()
            pc2.header.stamp = self.get_clock().now().to_msg()
            pc2.header.frame_id = 'lidar_frame'
            pc2.height = 1
            pc2.width = n_points
            pc2.is_dense = True
            pc2.is_bigendian = False
            pc2.point_step = 16  # xyz(float32) + intensity(float32)
            pc2.row_step = pc2.point_step * n_points

            # 字段定义
            pc2.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            # 填充数据 (简化: 从packed数据中提取)
            pc2.data = data[:n_points * 16] if len(data) >= n_points * 16 else data
            self.pc_pub.publish(pc2)

            self.frame_count += 1
            if self.frame_count % 100 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(f'LiDAR: {self.frame_count}帧 | {fps:.1f}FPS | {n_points}点')

        except Exception as e:
            self.get_logger().error(f'点云处理错误: {e}')

    def _on_laserscan(self, msg):
        """LaserScan → PointCloud2 (降级方案)"""
        try:
            n_points = msg.count
            if n_points == 0:
                return

            ranges = np.array(msg.ranges, dtype=np.float32)
            angle_min = msg.angle_min
            angle_step = msg.angle_step

            points = []
            for i, r in enumerate(ranges):
                if r > msg.range_min and r < msg.range_max:
                    angle = angle_min + i * angle_step
                    x = r * np.cos(angle)
                    y = r * np.sin(angle)
                    z = 0.0
                    points.extend([x, y, z, 0.0])  # xyz + intensity

            pc2 = PointCloud2()
            pc2.header.stamp = self.get_clock().now().to_msg()
            pc2.header.frame_id = 'lidar_frame'
            pc2.height = 1
            pc2.width = len(points) // 4
            pc2.point_step = 16
            pc2.row_step = pc2.point_step * pc2.width
            pc2.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            pc2.data = struct.pack(f'<{len(points)}f', *points)
            self.pc_pub.publish(pc2)

            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f'LaserScan处理错误: {e}')


def main():
    rclpy.init()
    node = GzLidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
