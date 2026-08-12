#!/usr/bin/env python3
"""
相机-雷达联合标定节点 (Task 29)
================================
发布 camera_frame ↔ lidar_frame 的外参标定TF，
处理点云与图像像素的对齐。

Gazebo仿真环境:
  标定参数精确已知（模型SDF中定义），通过TF静态变换发布。

实物环境:
  使用棋盘格标定板 → 采集同步的相机图像+LiDAR扫描
  → PnP求解相机位姿 + ICP配准点云 → 计算外参矩阵

TF树:
  base_link → camera_frame (相机内参标定)
  base_link → lidar_frame  (LiDAR外参标定)
  camera_frame → lidar_frame (联合标定结果)
"""
import numpy as np
import json
import os
import time

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class CalibrationNode(Node):
    """相机-LiDAR联合标定节点"""

    def __init__(self):
        super().__init__('calibration_node')

        # --- 参数 ---
        self.declare_parameter('calib_file', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('lidar_frame', 'lidar_frame')

        # --- 标定数据 ---
        # 相机: base_link前方15cm, 居中, 下方8cm
        # LiDAR: 假设安装在base_link顶部(0, 0, 0.3)
        self.camera_in_base = {
            'translation': [0.15, 0.0, -0.08],
            'rotation': [0.0, 0.0, 0.0, 1.0]  # 相机面向前方(+X)
        }
        self.lidar_in_base = {
            'translation': [0.0, 0.0, 0.30],
            'rotation': [0.0, 0.0, 0.0, 1.0]  # LiDAR面向前方
        }

        # 从文件加载标定参数
        calib_file = self.get_parameter('calib_file').value
        if calib_file and os.path.exists(calib_file):
            self._load_calibration(calib_file)

        # --- TF广播器 ---
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # --- 计算并发布标定TF ---
        self._publish_transforms()

        self.get_logger().info('相机-LiDAR标定TF已发布')
        self._log_calibration()

    def _load_calibration(self, path: str):
        """从JSON文件加载标定参数"""
        with open(path) as f:
            data = json.load(f)
        if 'camera_in_base' in data:
            self.camera_in_base = data['camera_in_base']
        if 'lidar_in_base' in data:
            self.lidar_in_base = data['lidar_in_base']
        self.get_logger().info(f'从文件加载标定: {path}')

    def _publish_transforms(self):
        """发布所有静态TF"""
        now = self.get_clock().now().to_msg()

        # base_link → camera_frame
        self._broadcast(
            self.get_parameter('base_frame').value,
            self.get_parameter('camera_frame').value,
            self.camera_in_base, now)

        # base_link → lidar_frame
        self._broadcast(
            self.get_parameter('base_frame').value,
            self.get_parameter('lidar_frame').value,
            self.lidar_in_base, now)

        # camera_frame → lidar_frame (联合标定)
        cam2lidar = self._compute_extrinsic()
        self._broadcast(
            self.get_parameter('camera_frame').value,
            self.get_parameter('lidar_frame').value,
            cam2lidar, now)

    def _broadcast(self, parent: str, child: str, transform: dict, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(transform['translation'][0])
        t.transform.translation.y = float(transform['translation'][1])
        t.transform.translation.z = float(transform['translation'][2])
        t.transform.rotation.x = float(transform['rotation'][0])
        t.transform.rotation.y = float(transform['rotation'][1])
        t.transform.rotation.z = float(transform['rotation'][2])
        t.transform.rotation.w = float(transform['rotation'][3])
        self.tf_broadcaster.sendTransform(t)

    def _compute_extrinsic(self):
        """计算 camera_frame → lidar_frame 外参矩阵
        T_cam_lidar = inv(T_base_cam) * T_base_lidar
        """
        # 四元数→旋转矩阵
        def q2r(q):
            x, y, z, w = q
            return np.array([
                [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
                [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]
            ])

        R_cam = q2r(self.camera_in_base['rotation'])
        t_cam = np.array(self.camera_in_base['translation'])
        R_lid = q2r(self.lidar_in_base['rotation'])
        t_lid = np.array(self.lidar_in_base['translation'])

        # T_base_cam 的逆
        R_cam_inv = R_cam.T
        t_cam_inv = -R_cam_inv @ t_cam

        # T_cam_lidar = inv(T_base_cam) * T_base_lidar
        R_ext = R_cam_inv @ R_lid
        t_ext = R_cam_inv @ t_lid + t_cam_inv

        # 旋转矩阵→四元数
        def r2q(R):
            w = np.sqrt(1.0 + R[0,0] + R[1,1] + R[2,2]) / 2.0
            if w > 1e-6:
                x = (R[2,1] - R[1,2]) / (4*w)
                y = (R[0,2] - R[2,0]) / (4*w)
                z = (R[1,0] - R[0,1]) / (4*w)
            else:
                x = np.sqrt(1 + R[0,0] - R[1,1] - R[2,2]) / 2 if R[0,0] > R[1,1] and R[0,0] > R[2,2] else 0
                y = np.sqrt(1 + R[1,1] - R[0,0] - R[2,2]) / 2 if R[1,1] > R[0,0] and R[1,1] > R[2,2] else 0
                z = np.sqrt(1 + R[2,2] - R[0,0] - R[1,1]) / 2 if R[2,2] > R[0,0] and R[2,2] > R[1,1] else 0
            return [float(x), float(y), float(z), float(w)]

        return {
            'translation': [float(t_ext[0]), float(t_ext[1]), float(t_ext[2])],
            'rotation': r2q(R_ext)
        }

    def _log_calibration(self):
        ext = self._compute_extrinsic()
        self.get_logger().info(
            f'camera→LiDAR外参: '
            f'T=({ext["translation"][0]:.3f}, {ext["translation"][1]:.3f}, {ext["translation"][2]:.3f}) '
            f'Q=({ext["rotation"][0]:.3f}, {ext["rotation"][1]:.3f}, {ext["rotation"][2]:.3f}, {ext["rotation"][3]:.3f})')

    def save_calibration(self, path: str):
        """保存当前标定参数到JSON"""
        data = {
            'camera_in_base': self.camera_in_base,
            'lidar_in_base': self.lidar_in_base,
            'extrinsic_cam2lidar': self._compute_extrinsic()
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f'标定参数已保存: {path}')


def main():
    rclpy.init()
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
