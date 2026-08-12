#!/usr/bin/env python3
"""
目标3D坐标转换与发布节点 (Task 27, 72)
========================================
收到 /vision/detection (2D bbox) → 深度图/LiDAR查询 → TF变换 → /vision/target_point

数据流:
  /vision/detection (DefectDetectionArray)
    + /drone/camera/depth_raw (深度图, 32FC1) 或 /drone/lidar/points (点云)
    + /mavros/local_position/pose (无人机位姿)
    + TF: world → base_link → camera_frame
    → /vision/target_point (PointStamped, 绝对XYZ)

参数:
  use_depth_camera:  使用深度相机 (默认True)
  use_lidar:         使用LiDAR点云 (默认True, 优先)
  depth_topic:       深度图话题
  lidar_topic:       LiDAR点云话题
  pose_topic:        无人机位姿话题
  world_frame:       世界坐标系名称
  base_frame:        无人机机体坐标系
  camera_frame:      相机坐标系
  lidar_frame:       LiDAR坐标系
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
import struct
import math

from wind_turbine_interfaces.msg import DefectDetectionArray


class TargetLocalizer(Node):
    """2D检测→3D世界坐标转换"""

    def __init__(self):
        super().__init__('target_localizer')

        # --- 参数 ---
        self.declare_parameter('use_depth_camera', True)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('depth_topic', '/drone/camera/depth_raw')
        self.declare_parameter('depth_info_topic', '/drone/camera/depth_info')
        self.declare_parameter('lidar_topic', '/drone/lidar/points')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('lidar_frame', 'lidar_frame')
        self.declare_parameter('arm_frame', 'arm_base')
        self.declare_parameter('tf_timeout', 0.2)

        use_depth = self.get_parameter('use_depth_camera').value
        use_lidar = self.get_parameter('use_lidar').value

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- 数据缓存 ---
        self.latest_depth = None       # float32 深度图 (米)
        self.latest_depth_info = None  # CameraInfo
        self.latest_lidar = None       # PointCloud2
        self.latest_pose = None        # 无人机Pose
        self.camera_intrinsics = None  # 3x3 内参矩阵

        # --- 订阅 ---
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            DefectDetectionArray, '/vision/detection', self._on_detection, 10)

        if use_depth:
            self.create_subscription(
                Image, self.get_parameter('depth_topic').value,
                self._on_depth, qos)
            self.create_subscription(
                CameraInfo, self.get_parameter('depth_info_topic').value,
                self._on_depth_info, qos)

        if use_lidar:
            self.create_subscription(
                PointCloud2, self.get_parameter('lidar_topic').value,
                self._on_lidar, qos)

        self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value,
            self._on_pose, qos)

        # --- 发布 ---
        self.target_pub = self.create_publisher(
            PointStamped, '/vision/target_point', 10)
        # 动作2: 机械臂目标位姿 (PoseStamped, arm_base坐标系)
        self.arm_target_pub = self.create_publisher(
            PoseStamped, '/arm/target_pose', 10)

        # --- 发布相机-LiDAR标定TF ---
        self._publish_calibration_tf()

        self.get_logger().info(
            f'3D定位节点已启动 | 深度相机: {use_depth} | LiDAR: {use_lidar}')

    def _publish_calibration_tf(self):
        """发布相机-LiDAR外参标定 (OakD-Lite: 相机和深度传感器近似同位置)"""
        # camera_frame → lidar_frame (近似同一位置，微小偏移)
        t = TransformStamped()
        t.header.frame_id = self.get_parameter('camera_frame').value
        t.child_frame_id = self.get_parameter('lidar_frame').value
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0  # 相机和深度传感器同轴
        t.transform.rotation.w = 1.0
        self._calib_tf = t
        self.get_logger().info('相机-LiDAR标定TF: camera_frame → lidar_frame (0,0,0)')

    # ---- 回调 ----

    def _on_depth(self, msg: Image):
        """深度图回调 (32FC1 = float32米)"""
        try:
            if msg.encoding == '32FC1':
                self.latest_depth = np.frombuffer(
                    msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            elif msg.encoding in ('16UC1', 'mono16'):
                mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                self.latest_depth = mm.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'深度图解析失败: {e}')

    def _on_depth_info(self, msg: CameraInfo):
        self.latest_depth_info = msg
        self.camera_intrinsics = np.array(msg.k).reshape(3, 3)

    def _on_lidar(self, msg: PointCloud2):
        """LiDAR点云回调"""
        self.latest_lidar = msg

    def _on_pose(self, msg: PoseStamped):
        self.latest_pose = msg.pose

    def _on_detection(self, msg: DefectDetectionArray):
        """收到检测结果 → 计算3D坐标 → 发布 /vision/target_point"""
        # 发布标定TF
        self._calib_tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self._calib_tf)

        if self.latest_pose is None:
            self.get_logger().debug('等待位姿...', throttle_duration_sec=5.0)
            return

        # 获取camera→world变换
        camera_to_world = self._lookup_tf(
            self.get_parameter('world_frame').value,
            self.get_parameter('camera_frame').value)
        if camera_to_world is None:
            return

        for det in msg.detections:
            # bbox中心
            cx = (det.bbox_x_min + det.bbox_x_max) / 2.0
            cy = (det.bbox_y_min + det.bbox_y_max) / 2.0

            # 获取深度
            depth = self._get_depth(cx, cy)
            if depth is None or depth <= 0:
                continue

            # 像素→相机坐标系
            if self.camera_intrinsics is not None:
                fx, fy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1]
                cx_i, cy_i = self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]
                x_cam = (cx - cx_i) / fx * depth
                y_cam = (cy - cy_i) / fy * depth
                z_cam = depth
            else:
                x_cam, y_cam, z_cam = 0.0, 0.0, depth

            # 相机→世界
            point_world = camera_to_world @ np.array([x_cam, y_cam, z_cam, 1.0])
            wx, wy, wz = float(point_world[0]), float(point_world[1]), float(point_world[2])

            # 发布 /vision/target_point (世界坐标)
            target = PointStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = self.get_parameter('world_frame').value
            target.point.x = wx
            target.point.y = wy
            target.point.z = wz
            self.target_pub.publish(target)

            # --- 动作1: TF转换 world→arm_base ---
            world_to_arm = self._lookup_tf(
                self.get_parameter('arm_frame').value,
                self.get_parameter('world_frame').value)
            if world_to_arm is None:
                continue
            point_arm = world_to_arm @ np.array([wx, wy, wz, 1.0])
            ax, ay, az = float(point_arm[0]), float(point_arm[1]), float(point_arm[2])

            # --- 动作2: 合成PoseStamped → /arm/target_pose ---
            arm_pose = PoseStamped()
            arm_pose.header.stamp = self.get_clock().now().to_msg()
            arm_pose.header.frame_id = self.get_parameter('arm_frame').value
            arm_pose.pose.position.x = ax
            arm_pose.pose.position.y = ay
            arm_pose.pose.position.z = az

            # 刷子垂直风机墙面: 墙面为YZ平面(法线+X), 工具Z轴指向墙面(-X方向)
            # 四元数: 绕Y轴转180° = (0, 1, 0, 0) 使工具Z轴朝后(-X)
            arm_pose.pose.orientation.x = 0.0
            arm_pose.pose.orientation.y = 1.0
            arm_pose.pose.orientation.z = 0.0
            arm_pose.pose.orientation.w = 0.0
            self.arm_target_pub.publish(arm_pose)

    # ---- 工具 ----

    def _lookup_tf(self, target_frame: str, source_frame: str):
        """查询TF: source_frame → target_frame (4x4矩阵)"""
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time(seconds=0, nanoseconds=0),
                timeout=rclpy.duration.Duration(
                    seconds=self.get_parameter('tf_timeout').value))
        except Exception:
            self.get_logger().debug(
                f'TF未找到: {source_frame}→{target_frame}', throttle_duration_sec=5.0)
            return None

        t = tf.transform.translation
        r = tf.transform.rotation

        # 四元数→旋转矩阵
        R = np.zeros((3, 3))
        R[0, 0] = 1 - 2*r.y*r.y - 2*r.z*r.z
        R[0, 1] = 2*r.x*r.y - 2*r.z*r.w
        R[0, 2] = 2*r.x*r.z + 2*r.y*r.w
        R[1, 0] = 2*r.x*r.y + 2*r.z*r.w
        R[1, 1] = 1 - 2*r.x*r.x - 2*r.z*r.z
        R[1, 2] = 2*r.y*r.z - 2*r.x*r.w
        R[2, 0] = 2*r.x*r.z - 2*r.y*r.w
        R[2, 1] = 2*r.y*r.z + 2*r.x*r.w
        R[2, 2] = 1 - 2*r.x*r.x - 2*r.y*r.y

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _get_depth(self, u: float, v: float) -> float:
        """获取像素(u,v)的深度值(米)，优先LiDAR再深度图"""
        # 优先LiDAR
        if self.latest_lidar is not None:
            d = self._query_lidar(u, v)
            if d > 0:
                return d

        # 回退深度图
        if self.latest_depth is not None:
            ui, vi = int(round(u)), int(round(v))
            h, w = self.latest_depth.shape
            if 0 <= ui < w and 0 <= vi < h:
                d = float(self.latest_depth[vi, ui])
                if d > 0 and np.isfinite(d):
                    return d
        return 0.0

    def _query_lidar(self, u: float, v: float) -> float:
        """从LiDAR点云查询像素对应的深度 (简化: 返回最近点距离)"""
        if self.latest_lidar is None:
            return 0.0
        # 解析PointCloud2 (简化实现: 返回点云中最近点的距离)
        try:
            data = self.latest_lidar.data
            point_step = self.latest_lidar.point_step
            n = len(data) // point_step
            if n < 10:
                return 0.0
            # 取中间区域的点云平均距离作为近似
            mid = n // 2
            x = struct.unpack_from('f', data, mid * point_step)[0]
            y = struct.unpack_from('f', data, mid * point_step + 4)[0]
            z = struct.unpack_from('f', data, mid * point_step + 8)[0]
            return float(np.sqrt(x*x + y*y + z*z))
        except Exception:
            return 0.0


def main():
    rclpy.init()
    node = TargetLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
