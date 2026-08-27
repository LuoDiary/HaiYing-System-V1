#!/usr/bin/env python3
"""
接近控制器 - 状态机 + 防丢保护 (动作3)
=========================================
订阅 /vision/target_point → 计算距离 → 状态机 → /uav/cmd_vel + /system/current_state

状态转换:
  SEARCHING  → 无目标, 原地悬停
  APPROACHING → 距离 > 2m, 朝目标前进
  BRUSHING   → 距离 < 0.5m, 悬停等待机械臂
  HOVERING   → 目标丢失超3秒, 强制悬停
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from std_msgs.msg import String
import math
import time


class ApproachController(Node):
    """接近控制 + 防丢保护"""

    def __init__(self):
        super().__init__('approach_controller')

        # --- 参数 ---
        self.declare_parameter('approach_distance', 2.0)    # 开始接近距离(m)
        self.declare_parameter('brush_distance', 0.5)       # 作业距离(m)
        self.declare_parameter('max_speed', 1.0)            # 最大接近速度(m/s)
        self.declare_parameter('target_timeout', 3.0)       # 目标丢失超时(s)
        self.declare_parameter('pose_source', 'mavros')     # mavros=飞控位姿, gt=Gazebo真值
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('pose_gt_topic', '/drone/pose_gt')

        self.approach_dist = self.get_parameter('approach_distance').value
        self.brush_dist = self.get_parameter('brush_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.target_timeout = self.get_parameter('target_timeout').value

        # --- 状态 ---
        self.state = 'SEARCHING'
        self.latest_drone_pose = None
        self.latest_target = None
        self.last_target_time = 0.0

        # --- 订阅 ---
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointStamped, '/vision/target_point', self._on_target, 10)

        pose_source = self.get_parameter('pose_source').value
        if pose_source == 'gt':
            self.create_subscription(
                PoseStamped, self.get_parameter('pose_gt_topic').value,
                self._on_pose, qos)
            self.get_logger().info('位姿源: Gazebo真值 (/drone/pose_gt)')
        else:
            self.create_subscription(
                PoseStamped, self.get_parameter('pose_topic').value,
                self._on_pose, qos)
            self.get_logger().info('位姿源: MAVROS (/mavros/local_position/pose)')

        # --- 发布 ---
        self.state_pub = self.create_publisher(
            String, '/system/current_state', 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/uav/cmd_vel', 10)

        # --- 定时器 (10Hz) ---
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'接近控制器启动 | 接近距离: {self.approach_dist}m | '
            f'作业距离: {self.brush_dist}m | 超时: {self.target_timeout}s')

    def _on_target(self, msg: PointStamped):
        self.latest_target = msg
        self.last_target_time = time.time()

    def _on_pose(self, msg: PoseStamped):
        self.latest_drone_pose = msg.pose

    def _control_loop(self):
        """主控制循环 — 状态机"""
        now = time.time()
        self._publish_state()  # 持续发布状态

        # 检查目标超时
        has_target = (self.latest_target is not None and
                      now - self.last_target_time < self.target_timeout)

        if not has_target:
            # 目标丢失 → 强制悬停
            if self.state != 'SEARCHING' and self.state != 'HOVERING':
                self.get_logger().warn(
                    f'目标丢失 {now - self.last_target_time:.1f}s, 强制悬停!')
            self._set_state('HOVERING')
            self._publish_cmd_vel(0.0, 0.0, 0.0, 0.0)  # 悬停
            return

        if self.latest_drone_pose is None:
            return

        # 计算距离
        tx = self.latest_target.point.x
        ty = self.latest_target.point.y
        tz = self.latest_target.point.z
        dx = self.latest_drone_pose.position.x
        dy = self.latest_drone_pose.position.y
        dz = self.latest_drone_pose.position.z

        distance = math.sqrt((tx-dx)**2 + (ty-dy)**2 + (tz-dz)**2)

        # --- 状态转换 ---
        if distance > self.approach_dist:
            # 距离 > 2m → 接近
            self._set_state('APPROACHING')

            # 计算接近速度 (方向: drone → target)
            vx = (tx - dx) / distance * self.max_speed
            vy = (ty - dy) / distance * self.max_speed
            vz = (tz - dz) / distance * self.max_speed
            # 限幅
            vx = max(-self.max_speed, min(self.max_speed, vx))
            vy = max(-self.max_speed, min(self.max_speed, vy))
            vz = max(-self.max_speed, min(self.max_speed, vz))
            self._publish_cmd_vel(vx, vy, vz, 0.0)

            self.get_logger().info(
                f'APPROACHING | 距离: {distance:.1f}m | '
                f'速度: ({vx:.1f}, {vy:.1f}, {vz:.1f})',
                throttle_duration_sec=2.0)

        elif distance < self.brush_dist:
            # 距离 < 0.5m → 作业
            self._set_state('BRUSHING')
            self._publish_cmd_vel(0.0, 0.0, 0.0, 0.0)  # 悬停
            self.get_logger().info(
                f'BRUSHING | 距离: {distance:.2f}m | 机械臂作业中',
                throttle_duration_sec=2.0)

        else:
            # 0.5m ~ 2m → 继续接近 (减速)
            self._set_state('APPROACHING')
            ratio = (distance - self.brush_dist) / (self.approach_dist - self.brush_dist)
            speed = self.max_speed * ratio * 0.5
            vx = (tx - dx) / distance * speed
            vy = (ty - dy) / distance * speed
            vz = (tz - dz) / distance * speed
            self._publish_cmd_vel(vx, vy, vz, 0.0)

    def _set_state(self, new_state: str):
        if self.state != new_state:
            self.state = new_state
            self.get_logger().info(f'状态切换: → {new_state}')

    def _publish_state(self):
        """持续发布当前状态 (10Hz) - 防止下游丢消息"""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_cmd_vel(self, vx: float, vy: float, vz: float, yaw_rate: float):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.cmd_vel_pub.publish(msg)


def main():
    rclpy.init()
    node = ApproachController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
