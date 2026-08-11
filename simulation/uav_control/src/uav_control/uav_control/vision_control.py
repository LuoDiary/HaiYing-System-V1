#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class VisionControlNode(Node):
    def __init__(self):
        super().__init__('vision_control_node')
        
        # ---------- 发布目标位置（给飞控） ----------
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # ---------- 订阅视觉目标点 ----------
        self.vision_sub = self.create_subscription(
            PointStamped,
            '/vision/target_point',
            self.vision_callback,
            10
        )
        
        # ---------- 订阅无人机状态 ----------
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        # ---------- 服务客户端 ----------
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.arm_client.wait_for_service()
        self.mode_client.wait_for_service()
        
        # ---------- 定时器 ----------
        self.timer = self.create_timer(0.05, self.send_pose)
        self.create_timer(2.0, self.arm_and_offboard)
        
        # ---------- 状态变量 ----------
        self.current_state = None
        self.target_position = [0.0, 0.0, 2.0]   # 默认目标：高度2米悬停
        
        self.get_logger().info('视觉控制节点已启动，等待目标点...')
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def vision_callback(self, msg):
        """当视觉组发布目标点时调用"""
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        
        self.target_position = [x, y, z]
        self.get_logger().info(f'收到新目标点: ({x:.2f}, {y:.2f}, {z:.2f})')
    
    def send_pose(self):
        """持续发布目标位置（20Hz）"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.target_position[0]
        pose.pose.position.y = self.target_position[1]
        pose.pose.position.z = self.target_position[2]
        self.pose_pub.publish(pose)
    
    def arm_and_offboard(self):
        """解锁并切换 OFFBOARD 模式"""
        arm_cmd = CommandBool.Request()
        arm_cmd.value = True
        self.arm_client.call_async(arm_cmd)
        
        mode_cmd = SetMode.Request()
        mode_cmd.custom_mode = 'OFFBOARD'
        self.mode_client.call_async(mode_cmd)
        
        self.get_logger().info('已发送解锁和OFFBOARD指令')

def main(args=None):
    rclpy.init(args=args)
    node = VisionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
