#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            10
        )
        
        self.state_sub = self.create_subscription(
            State, 
            '/mavros/state', 
            self.state_callback, 
            10
        )
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.arm_client.wait_for_service()
        self.mode_client.wait_for_service()
        
        self.timer = self.create_timer(0.05, self.send_pose)
        self.create_timer(2.0, self.arm_and_offboard)
        
        self.current_state = None
        self.takeoff_height = 2.0
        
        self.get_logger().info('起飞节点已启动，等待解锁...')
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def send_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 3.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 2.0
        self.pose_pub.publish(pose)
    
    def arm_and_offboard(self):
        arm_cmd = CommandBool.Request()
        arm_cmd.value = True
        self.arm_client.call_async(arm_cmd)
        
        mode_cmd = SetMode.Request()
        mode_cmd.custom_mode = 'OFFBOARD'
        self.mode_client.call_async(mode_cmd)
        
        self.get_logger().info('已发送解锁和OFFBOARD指令')

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
