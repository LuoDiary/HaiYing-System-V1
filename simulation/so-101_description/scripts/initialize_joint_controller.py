#!/usr/bin/env python3
"""Initialize the Gazebo arm controller with a deterministic zero trajectory."""

import sys

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [
    'J1_Rotation',
    'J2_Shoulder_Pitch',
    'J3_Elbow_Pitch',
    'J4_Wrist_Pitch',
    'J5_Wrist_Roll',
]


class JointControllerInitializer(Node):
    def __init__(self):
        super().__init__('so101_joint_controller_initializer')
        self._client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

    def run(self):
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('arm_controller action 在 15 秒内未就绪')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(JOINT_NAMES)
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start = Duration(sec=2)
        goal.trajectory.points.append(point)
        # 启动阶段的目的只是建立首个确定 position 命令；此时模型可能已在
        # controller switch 的几十毫秒内产生瞬态误差，不能用正常轨迹容差中止。
        for name in JOINT_NAMES:
            path_tolerance = JointTolerance()
            path_tolerance.name = name
            path_tolerance.position = -1.0
            path_tolerance.velocity = -1.0
            path_tolerance.acceleration = -1.0
            goal.path_tolerance.append(path_tolerance)

            goal_tolerance = JointTolerance()
            goal_tolerance.name = name
            goal_tolerance.position = -1.0
            goal_tolerance.velocity = -1.0
            goal_tolerance.acceleration = -1.0
            goal.goal_tolerance.append(goal_tolerance)

        self.get_logger().info('发送 Gazebo 零位初始化轨迹')
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('arm_controller 拒绝零位初始化轨迹')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error('零位初始化轨迹执行超时')
            return False
        if wrapped_result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'零位初始化失败，error_code={wrapped_result.result.error_code}'
            )
            return False

        self.get_logger().info('Gazebo 关节已初始化并保持在零位')
        return True


def main():
    rclpy.init()
    node = JointControllerInitializer()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
