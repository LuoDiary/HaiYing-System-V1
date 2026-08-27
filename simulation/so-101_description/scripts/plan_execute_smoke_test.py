#!/usr/bin/env python3
"""Send a deterministic joint-space Plan+Execute request to MoveIt."""

import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node


JOINT_NAMES = [
    'J1_Rotation',
    'J2_Shoulder_Pitch',
    'J3_Elbow_Pitch',
    'J4_Wrist_Pitch',
    'J5_Wrist_Roll',
]


class PlanExecuteSmokeTest(Node):
    def __init__(self):
        super().__init__('so101_plan_execute_smoke_test')
        self.declare_parameter('target', [0.25, -0.35, 0.45, -0.25, 0.20])
        self.declare_parameter('plan_only', False)
        self._client = ActionClient(self, MoveGroup, 'move_action')

    def run(self):
        target = list(self.get_parameter('target').value)
        plan_only = bool(self.get_parameter('plan_only').value)
        if len(target) != len(JOINT_NAMES):
            self.get_logger().error('target 必须包含 5 个关节角度（rad）')
            return False
        if any(abs(value) > 1.57 for value in target):
            self.get_logger().error('目标超出当前 SO-101 的 ±1.57 rad 关节限位')
            return False

        self.get_logger().info('等待 MoveIt /move_action...')
        if not self._client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('/move_action 在 20 秒内未就绪')
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = 'so101_joint_goal'
        for name, position in zip(JOINT_NAMES, target):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(position)
            constraint.tolerance_above = 0.005
            constraint.tolerance_below = 0.005
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        mode = 'Plan' if plan_only else 'Plan+Execute'
        self.get_logger().info(f'{mode} 目标(rad): {target}')
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('MoveIt 拒绝了目标')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=45.0)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error(f'{mode} 超时')
            return False

        error_code = wrapped_result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'{mode} 失败，MoveItErrorCodes={error_code}')
            return False

        points = len(wrapped_result.result.planned_trajectory.joint_trajectory.points)
        self.get_logger().info(f'{mode} 成功，轨迹点数={points}')
        return True


def main():
    rclpy.init()
    node = PlanExecuteSmokeTest()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
