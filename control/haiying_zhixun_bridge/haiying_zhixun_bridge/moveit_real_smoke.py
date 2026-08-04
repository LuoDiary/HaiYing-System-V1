from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .config import load_bridge_config
from .moveit_bridge import MoveItRealClient, build_snapshot, simulation_endpoint_error_deg


JOINT_NAMES = (
    "J1_Rotation",
    "J2_Shoulder_Pitch",
    "J3_Elbow_Pitch",
    "J4_Wrist_Pitch",
    "J5_Wrist_Roll",
)


class MoveItRealSmoke(Node):
    def __init__(self) -> None:
        super().__init__("haiying_moveit_real_smoke")
        self.declare_parameter("target", [0.15, -0.20, 0.25, -0.15, 0.10])
        config_path = (
            Path(get_package_share_directory("haiying_zhixun_bridge"))
            / "config"
            / "arm_bridge.yaml"
        )
        self.config = load_bridge_config(config_path)
        self.client = MoveItRealClient(self.config.moveit_real.server_url, timeout_s=10.0)
        self.action_client = ActionClient(self, MoveGroup, "move_action")
        self.display_publisher = self.create_publisher(
            DisplayTrajectory,
            self.config.moveit_real.display_trajectory_topic,
            10,
        )
        self.joint_state: JointState | None = None
        self.joint_state_monotonic = 0.0
        self.create_subscription(
            JointState,
            self.config.moveit_real.joint_states_topic,
            self._on_joint_state,
            20,
        )

    def _on_joint_state(self, message: JointState) -> None:
        self.joint_state = message
        self.joint_state_monotonic = time.monotonic()

    def run(self) -> dict[str, object]:
        target = tuple(float(value) for value in self.get_parameter("target").value)
        if len(target) != len(JOINT_NAMES) or any(abs(value) > 1.57 for value in target):
            raise ValueError("target 必须包含五个 ±1.57 rad 范围内的关节角")
        if not self.action_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("MoveIt /move_action 在 20 秒内未就绪")

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.start_state.is_diff = True
        constraints = Constraints()
        constraints.name = "haiying_moveit_real_smoke_goal"
        for name, position in zip(JOINT_NAMES, target, strict=True):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.005
            constraint.tolerance_below = 0.005
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveIt 拒绝 Plan+Execute 目标")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=45.0)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("MoveIt Plan+Execute 超时")
        result = wrapped.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt Plan+Execute 失败：{result.error_code.val}")

        trajectory = result.planned_trajectory.joint_trajectory
        snapshot = build_snapshot(
            list(trajectory.joint_names),
            [list(point.positions) for point in trajectory.points],
            [
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) / 1_000_000_000.0
                for point in trajectory.points
            ],
            list(result.trajectory_start.joint_state.name),
            list(result.trajectory_start.joint_state.position),
        )

        display = DisplayTrajectory()
        display.model_id = "so101_arm"
        display.trajectory_start = result.trajectory_start
        display.trajectory.append(result.planned_trajectory)
        for _ in range(3):
            self.display_publisher.publish(display)
            rclpy.spin_once(self, timeout_sec=0.1)

        deadline = time.monotonic() + 10.0
        simulation_error = float("inf")
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state is None:
                continue
            simulation_error = simulation_endpoint_error_deg(
                snapshot,
                list(self.joint_state.name),
                list(self.joint_state.position),
            )
            if simulation_error <= self.config.moveit_real.simulation_tolerance_deg:
                break
        if simulation_error > self.config.moveit_real.simulation_tolerance_deg:
            raise RuntimeError(
                f"Gazebo 未到达 MoveIt 终点：最大误差 {simulation_error:.3f}°"
            )

        health = self.client.health()
        validated = self.client.validate(snapshot)
        return {
            "moveit_plan_execute": "success",
            "gazebo_endpoint_error_deg": simulation_error,
            "trajectory_points": len(snapshot.positions_rad),
            "duration_s": snapshot.duration_s,
            "target_positions_deg": list(snapshot.target_positions_deg),
            "real_service_calibration_valid": health.get("calibration_valid"),
            "real_service_hardware_connected": health.get("hardware_connected"),
            "validated_trajectory_id": validated.get("trajectory_id"),
            "validated_resampled_frames": validated.get("resampled_frames"),
            "real_execute_called": False,
        }


def main() -> None:
    rclpy.init()
    node = MoveItRealSmoke()
    try:
        result = node.run()
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
