from __future__ import annotations

import threading
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path

from .config import load_bridge_config
from .contracts import MissionState, PlanSummary, TargetPose
from .coordinator import ArmCoordinator, StateGateError
from .ik_client import IkClient


try:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from std_msgs.msg import String

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object


class HaiYingArmBridgeNode(Node):
    def __init__(self) -> None:
        if not ROS2_AVAILABLE:
            raise RuntimeError("缺少 ROS 2 Humble 运行依赖：rclpy/geometry_msgs/std_msgs")
        super().__init__("haiying_arm_bridge")
        default_config = Path(get_package_share_directory("haiying_zhixun_bridge")) / "config" / "arm_bridge.yaml"
        self.declare_parameter("config_path", str(default_config))
        config_value = self.get_parameter("config_path").value
        if not isinstance(config_value, str):
            raise ValueError("config_path ROS 参数必须是字符串")
        self._config = load_bridge_config(Path(config_value))
        self._coordinator = ArmCoordinator(IkClient(self._config.ik), (0.0,) * 5)
        self._planning_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="haiying_ik_planner"
        )
        self._planning_lock = threading.Lock()
        self._planning_future: Future[PlanSummary] | None = None
        self._destroying = False
        self.create_subscription(String, self._config.ros2.current_state_topic, self._on_state, 10)
        self.create_subscription(PoseStamped, self._config.ros2.target_pose_topic, self._on_target, 10)
        self.get_logger().info("海鹰智巡机械臂桥接节点已启动：默认仅规划，不执行实机")
        self.get_logger().warning("当前五自由度 IK 只使用 PoseStamped.position，不约束 orientation")

    def _on_state(self, message: object) -> None:
        state_text = getattr(message, "data", None)
        if not isinstance(state_text, str):
            self.get_logger().error("/system/current_state 消息缺少字符串 data")
            return
        try:
            state = self._coordinator.update_state(state_text)
            self.get_logger().info(f"系统状态更新为 {state.value}")
        except ValueError as error:
            self.get_logger().error(str(error))

    def _on_target(self, message: object) -> None:
        header = getattr(message, "header", None)
        pose = getattr(message, "pose", None)
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        frame_id = getattr(header, "frame_id", None)
        values = (
            frame_id,
            getattr(position, "x", None),
            getattr(position, "y", None),
            getattr(position, "z", None),
            getattr(orientation, "x", None),
            getattr(orientation, "y", None),
            getattr(orientation, "z", None),
            getattr(orientation, "w", None),
        )
        if not isinstance(values[0], str) or not all(isinstance(value, int | float) for value in values[1:]):
            self.get_logger().error("/arm/target_pose 消息字段不完整")
            return
        try:
            target = TargetPose(
                frame_id=values[0],
                x=float(values[1]),
                y=float(values[2]),
                z=float(values[3]),
                qx=float(values[4]),
                qy=float(values[5]),
                qz=float(values[6]),
                qw=float(values[7]),
            )
            if self._coordinator.state is not MissionState.BRUSHING:
                raise StateGateError(
                    f"当前状态 {self._coordinator.state.value} 不允许机械臂规划，"
                    "仅 BRUSHING 状态允许"
                )
            with self._planning_lock:
                if self._destroying:
                    self.get_logger().warning("桥接节点正在关闭，不再接受机械臂目标")
                    return
                if self._planning_future is not None and not self._planning_future.done():
                    self.get_logger().warning("已有机械臂目标正在规划，本次重复目标已拒绝")
                    return
                future = self._planning_executor.submit(self._coordinator.plan_target, target)
                self._planning_future = future
            self.get_logger().info("已接收机械臂目标，正在后台执行 IK 与自碰撞检查")
            future.add_done_callback(self._on_plan_done)
        except (StateGateError, RuntimeError, ValueError) as error:
            self.get_logger().warning(str(error))

    def _on_plan_done(self, future: Future[PlanSummary]) -> None:
        with self._planning_lock:
            if self._planning_future is future:
                self._planning_future = None
            destroying = self._destroying
        if destroying:
            return
        try:
            summary = future.result()
            self.get_logger().info(
                f"机械臂规划成功 plan_id={summary.plan_id}，"
                f"误差={summary.error_m * 1000:.3f} mm，帧数={summary.trajectory_frames}"
            )
        except (StateGateError, RuntimeError, ValueError) as error:
            self.get_logger().warning(str(error))
        except Exception as error:  # Keep executor failures from disappearing silently.
            self.get_logger().error(f"机械臂规划线程异常：{error}")

    def destroy_node(self) -> bool:
        with self._planning_lock:
            self._destroying = True
        self._planning_executor.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main() -> None:
    if not ROS2_AVAILABLE:
        raise RuntimeError("当前环境未安装 ROS 2 Humble，无法启动 haiying-arm-bridge-node")
    rclpy.init(args=None)
    node: HaiYingArmBridgeNode | None = None
    try:
        node = HaiYingArmBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
