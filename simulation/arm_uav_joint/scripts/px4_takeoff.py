#!/usr/bin/env python3
"""通过 MAVROS 进入 PX4 Offboard 并保持 custom quad 在目标高度。"""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, ParamSetV2, SetMode
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.task import Future


class Px4Takeoff(Node):
    """发布位置设定点，并按 PX4 要求完成 Offboard、解锁和爬升。"""

    def __init__(self) -> None:
        super().__init__('px4_takeoff')
        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('hover_thrust', 0.65)
        self.declare_parameter('handoff_to_qgc', True)
        self.target_altitude = float(self.get_parameter('takeoff_altitude').value)
        self.hover_thrust = float(self.get_parameter('hover_thrust').value)
        handoff_value = self.get_parameter('handoff_to_qgc').value
        if isinstance(handoff_value, bool):
            self.handoff_to_qgc = handoff_value
        else:
            self.handoff_to_qgc = str(handoff_value).strip().lower() in {
                '1', 'true', 'yes', 'on',
            }
        self.state = State()
        self.current_altitude = 0.0
        self.setpoint_count = 0
        self.last_log_ns = 0
        self.last_mode_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_param_request_ns = 0
        self.last_handoff_request_ns = 0
        self.mode_request_pending = False
        self.arm_request_pending = False
        self.param_request_pending = False
        self.handoff_request_pending = False
        self.takeoff_started = False
        self.handoff_complete = False
        self.px4_params_configured = False

        self.setpoint_publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10,
        )
        self.create_subscription(State, '/mavros/state', self._state_callback, 10)
        pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            pose_qos,
        )
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.param_client = self.create_client(ParamSetV2, '/mavros/param/set')
        self.control_timer = self.create_timer(0.05, self._control_callback)
        self.get_logger().info(
            f'等待 PX4 连接，目标高度（ENU）={self.target_altitude:.2f} m，'
            f'悬停推力={self.hover_thrust:.2f}，'
            f'起飞后交给 QGC={self.handoff_to_qgc}'
        )

    def _state_callback(self, message: State) -> None:
        self.state = message

    def _pose_callback(self, message: PoseStamped) -> None:
        self.current_altitude = message.pose.position.z

    def _publish_setpoint(self) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = 0.0
        message.pose.position.y = 0.0
        message.pose.position.z = self.target_altitude
        message.pose.orientation.w = 1.0
        self.setpoint_publisher.publish(message)
        self.setpoint_count += 1

    def _control_callback(self) -> None:
        if self.handoff_complete:
            return
        self._publish_setpoint()
        if not self.state.connected:
            self._log_throttled('等待 MAVROS 与 PX4 建立连接')
            return

        # 进入 OFFBOARD 后如果外部已经切换模式，说明 QGC 正在接管，不能再抢回 OFFBOARD。
        if self.takeoff_started and self.state.mode != 'OFFBOARD':
            if self.handoff_to_qgc:
                self._finish_qgc_handoff()
                return

        if not self.px4_params_configured:
            self._configure_px4_parameters()
            return
        if self.setpoint_count < 100:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.state.mode != 'OFFBOARD':
            if (
                not self.mode_request_pending
                and now_ns - self.last_mode_request_ns > 2_000_000_000
                and self.mode_client.service_is_ready()
            ):
                request = SetMode.Request()
                request.base_mode = 0
                request.custom_mode = 'OFFBOARD'
                self.mode_request_pending = True
                self.last_mode_request_ns = now_ns
                future = self.mode_client.call_async(request)
                future.add_done_callback(self._mode_response_callback)
            return

        self.takeoff_started = True
        if not self.state.armed:
            if (
                not self.arm_request_pending
                and now_ns - self.last_arm_request_ns > 2_000_000_000
                and self.arm_client.service_is_ready()
            ):
                request = CommandBool.Request()
                request.value = True
                self.arm_request_pending = True
                self.last_arm_request_ns = now_ns
                future = self.arm_client.call_async(request)
                future.add_done_callback(self._arm_response_callback)
            return

        if self.handoff_to_qgc and self.current_altitude >= self.target_altitude - 0.15:
            self._request_qgc_handoff()
            return

        self._log_throttled(
            f'PX4 已解锁并保持 Offboard，当前高度={self.current_altitude:.2f} m'
        )

    def _mode_response_callback(self, future: Future) -> None:
        self.mode_request_pending = False
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('PX4 已切换到 OFFBOARD')
            else:
                self.get_logger().warning('PX4 拒绝 OFFBOARD，2 秒后重试')
        except Exception as error:
            self.get_logger().warning(f'请求 OFFBOARD 失败：{error}')

    def _request_qgc_handoff(self) -> None:
        """起飞完成后切入自动悬停，停止本节点发布以便 QGC 接管任务。"""
        if self.handoff_request_pending or self.state.mode == 'AUTO.LOITER':
            if self.state.mode == 'AUTO.LOITER':
                self._finish_qgc_handoff()
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_handoff_request_ns <= 2_000_000_000:
            return
        if not self.mode_client.service_is_ready():
            self._log_throttled('等待 MAVROS 模式服务，准备释放控制权给 QGC')
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = 'AUTO.LOITER'
        self.handoff_request_pending = True
        self.last_handoff_request_ns = now_ns
        future = self.mode_client.call_async(request)
        future.add_done_callback(self._handoff_response_callback)

    def _handoff_response_callback(self, future: Future) -> None:
        self.handoff_request_pending = False
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('PX4 已切换到 AUTO.LOITER，控制权已释放给 QGC')
            else:
                self.get_logger().warning('PX4 拒绝 AUTO.LOITER，2 秒后重试')
        except Exception as error:
            self.get_logger().warning(f'请求 AUTO.LOITER 失败：{error}')

    def _finish_qgc_handoff(self) -> None:
        if self.handoff_complete:
            return
        self.handoff_complete = True
        self.control_timer.cancel()
        self.get_logger().info('自动起飞节点已停止发布 setpoint，现在可以由 QGC 规划并执行任务')

    def _arm_response_callback(self, future: Future) -> None:
        self.arm_request_pending = False
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('PX4 解锁请求成功')
            else:
                self.get_logger().warning('PX4 拒绝解锁，2 秒后重试')
        except Exception as error:
            self.get_logger().warning(f'请求解锁失败：{error}')

    def _configure_px4_parameters(self) -> None:
        """按 custom quad 的实际组合质量设置 PX4 悬停推力。"""
        if self.param_request_pending:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_param_request_ns <= 2_000_000_000:
            return
        if not self.param_client.service_is_ready():
            self._log_throttled('等待 MAVROS 参数服务')
            return
        request = ParamSetV2.Request()
        request.force_set = False
        request.param_id = 'MPC_THR_HOVER'
        request.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=self.hover_thrust,
        )
        self.param_request_pending = True
        self.last_param_request_ns = now_ns
        future = self.param_client.call_async(request)
        future.add_done_callback(self._parameter_response_callback)

    def _parameter_response_callback(self, future: Future) -> None:
        self.param_request_pending = False
        try:
            response = future.result()
            if response.success:
                self.px4_params_configured = True
                self.get_logger().info(
                    f'已设置 PX4 MPC_THR_HOVER={response.value.double_value:.2f}'
                )
            else:
                self.get_logger().warning('PX4 拒绝设置 MPC_THR_HOVER，2 秒后重试')
        except Exception as error:
            self.get_logger().warning(f'设置 PX4 悬停推力失败：{error}')

    def _log_throttled(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns > 3_000_000_000:
            self.get_logger().info(message)
            self.last_log_ns = now_ns


def main(args: list[str]) -> int:
    rclpy.init(args=args)
    node = Px4Takeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
