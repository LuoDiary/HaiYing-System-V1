from __future__ import annotations

import math
import queue
import signal
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .config import BridgeConfig, load_bridge_config
from .moveit_bridge import (
    MoveItRealClient,
    MoveItTrajectorySnapshot,
    build_snapshot,
    simulation_endpoint_error_deg,
)


class MoveItTrajectoryMonitor(Node):
    def __init__(self, config: BridgeConfig):
        super().__init__("haiying_moveit_real_gui")
        self.config = config
        self.snapshot: MoveItTrajectorySnapshot | None = None
        self.snapshot_version = 0
        self.trajectory_error: str | None = None
        self.joint_state_names: tuple[str, ...] = ()
        self.joint_state_positions: tuple[float, ...] = ()
        self.joint_state_received_monotonic = 0.0
        self.create_subscription(
            DisplayTrajectory,
            config.moveit_real.display_trajectory_topic,
            self._on_trajectory,
            10,
        )
        self.create_subscription(
            JointState,
            config.moveit_real.joint_states_topic,
            self._on_joint_state,
            20,
        )

    def _on_trajectory(self, message: DisplayTrajectory) -> None:
        try:
            if len(message.trajectory) != 1:
                raise ValueError(
                    f"只接受单段 MoveIt RobotTrajectory，当前为 {len(message.trajectory)} 段"
                )
            trajectory = message.trajectory[0].joint_trajectory
            snapshot = build_snapshot(
                list(trajectory.joint_names),
                [list(point.positions) for point in trajectory.points],
                [
                    float(point.time_from_start.sec)
                    + float(point.time_from_start.nanosec) / 1_000_000_000.0
                    for point in trajectory.points
                ],
                list(message.trajectory_start.joint_state.name),
                list(message.trajectory_start.joint_state.position),
            )
        except ValueError as error:
            self.trajectory_error = str(error)
            self.get_logger().error(f"MoveIt 轨迹拒绝：{error}")
            return
        self.snapshot = snapshot
        self.snapshot_version += 1
        self.trajectory_error = None
        self.get_logger().info(
            f"捕获 MoveIt 轨迹：{len(snapshot.positions_rad)} 点，"
            f"{snapshot.duration_s:.2f}s，目标(deg)="
            f"{[round(value, 2) for value in snapshot.target_positions_deg]}"
        )

    def _on_joint_state(self, message: JointState) -> None:
        self.joint_state_names = tuple(message.name)
        self.joint_state_positions = tuple(float(value) for value in message.position)
        self.joint_state_received_monotonic = time.monotonic()


class MoveItRealGui:
    def __init__(self, root: tk.Tk, node: MoveItTrajectoryMonitor, config: BridgeConfig):
        self.root = root
        self.node = node
        self.config = config
        self.client = MoveItRealClient(config.moveit_real.server_url, timeout_s=10.0)
        self.results: queue.Queue[
            tuple[Callable[[object], None], object, bool]
        ] = queue.Queue()
        self.validated_trajectory_id: str | None = None
        self.validated_snapshot_version = -1
        self.service_health: dict[str, object] | None = None
        self.busy = False
        self.last_snapshot_version = -1

        root.title("海鹰智巡 · MoveIt 仿真到实机")
        root.geometry("720x600")
        root.minsize(680, 560)
        self._build()
        self._poll()
        self._refresh_health()

    def _build(self) -> None:
        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill=tk.BOTH, expand=True)
        ttk.Label(outer, text="MoveIt 仿真 → SO-101 实机", font=("Sans", 18, "bold")).pack(anchor=tk.W)
        ttk.Label(
            outer,
            text="先在 RViz 中 Plan & Execute；Gazebo 到达终点后，验证同一轨迹，再由按钮单次控制实机。",
            wraplength=660,
        ).pack(anchor=tk.W, pady=(4, 14))

        status = ttk.LabelFrame(outer, text="联锁状态", padding=12)
        status.pack(fill=tk.X)
        self.service_var = tk.StringVar(value="实机服务：检查中")
        self.trajectory_var = tk.StringVar(value="MoveIt 轨迹：等待 /display_planned_path")
        self.simulation_var = tk.StringVar(value="Gazebo 终点：等待 /joint_states")
        self.validation_var = tk.StringVar(value="轨迹验证：未验证")
        for variable in (
            self.service_var,
            self.trajectory_var,
            self.simulation_var,
            self.validation_var,
        ):
            ttk.Label(status, textvariable=variable, wraplength=640).pack(anchor=tk.W, pady=2)

        target = ttk.LabelFrame(outer, text="MoveIt 目标关节角（deg）", padding=12)
        target.pack(fill=tk.X, pady=12)
        self.target_vars = [tk.StringVar(value="—") for _ in range(5)]
        names = ("J1", "J2", "J3", "J4", "J5")
        for column, (name, variable) in enumerate(zip(names, self.target_vars, strict=True)):
            ttk.Label(target, text=name, font=("Sans", 10, "bold")).grid(row=0, column=column, padx=12)
            ttk.Label(target, textvariable=variable).grid(row=1, column=column, padx=12, pady=4)
            target.columnconfigure(column, weight=1)

        controls = ttk.Frame(outer)
        controls.pack(fill=tk.X, pady=4)
        self.refresh_button = ttk.Button(controls, text="刷新实机服务", command=self._refresh_health)
        self.refresh_button.pack(side=tk.LEFT)
        self.validate_button = ttk.Button(
            controls, text="验证最新 MoveIt 轨迹（不连接实机）", command=self._validate
        )
        self.validate_button.pack(side=tk.LEFT, padx=8)

        self.hardware_ack = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            outer,
            text="我已完成校准、清空机械臂工作区，并准备好急停/断电",
            variable=self.hardware_ack,
            command=self._update_buttons,
        ).pack(anchor=tk.W, pady=(14, 8))
        self.execute_button = tk.Button(
            outer,
            text="控制实机到 Gazebo 已到达的位置",
            command=self._execute,
            bg="#b91c1c",
            fg="white",
            activebackground="#991b1b",
            activeforeground="white",
            font=("Sans", 13, "bold"),
            padx=16,
            pady=12,
            state=tk.DISABLED,
        )
        self.execute_button.pack(fill=tk.X)

        log_frame = ttk.LabelFrame(outer, text="操作日志", padding=8)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(14, 0))
        self.log = tk.Text(log_frame, height=8, state=tk.DISABLED, wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)
        self._append_log("GUI 启动；不会自动连接或移动机械臂。")

    def _append_log(self, text: str) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def _run(self, operation: Callable[[], object], callback: Callable[[object], None]) -> None:
        if self.busy:
            return
        self.busy = True
        self._update_buttons()

        def worker() -> None:
            try:
                value: object = operation()
                success = True
            except Exception as error:
                value = error
                success = False
            self.results.put((callback, value, success))

        threading.Thread(target=worker, daemon=True).start()

    def _poll(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        while True:
            try:
                callback, value, success = self.results.get_nowait()
            except queue.Empty:
                break
            self.busy = False
            if success:
                callback(value)
            else:
                self._append_log(f"失败：{value}")
                messagebox.showerror("操作失败", str(value), parent=self.root)
            self._update_buttons()
        self._refresh_snapshot_display()
        self.root.after(50, self._poll)

    def _refresh_snapshot_display(self) -> None:
        snapshot = self.node.snapshot
        if self.node.snapshot_version != self.last_snapshot_version:
            self.last_snapshot_version = self.node.snapshot_version
            self.validated_trajectory_id = None
            self.validated_snapshot_version = -1
            self.validation_var.set("轨迹验证：新轨迹尚未验证")
            if snapshot is not None:
                self._append_log(
                    f"捕获 MoveIt 轨迹：{len(snapshot.positions_rad)} 点，"
                    f"{snapshot.duration_s:.2f}s"
                )
        if snapshot is None:
            if self.node.trajectory_error:
                self.trajectory_var.set(f"MoveIt 轨迹：拒绝（{self.node.trajectory_error}）")
            self._update_buttons()
            return
        self.trajectory_var.set(
            f"MoveIt 轨迹：已捕获 {len(snapshot.positions_rad)} 点 / {snapshot.duration_s:.2f}s"
        )
        for variable, value in zip(self.target_vars, snapshot.target_positions_deg, strict=True):
            variable.set(f"{value:.2f}°")
        try:
            age = time.monotonic() - self.node.joint_state_received_monotonic
            if age > self.config.moveit_real.joint_state_timeout_s:
                raise ValueError(f"joint_states 已过期 {age:.1f}s")
            error = simulation_endpoint_error_deg(
                snapshot,
                self.node.joint_state_names,
                self.node.joint_state_positions,
            )
            if error <= self.config.moveit_real.simulation_tolerance_deg:
                self.simulation_var.set(f"Gazebo 终点：已到达（最大误差 {error:.3f}°）")
            else:
                self.simulation_var.set(
                    f"Gazebo 终点：未到达（最大误差 {error:.3f}°，"
                    f"要求 ≤ {self.config.moveit_real.simulation_tolerance_deg:g}°）"
                )
        except ValueError as error:
            self.simulation_var.set(f"Gazebo 终点：不可用（{error}）")
        self._update_buttons()

    def _mapping_matches(self, health: dict[str, object]) -> bool:
        directions = health.get("direction_signs")
        offsets = health.get("zero_offsets_deg")
        if not isinstance(directions, list) or not isinstance(offsets, list):
            return False
        if len(directions) != 5 or len(offsets) != 5:
            return False
        return all(
            abs(float(actual) - expected) <= 1e-5
            for actual, expected in zip(directions, self.config.mapping.direction_signs, strict=True)
        ) and all(
            abs(float(actual) - expected) <= 1e-5
            for actual, expected in zip(offsets, self.config.mapping.zero_offsets_deg, strict=True)
        )

    def _refresh_health(self) -> None:
        self.service_var.set("实机服务：检查中")

        def completed(value: object) -> None:
            health = value
            assert isinstance(health, dict)
            self.service_health = health
            calibration = bool(health.get("calibration_valid"))
            mapping = self._mapping_matches(health)
            busy = bool(health.get("busy"))
            execution_enabled = bool(health.get("hardware_execution_enabled"))
            self.service_var.set(
                f"实机服务：在线；校准={'有效' if calibration else '无效'}；"
                f"映射={'一致' if mapping else '不一致'}；"
                f"执行={'已启用' if execution_enabled else '已禁用'}；"
                f"状态={'执行中' if busy else '空闲'}"
            )
            self._append_log("实机服务状态已刷新（未连接硬件）。")

        self._run(self.client.health, completed)

    def _validate(self) -> None:
        snapshot = self.node.snapshot
        version = self.node.snapshot_version
        if snapshot is None:
            messagebox.showwarning("没有轨迹", "请先在 RViz 中 Plan 或 Plan & Execute。", parent=self.root)
            return
        self.validation_var.set("轨迹验证：验证中（不连接实机）")

        def completed(value: object) -> None:
            result = value
            assert isinstance(result, dict)
            trajectory_id = result.get("trajectory_id")
            if not isinstance(trajectory_id, str):
                raise RuntimeError("实机服务未返回 trajectory_id")
            if version != self.node.snapshot_version:
                self.validation_var.set("轨迹验证：轨迹已变化，请重新验证")
                return
            self.validated_trajectory_id = trajectory_id
            self.validated_snapshot_version = version
            self.validation_var.set(
                f"轨迹验证：通过；{result.get('resampled_frames')} 帧；"
                f"最大速度 {float(result.get('maximum_speed_deg_s', 0.0)):.2f}°/s"
            )
            self._append_log(f"轨迹验证通过：{trajectory_id[:12]}…（未连接实机）")
            self._refresh_health()

        self._run(lambda: self.client.validate(snapshot), completed)

    def _simulation_ready(self) -> tuple[bool, float | None, str | None]:
        snapshot = self.node.snapshot
        if snapshot is None:
            return False, None, "没有 MoveIt 轨迹"
        age = time.monotonic() - self.node.joint_state_received_monotonic
        if age > self.config.moveit_real.joint_state_timeout_s:
            return False, None, f"joint_states 已过期 {age:.1f}s"
        try:
            error = simulation_endpoint_error_deg(
                snapshot,
                self.node.joint_state_names,
                self.node.joint_state_positions,
            )
        except ValueError as exception:
            return False, None, str(exception)
        if error > self.config.moveit_real.simulation_tolerance_deg:
            return False, error, "Gazebo 尚未到达规划终点"
        return True, error, None

    def _update_buttons(self) -> None:
        can_validate = self.node.snapshot is not None and not self.busy
        self.validate_button.configure(state=tk.NORMAL if can_validate else tk.DISABLED)
        calibration = bool(self.service_health and self.service_health.get("calibration_valid"))
        mapping = bool(self.service_health and self._mapping_matches(self.service_health))
        execution_enabled = bool(
            self.service_health
            and self.service_health.get("hardware_execution_enabled")
            and self.config.moveit_real.hardware_execution_enabled
        )
        simulation_ready, _, _ = self._simulation_ready()
        validated = (
            self.validated_trajectory_id is not None
            and self.validated_snapshot_version == self.node.snapshot_version
        )
        can_execute = (
            not self.busy
            and calibration
            and mapping
            and execution_enabled
            and simulation_ready
            and validated
            and self.hardware_ack.get()
        )
        self.execute_button.configure(state=tk.NORMAL if can_execute else tk.DISABLED)

    def _execute(self) -> None:
        snapshot = self.node.snapshot
        trajectory_id = self.validated_trajectory_id
        simulation_ready, error, reason = self._simulation_ready()
        if snapshot is None or trajectory_id is None:
            messagebox.showerror("联锁未通过", "轨迹尚未验证。", parent=self.root)
            return
        if not simulation_ready:
            messagebox.showerror("联锁未通过", reason or "Gazebo 未到达终点", parent=self.root)
            return
        targets = ", ".join(f"{value:.2f}°" for value in snapshot.target_positions_deg)
        confirmed = messagebox.askyesno(
            "确认控制实体机械臂",
            "即将连接 /dev/ttyACM0 并执行刚刚在 Gazebo 完成的轨迹。\n\n"
            f"目标：[{targets}]\n"
            f"Gazebo 终点最大误差：{error:.3f}°\n"
            f"轨迹编号：{trajectory_id[:12]}…\n\n"
            "确认工作区无障碍并继续？",
            icon="warning",
            parent=self.root,
        )
        if not confirmed:
            return
        self._append_log("用户确认实机执行；正在连接机械臂。")

        def completed(value: object) -> None:
            result = value
            assert isinstance(result, dict)
            self._append_log(
                f"实机轨迹完成：{result.get('commanded_frames')} 帧，"
                f"最大反馈误差 {float(result.get('maximum_feedback_error_deg', 0.0)):.3f}°"
            )
            messagebox.showinfo("执行完成", "实体机械臂已完成该 MoveIt 轨迹。", parent=self.root)
            self.validated_trajectory_id = None
            self.validated_snapshot_version = -1
            self.hardware_ack.set(False)
            self._refresh_health()

        self._run(lambda: self.client.execute(trajectory_id), completed)


def main() -> None:
    rclpy.init()
    default_config = (
        Path(get_package_share_directory("haiying_zhixun_bridge"))
        / "config"
        / "arm_bridge.yaml"
    )
    config = load_bridge_config(default_config)
    node = MoveItTrajectoryMonitor(config)
    root = tk.Tk()
    MoveItRealGui(root, node, config)
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    signal.signal(signal.SIGINT, lambda *_: root.after(0, root.destroy))
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
