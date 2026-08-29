# HaiYing V9.2 联合仿真模型

本包保存仿真组已验证的 V9.2 四旋翼、SO-101 机械臂、AR0234
相机和 MID-360 雷达联合模型。

## 已冻结内容

- SO-101 使用仿真组修正后的无明显穿模挂载位置。
- J2 限位：`[-0.5236, 2.9]` rad。
- J3 限位：`[0.0, 3.14]` rad。
- J5 限位：`[-3.14, 3.14]` rad。
- 相机图像：`/drone/camera/image_raw`。
- 相机信息：`/drone/camera/camera_info`。
- 雷达点云：`/drone/lidar/points`。
- 机械臂轨迹 Action：`/arm_controller/follow_joint_trajectory`。

## 复用的正式资源

本包不重复提交网格和控制器配置：

- 四旋翼网格来自 `arm_uav_joint`。
- 机械臂网格来自 `so-101_description`。
- ros2_control 配置来自
  `so-101_description/config/ros2_controllers.yaml`。

## 构建

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --base-paths simulation \
  --packages-select \
    so-101_description \
    arm_uav_joint \
    haiying_v9_2 \
  --symlink-install

source install/setup.bash
```

## 启动模型

先指定本机 PX4-Autopilot 路径：

```bash
export PX4_AUTOPILOT_DIR=/absolute/path/to/PX4-Autopilot
ros2 launch haiying_v9_2 v9_2_simulation.launch.py
```

该 launch 负责加载风机场景、生成 V9.2 联合模型、启动
robot_state_publisher、SO-101 ros2_control 控制器以及相机和雷达静态 TF。

为避免未连接 PX4 时模型自由下落，Gazebo 默认保持暂停。机械臂控制器会在暂停期间保持等待，并在物理仿真恢复后完成激活。完成 PX4 准备后可显式使用 `pause:=false`。

## 安全边界

该入口不自动发送解锁、起飞、飞行速度或机械臂轨迹。

正式 FSM、视觉节点、飞控控制链和机械臂轨迹规划节点应由各自正式功能包接入。
本目录不包含临时仿真 FSM 或触发式机械臂适配器。
