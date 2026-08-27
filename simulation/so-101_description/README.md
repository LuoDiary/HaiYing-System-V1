# SO-101 仿真入口

先在工作空间根目录构建并刷新环境：

```bash
colcon build --packages-select so-101_description --symlink-install
source install/setup.bash
```

## 仅模型与关节调节

```bash
ros2 launch so-101_description display.launch.py
```

`joint_state_publisher_gui` 提供 5 个关节角度滑块；RViz 会显示机械臂、末端相机支架、
小型摄像头及其 TF。非 Gazebo 模式没有图像发布者，因此 `WristCamera` 图像面板为空属于
正常现象。

## MoveIt 自主规划与执行（无 Gazebo）

```bash
ros2 launch so-101_description moveit.launch.py
```

在 RViz 的 **MotionPlanning** 面板选择 `arm`，使用交互标记或目标关节状态后依次点击 **Plan**、**Execute**。执行链路为 MoveIt → `/arm_controller/follow_joint_trajectory` → `joint_trajectory_controller` → `fake_components/GenericSystem`，不再使用自写的假 action 服务。

当前默认速度和加速度缩放均为 `0.2`。五轴顺序固定为
`J1_Rotation, J2_Shoulder_Pitch, J3_Elbow_Pitch, J4_Wrist_Pitch, J5_Wrist_Roll`，规划组为
`arm`，根坐标为 `base_footprint`，TCP 为 `end_effector`。

SO-101 只有 5 自由度，任意 6D 末端位姿不一定存在逆解。稳定的自主规划操作方式是：

1. 展开右侧 **MotionPlanning**，进入 **Joints** 页；
2. 分别调节 `J1_Rotation` 到 `J5_Wrist_Roll` 的目标角度（橙色目标模型随之变化）；
3. 回到 **Planning** 页点击 **Plan**，确认轨迹后点击 **Execute**。

也可在另一个终端执行自动冒烟测试：

```bash
source install/setup.bash
ros2 run so-101_description plan_execute_smoke_test.py
```

脚本会向 `/move_action` 发送确定性的五关节目标，并检查规划和
`/arm_controller/follow_joint_trajectory` 执行结果；脚本同样使用 `0.2` 速度和加速度
缩放。

## Gazebo Classic 联调

```bash
ros2 launch so-101_description gazebo_moveit.launch.py
```

此入口加载 `gazebo_ros2_control`、`joint_state_broadcaster`、`joint_trajectory_controller` 和简化塔筒/叶片。塔筒与叶片同时作为 Gazebo 碰撞模型和 MoveIt 规划场景障碍物出现；MoveIt 的执行 action 为 `/arm_controller/follow_joint_trajectory`。

末端支架上的 RGB 摄像头以 640×480、30 Hz 发布：

- 图像：`/so101/wrist_camera/image_raw`
- 标定信息：`/so101/wrist_camera/camera_info`
- 光学坐标系：`wrist_camera_optical_frame`

Gazebo 中启用了相机传感器可视化。`gazebo_moveit.launch.py` 启动 MoveIt RViz 的同时，
会打开独立的 `rqt_image_view` 单目图像窗口并自动订阅该话题。图像窗口与 RViz 分离，
可避免部分 Xwayland/GLX 环境中多个 OGRE 渲染窗口导致的显存误报崩溃。

不需要图像窗口时可添加 `use_camera_view:=false`。也可用以下命令独立检查：

```bash
ros2 topic hz /so101/wrist_camera/image_raw
ros2 topic echo /so101/wrist_camera/camera_info --once
```

当前入口使用 Gazebo 的 `position` 接口做稳定的运动学轨迹跟随，不加载未标定的
Gazebo PID 力矩环。动力学、重力补偿和抗扰实验应另建 `effort` 控制器，并使用实物
辨识后的惯量、摩擦和增益，不能与本入口混用。

启动过程会先暂停 Gazebo，加载控制器，解除暂停并原子激活控制器，然后通过
`FollowJointTrajectory` 把五个关节初始化到零位；初始化完成后才启动 MoveIt。
这可避免机械臂在第一条规划轨迹到来前自由下落到自碰撞状态。

服务器或 CI 无图形环境时可用：

```bash
ros2 launch so-101_description gazebo_moveit.launch.py gui:=false use_rviz:=false
```

## 与实机桥接

本包只负责模型、MoveIt 和 Gazebo 轨迹执行，不直接访问 `/dev/ttyACM0`。实机操作必须
使用 `haiying_zhixun_bridge` 的 GUI：先在 Gazebo 完成同一条轨迹并通过终点验证，再由
人工确认调用 LeRobot 实机服务。完整步骤见
[`control/haiying_zhixun_bridge/docs/MOVEIT_REAL_GUI.md`](../../control/haiying_zhixun_bridge/docs/MOVEIT_REAL_GUI.md)。
