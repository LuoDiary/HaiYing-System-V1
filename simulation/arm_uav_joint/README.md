# arm_uav_joint

`arm_uav_joint` 是 `custom_quad_333.sdf` 为模型源文件的四旋翼 + SO-101
机械臂联合仿真包。组合模型中的机械臂连杆、挂载质量和五个关节均已保留；四旋翼
网格放在本包内，机械臂网格复用 `so-101_description`，因此不依赖旧的
`real_uav` 模型。

## 环境与依赖

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11 与 `gazebo_ros`
- Python 3.10、`lxml`、`xacro`
- ROS 包：`gazebo_ros2_control`、`controller_manager`、
  `joint_state_broadcaster`、`joint_trajectory_controller`、`so-101_description`
- MoveIt 2 仅在启动独立 SO-101 机械臂时使用
- PX4 SITL、MAVROS 仅在启用 PX4 模式时使用

先加载 ROS 环境，并确认 `so-101_description` 已安装到同一个工作空间：

```bash
cd /path/to/your_workspace
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/HaiYing-System-V1/simulation \
  --ignore-src -r -y
colcon build --base-paths src/HaiYing-System-V1/simulation \
  --packages-select so-101_description arm_uav_joint --symlink-install
source install/setup.bash
```

如果 `spawn_entity.py` 报 `No module named 'lxml'`，安装当前 ROS Python
解释器可见的依赖：

```bash
sudo apt install python3-lxml
```

## 启动方式

### 1. 仅显示联合模型（默认，推荐先验证模型）

默认不加载 PX4 插件，并将模型固定在空中，适合检查 Gazebo、网格和挂载关系：

```bash
ros2 launch arm_uav_joint arm_uav_joint.launch.py
```

无图形模式：

```bash
ros2 launch arm_uav_joint arm_uav_joint.launch.py \
  gui:=false use_rviz:=false use_camera_view:=false
```

### 2. 启用 PX4 SITL + MAVROS

动态飞行模式必须显式关闭静态模型，并提供本机 PX4 路径：

```bash
ros2 launch arm_uav_joint arm_uav_joint.launch.py \
  use_px4_plugins:=true custom_quad_static:=false auto_takeoff:=false \
  px4_autopilot_dir:=/path/to/PX4-Autopilot \
  px4_build_dir:=/path/to/PX4-Autopilot/build/px4_sitl_default \
  use_rviz:=false use_camera_view:=false
```

自动起飞时，在上述命令基础上增加：

```bash
auto_takeoff:=true handoff_to_qgc:=true takeoff_altitude:=2.0
```

这里的 `custom_quad_333.sdf` 已经包含 SO-101。联合模型模式下不要再单独
执行一次 SO-101 的 `spawn_entity.py`，否则会生成两个机械臂。

### 3. 单独调试机械臂

如需测试原有 Gazebo + MoveIt + ros2_control 机械臂链路，可关闭联合模型：

```bash
ros2 launch arm_uav_joint arm_uav_joint.launch.py \
  show_custom_quad:=false show_so101_arm:=true \
  use_px4_plugins:=false custom_quad_static:=true
```

机械臂控制器和 MoveIt 配置继续来自 `so-101_description`；本包只负责统一
启动入口和联合模型资源。

## 模型与路径约定

- 源模型：`models/custom_quad_333/custom_quad_333.sdf`
- Jinja 源：`models/custom_quad_333/custom_quad_333.sdf.jinja`
- 四旋翼网格：`models/custom_quad_333/meshes/`
- 组合模型中的 `model://so-101_description/...` 路径由启动文件加入
  `GAZEBO_MODEL_PATH`
- `SO101_CUSTOM_QUAD_USE_PX4_PLUGINS` 和 `SO101_CUSTOM_QUAD_STATIC` 由 launch
  参数自动设置，不建议手工覆盖

该包只保证模型资源、ROS 2 启动和构建路径可复现；实际飞行前仍需在目标
Jetson 上根据 PX4、Gazebo Classic 和飞控连接方式重新标定动力学与安全参数。
