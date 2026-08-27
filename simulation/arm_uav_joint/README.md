# arm_uav_joint

`arm_uav_joint` 承接
`src/SO101_COMPLETE/V7_SO101_SEND_TO_TEAMMATE` 中 V7 联合仿真的专有内容，
以 `custom_quad_333.sdf` 作为四旋翼 + SO-101 的唯一联合模型入口。组合模型中的
无人机、机械臂连杆、挂载质量和五个关节均已保留；四旋翼网格放在本包内，机械臂
网格和独立控制配置复用 `so-101_description`，不依赖旧的 `real_uav` 模型。

## 包边界

本包只承接 V7 独有的联合仿真部分：

- 最终联合 SDF、custom quad 网格和模型元数据；
- 联合 Gazebo/PX4/MAVROS 启动入口；
- SDF 发布清理脚本与 PX4 自动起飞脚本。

已有的 `so-101_description` 不迁移、不复制。SO-101 的 URDF/Xacro、STL、SRDF、
MoveIt、ros2_control、RViz、控制器配置和风机场景仍由该包唯一维护。这里也不再
提供名为 `so101_arm_uav_gazebo.urdf.xacro` 的纯机械臂包装文件，因为它不包含
无人机，容易被误认为联合模型。

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

- 唯一联合模型：`models/custom_quad_333/custom_quad_333.sdf`
- V7 四旋翼模板：`models/custom_quad_333/custom_quad_333.sdf.jinja`；该模板不包含
  最终人工合并的 SO-101 段，不能用它直接覆盖联合 SDF
- 四旋翼网格：`models/custom_quad_333/meshes/`
- 组合模型中的 `model://so-101_description/...` 路径由启动文件加入
  `GAZEBO_MODEL_PATH`
- `SO101_CUSTOM_QUAD_USE_PX4_PLUGINS` 和 `SO101_CUSTOM_QUAD_STATIC` 由 launch
  参数自动设置，不建议手工覆盖

联合模型必须使用 SDF：PX4 电机、MAVLink、IMU/GPS 等 Gazebo Classic 插件无法
由普通 URDF 完整表达。若要单独展开和控制机械臂，使用
`so-101_description/urdf/so101_arm_camera_gazebo.urdf.xacro`，不要在本包再维护
一份机械臂 URDF。

该包只保证模型资源、ROS 2 启动和构建路径可复现；实际飞行前仍需在目标
Jetson 上根据 PX4、Gazebo Classic 和飞控连接方式重新标定动力学与安全参数。
