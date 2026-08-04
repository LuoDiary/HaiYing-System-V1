# SO-101 机械臂控制与仿真实机桥接

曹圆圆负责的 ROS 2 Humble `ament_python` 功能包。包内接口严格遵循仓库
[`docs/ROS2_Interface_V1.md`](../../docs/ROS2_Interface_V1.md)，不修改全局 Topic。

## 接口

| 方向 | Topic | 消息类型 | 说明 |
|---|---|---|---|
| 订阅 | `/arm/target_pose` | `geometry_msgs/msg/PoseStamped` | 机械臂末端目标；当前位置 IK 使用 `position`，暂不约束 `orientation` |
| 订阅 | `/system/current_state` | `std_msgs/msg/String` | 任务状态；只有 `BRUSHING` 接受机械臂目标 |
| 订阅 | `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | GUI 捕获 MoveIt 最新规划轨迹 |
| 订阅 | `/joint_states` | `sensor_msgs/msg/JointState` | 验证 Gazebo 是否到达轨迹终点 |

## 已实现

- `/system/current_state` 状态门控和 `/arm/target_pose` 后台 IK 规划，规划期间状态变化会使结果作废；
- MoveIt 轨迹关节顺序、时间、限位、速度与帧间步长检查；
- RViz/Gazebo 终点一致性验证；
- MoveIt/URDF 角度到 LeRobot 舵机角度的方向、零偏与校准转换；
- GUI 人工确认控制实机，服务启动与轨迹验证阶段均不连接串口；
- 轨迹哈希、起始姿态平滑对齐、反馈误差监控以及执行后断开保护；
- 不连接硬件的 MoveIt/Gazebo 冒烟测试入口。

当前现场映射为方向 `[+1,+1,-1,+1,+1]`，零偏
`[-5.406593,12.615385,0.131868,19.428571,17.450549]°`。这些参数仍需通过最终
小角度实机验收固化，修改时必须同时更新 `config/arm_bridge.yaml` 和
`haiying_zhixun_bridge/lerobot_adapter.py`。

## 环境与依赖

- ROS 2 Humble、MoveIt 2、Gazebo Classic 使用系统 Python 3.10；
- LeRobot 实机服务使用 Python 3.12 Conda 环境，默认环境名为 `haiying`；
- 仿真启动文件运行时需要仿真组提供 `so-101_description`；
- 旧位置 IK 路径运行时需要本机 `8766` IK 服务和 `arm_urdf` 模型包；
- 仓库不包含 LeRobot 源码、Conda 环境、校准缓存、YOLO 权重或数据集。

将本包安装到 LeRobot Conda 环境，以提供独立实机服务命令：

```bash
conda run -n haiying python -m pip install -e control/haiying_zhixun_bridge --no-deps
```

## 构建

在仓库作为 ROS 2 工作空间时执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths control --packages-select haiying_zhixun_bridge --symlink-install
source install/setup.bash
```

启动只规划、不控制实机的接口节点：

```bash
ros2 run haiying_zhixun_bridge haiying-arm-bridge-node
```

## MoveIt 仿真到实机

终端 A：

```bash
./control/haiying_zhixun_bridge/scripts/start_moveit_real_server.sh
```

终端 B（需要仿真组的 `so-101_description` 已构建）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch haiying_zhixun_bridge moveit_real_gui.launch.py
```

操作顺序为 RViz `Plan & Execute`、等待 Gazebo 到达、GUI 验证轨迹、勾选现场安全
确认，最后通过红色按钮二次确认控制实体机械臂。详细步骤见
[`docs/MOVEIT_REAL_GUI.md`](docs/MOVEIT_REAL_GUI.md)。

## 安全参数

实机服务默认只监听 `127.0.0.1:8767`。当前启动脚本使用：10 Hz、最大速度
30°/s、单帧 5°、轻微安全裁剪 0.5°、首帧误差 10°、反馈误差 3°、关节限位
±89.9°、轨迹最长 100 s。任何实机测试均应空载、小角度，并保持急停或断电可用。

## 测试

```bash
conda run -n haiying python -m pytest -q control/haiying_zhixun_bridge/tests
```

完整的 ROS 2 集成、环境检查和首次实机测试说明位于 [`docs/`](docs/)。
