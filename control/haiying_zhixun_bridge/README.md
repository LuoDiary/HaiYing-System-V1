# SO-101 机械臂控制与仿真实机桥接

曹圆圆负责的 ROS 2 Humble `ament_python` 功能包。包内接口严格遵循仓库
[`docs/ROS2_Interface_V1.md`](../../docs/ROS2_Interface_V1.md)，不修改全局 Topic。

## 接口


| 方向 | Topic                   | 消息类型                            | 说明                                                               |
| ---- | ----------------------- | ----------------------------------- | ------------------------------------------------------------------ |
| 订阅 | `/arm/target_pose`      | `geometry_msgs/msg/PoseStamped`     | 机械臂末端目标；当前位置 IK 使用`position`，暂不约束 `orientation` |
| 订阅 | `/system/current_state` | `std_msgs/msg/String`               | 任务状态；只有`BRUSHING` 接受机械臂目标                            |
| 订阅 | `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | GUI 捕获 MoveIt 最新规划轨迹                                       |
| 订阅 | `/joint_states`         | `sensor_msgs/msg/JointState`        | 验证 Gazebo 是否到达轨迹终点                                       |

## 已实现

- `/system/current_state` 状态门控和 `/arm/target_pose` 后台 IK 规划，规划期间状态变化会使结果作废；
- MoveIt 轨迹关节顺序、时间、限位、速度与帧间步长检查；
- RViz/Gazebo 终点一致性验证；
- MoveIt/URDF 角度到 LeRobot 舵机角度的方向、零偏与校准转换；
- GUI 人工确认控制实机，服务启动与轨迹验证阶段均不连接串口；
- 轨迹哈希、起始姿态平滑对齐、反馈误差监控以及执行后断开保护；
- 不连接硬件的 MoveIt/Gazebo 冒烟测试入口。

当前现场映射为方向 `[+1,+1,-1,+1,+1]`，零偏
`[-6.417582,-0.747253,-0.527473,16.967033,-6.197802]°`。这些参数仍需通过最终
小角度实机验收固化，修改时必须同时更新 `config/arm_bridge.yaml` 和
`haiying_zhixun_bridge/lerobot_adapter.py`。

## 环境与依赖

- 本项目使用两个 Python 环境，不能把 ROS 2 和 LeRobot 混装：
  - 系统环境：ROS 2 Humble、MoveIt 2、Gazebo Classic，使用系统 Python 3.10；
  - `haiying` 环境：LeRobot、IK 和串口实机服务，使用 Python 3.12；
  - 两侧通过本机 `127.0.0.1` HTTP 服务通信。运行 `ros2 launch` 的终端不要激活
    `haiying`，运行 LeRobot 命令的终端不要用系统 Python 直接启动实机服务。
- 仿真启动文件需要仿真组提供并构建 `so-101_description`；旧位置 IK 路径需要
  `simulation/arm_urdf` 模型。
- 仓库不包含 LeRobot 源码、Conda 环境、校准缓存、YOLO 权重或数据集。当前工作空间
  中若存在 `../../vendor/lerobot`，它是仓库外的 LeRobot 源码目录。

### `haiying` 环境版本

下面是本机已验证的实机链路版本；版本约束来自 LeRobot 当前源码的
`vendor/lerobot/pyproject.toml`，括号内为本机实际版本。


| 依赖                |         版本约束 | 本机已验证版本 | 用途                          |
| ------------------- | ---------------: | -------------: | ----------------------------- |
| Python              |         `>=3.12` |      `3.12.13` | LeRobot 实机服务运行时        |
| `lerobot`           |        `==0.6.1` |        `0.6.1` | SO-101 follower、IK CLI、校准 |
| `draccus`           | `>=0.11.6,<0.12` |       `0.11.6` | LeRobot/实机服务配置解析      |
| `feetech-servo-sdk` |       `>=1.0,<2` |        `1.0.0` | Feetech STS3215 舵机通信      |
| `pyserial`          |       `>=3.5,<4` |          `3.5` | `/dev/ttyACM0` 串口访问       |
| `numpy`             |     `>=2.0,<2.3` |        `2.2.6` | LeRobot 数值运算              |
| `huggingface-hub`   |       `>=1.6,<2` |       `1.26.0` | LeRobot 校准目录常量          |
| `deepdiff`          |         `>=7,<9` |        `8.6.2` | LeRobot 配置/校准依赖         |
| `PyYAML`            |          `>=5.4` |        `6.0.3` | 本包 YAML 配置                |
| `tqdm`              |      `>=4.66,<5` |       `4.70.0` | LeRobot CLI 依赖              |

`torch`、训练数据集、相机和模型训练相关依赖不是本项目五轴实机控制链路的必需项，
不要为了启动实机服务把 ROS 2 的 `rclpy` 安装到 `haiying` 环境中。

### 创建和安装 `haiying`

以下命令从本仓库根目录（即 `src/HaiYing-System-V1`）执行。首次安装时先创建环境；
已有环境不要重复创建。

```bash
cd <arm_ws>/src/HaiYing-System-V1

# 首次安装
conda create -n haiying python=3.12 -y
conda activate haiying
python -m pip install --upgrade pip

# 推荐：使用工作空间中仓库外的 LeRobot 源码（路径不存在时改为实际路径）
python -m pip install -e ../../vendor/lerobot

# 如果没有本地 LeRobot 源码，可改用相同版本的发布包：
# python -m pip install "lerobot[feetech]==0.6.1"

# 安装本项目的 LeRobot 侧命令；--no-deps 避免把 ROS 依赖装进 Python 3.12
python -m pip install -e control/haiying_zhixun_bridge --no-deps
```

安装后进行不接触硬件的环境检查：

```bash
conda activate haiying
python --version
python -c 'import lerobot, draccus, serial; from lerobot.robots.so_follower import SO101Follower5DOF; print(lerobot.__file__); print("SO-101 LeRobot API: OK")'
python -m pip check
command -v lerobot-ik-sim lerobot-ik-real lerobot-calibrate haiying-moveit-real-server
```

预期能看到 Python 3.12、LeRobot 模块路径、`SO-101 LeRobot API: OK`、
`No broken requirements found.` 和四个命令路径。以上检查不会打开串口。

## 构建

ROS 2 构建必须在未激活 `haiying` 的系统 ROS 终端执行：

```bash
cd <arm_ws>/src/HaiYing-System-V1
source /opt/ros/humble/setup.bash
colcon build --base-paths control --packages-select haiying_zhixun_bridge --symlink-install
source install/setup.bash
```

启动只规划、不控制实机的接口节点：

```bash
ros2 run haiying_zhixun_bridge haiying-arm-bridge-node
```

该节点默认只做规划，不会自动连接或移动实体机械臂。只有系统状态为 `BRUSHING`
时才接受 `/arm/target_pose`。

## `haiying` 环境操作

### 1. 启动 IK 服务、规划和 dry-run

终端 A 使用任意普通 shell；启动脚本会自动调用 `haiying` 环境中的
`lerobot-ik-sim`。该服务只使用 URDF 模型，不访问串口：

```bash
cd <arm_ws>/src/HaiYing-System-V1
./control/haiying_zhixun_bridge/scripts/start_ik_server.sh
```

终端 B 激活 `haiying`，执行目标规划：

```bash
cd <arm_ws>/src/HaiYing-System-V1
conda activate haiying
python control/haiying_zhixun_bridge/scripts/plan_target.py \
  --x 0.005534 --y -0.179839 --z 0.171219
```

保存输出中的 `plan_id`，再执行不连接硬件的验证：

```bash
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  dry-run --plan-id <PLAN_ID>
```

`dry-run` 会检查轨迹帧数、碰撞、帧间步长和执行锁，不会打开串口。

### 2. 校准和直接实机小角度测试

先确认机械臂串口存在并有访问权限：

```bash
conda activate haiying
ls -l /dev/ttyACM0
```

首次使用或更换机械臂后进行校准。校准会连接舵机并关闭扭矩，执行前必须卸下末端
负载、确认关节活动空间安全，并准备随时断电：

```bash
lerobot-calibrate \
  --robot.type=so101_follower_5dof \
  --robot.port=/dev/ttyACM0 \
  --robot.id=jiebang_follower_arm
```

校准文件应为：

```text
~/.cache/huggingface/lerobot/calibration/robots/so101_follower_5dof/jiebang_follower_arm.json
```

校准完成后按以下顺序测试。`inspect` 会连接并读取反馈但不发送目标位置；`jog`
会移动单个关节；`execute` 会执行已通过 dry-run 的轨迹：

```bash
# 只读状态检查（会连接舵机）
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  --allow-hardware inspect

# 先只做 1° 单轴点动
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  --allow-hardware jog --joint shoulder_pan --delta-deg 1

# 仅在确认轨迹、起始姿态和现场安全条件后执行
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  --allow-hardware execute \
  --plan-id <PLAN_ID> \
  --confirm-execute
```

`--allow-hardware` 和 `--confirm-execute` 是有意设置的双重人工确认。执行前仍会检查
校准、五个关节起始误差、轨迹范围、速度、单帧步长和反馈误差。任何实机测试都应
空载、小角度，并保持急停或断电可用。

## MoveIt 仿真到实机

终端 A 使用普通系统 shell（不要激活 `haiying`）。启动脚本会自动在 `haiying`
环境中运行 `haiying-moveit-real-server`；服务启动和轨迹验证阶段不会打开串口，只有
最终执行请求才会连接实体机械臂：

```bash
cd <arm_ws>/src/HaiYing-System-V1
./control/haiying_zhixun_bridge/scripts/start_moveit_real_server.sh
```

终端 B 使用系统 ROS 环境（不要激活 `haiying`，需要仿真组的 `so-101_description`
已构建）：

```bash
cd <arm_ws>/src/HaiYing-System-V1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch haiying_zhixun_bridge moveit_real_gui.launch.py
```

操作顺序为 RViz `Plan & Execute`、等待 Gazebo 到达、GUI 验证轨迹、勾选现场安全
确认，最后通过红色按钮二次确认控制实体机械臂。详细步骤见
[`docs/MOVEIT_REAL_GUI.md`](docs/MOVEIT_REAL_GUI.md)。

如果运行 `haiying-moveit-real-server` 时出现 `ModuleNotFoundError: lerobot` 或
`ModuleNotFoundError: draccus`，说明实机服务没有在 `haiying` 环境中运行；不要把
这些包安装到系统 ROS Python，先检查 `conda run -n haiying` 和上面的环境验证命令。

如果出现 `/dev/ttyACM0` 不存在或 `Permission denied`，先检查 USB 连接、实际串口名
和当前用户的串口访问权限；在硬件问题解决前不要绕过安全联锁。

## 安全参数

实机服务默认只监听 `127.0.0.1:8767`。当前启动脚本使用：20 Hz、最大速度
20°/s、单帧 2°、首帧误差 20°、普通关节反馈误差 8°、腕滚反馈误差 15°、关节限位
±89.9°、轨迹最长 120 s。任何实机测试均应空载、小角度，并保持急停或断电可用。

## 测试

```bash
conda run -n haiying python -m pytest -q control/haiying_zhixun_bridge/tests
```

完整的 ROS 2 集成、环境检查和首次实机测试说明位于 [`docs/`](docs/)。
