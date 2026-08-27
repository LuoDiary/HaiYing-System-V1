# HaiYing-System-V1 / arm_ws 主机迁移交接说明

> 编写日期：2026-08-27
>
> 用途：当前开发机的 `arm_ws` 即将删除，后续在新的主机或 Jetson 上重新获取、构建和运行本项目。

## 1. 先记住这几件事

1. 正式源码只有 Git 仓库 `HaiYing-System-V1`，远程地址为：
   `git@github.com:woyebuzhidaocaonm-gif/HaiYing-System-V1.git`。
2. 新主机只需要克隆该仓库的 `main` 分支；不要把当前工作区的 `build/`、
   `install/`、`log/`、压缩包或历史重复包当作源码迁移。
3. 联合仿真的唯一入口是 `simulation/arm_uav_joint`，模型源文件是
   `simulation/arm_uav_joint/models/custom_quad_333/custom_quad_333.sdf`。
4. ROS 2 与 Gazebo 使用系统 Python 3.10；LeRobot、IK 和实机服务使用独立的
   Python 3.12 Conda 环境 `haiying`。两个环境不要混用。
5. 校准 JSON 不在 Git 中。若新主机要控制同一台机械臂，删除旧工作区前必须单独备份，
   再安全地复制到新主机。

## 2. 当前仓库树

下面是交接时需要认识的源码树。大量 STL、历史压缩包和编译中间文件用省略号表示；
它们不改变构建入口。

```text
arm_ws/
├── src/
│   ├── HaiYing-System-V1/                         # 唯一正式 Git 仓库
│   │   ├── control/
│   │   │   ├── haiying_zhixun_bridge/             # SO-101 ROS 2/LeRobot 桥接
│   │   │   │   ├── config/arm_bridge.yaml
│   │   │   │   ├── haiying_zhixun_bridge/
│   │   │   │   ├── launch/moveit_real_gui.launch.py
│   │   │   │   ├── scripts/
│   │   │   │   ├── tests/
│   │   │   │   └── README.md
│   │   │   ├── ros-package/attitude_cmd/           # 可选 UAV 姿态控制包
│   │   │   └── README.md
│   │   ├── decision/                               # 决策代码及历史交付资料
│   │   ├── docs/
│   │   │   ├── ROS2_Interface_V1.md                # 跨组 ROS 2 接口约定
│   │   │   ├── ARM_WS_HANDOVER.md                  # 本交接说明
│   │   │   └── superpowers/                         # 设计与执行记录
│   │   ├── hardware/README.md                      # 硬件目录说明
│   │   ├── scripts/                                # 相机、视觉、TF 等辅助脚本
│   │   ├── simulation/
│   │   │   ├── arm/                                # 轻量 SO-101 URDF；IK 使用
│   │   │   ├── so-101_description/                 # Gazebo/MoveIt/ros2_control
│   │   │   ├── arm_uav_joint/                      # 当前四旋翼 + SO-101 联合仿真
│   │   │   │   ├── models/custom_quad_333/
│   │   │   │   │   ├── custom_quad_333.sdf         # 联合模型源文件
│   │   │   │   │   ├── custom_quad_333.sdf.jinja   # 生成参考源
│   │   │   │   │   ├── model.config
│   │   │   │   │   └── meshes/                     # 四旋翼网格
│   │   │   │   ├── launch/arm_uav_joint.launch.py
│   │   │   │   ├── scripts/                        # SDF 发布、PX4 起飞
│   │   │   │   ├── urdf/                           # 独立 SO-101 Gazebo 包装层
│   │   │   │   ├── tests/
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── package.xml
│   │   │   │   └── README.md
│   │   │   ├── uav_control/src/uav_control/        # 可选 UAV 控制节点
│   │   │   ├── models/real_uav/                    # 旧的独立 UAV 模型，不是联合模型
│   │   │   └── README.md
│   │   ├── Jetson_Setup.md                         # Jetson 刷机与基础环境参考
│   │   └── .gitignore
│   ├── SO101_COMPLETE/                             # 工作区外的历史来源/归档，不是必需源码
│   ├── SO-100-arm/                                 # 另一套历史仓库，不要与本项目混建
│   ├── arm_urdf/                                   # 旧的重复包，不要迁移
│   ├── so-101_description/                         # 旧的重复包，不要迁移
│   ├── uav_arm_delivery/                           # 历史交付资料
│   └── 历史报告与 *.gz 压缩包                      # 可不迁移
├── vendor/lerobot/                                 # 工作区外部 LeRobot 源码，不在 Git 仓库内
├── environment-haiying.yml                         # 本机 Conda 环境骨架，不在 Git 仓库内
├── README.md                                       # 工作区集成说明
├── build*/                                         # colcon 生成，不迁移
├── install*/                                       # colcon 生成，不迁移
└── log*/                                           # colcon/Gazebo 日志，可选备份
```

`src/arm_urdf` 和 `src/so-101_description` 是当前开发机上仓库外的旧副本；新主机如果
把它们与 `HaiYing-System-V1` 一起放入同一个 colcon 搜索路径，可能造成重复包名或使用
错误模型。新主机只克隆 `HaiYing-System-V1` 即可。

## 3. 删除旧工作区前的保存事项

### 3.1 先把 Git 提交推到远程

在旧主机执行，确保交接文档和最后的代码不会随本地目录一起消失：

```bash
cd /path/to/old/arm_ws/src/HaiYing-System-V1
git status --short
git branch --show-current
git log -1 --oneline
git push origin main
```

如果新文档或最后修改尚未提交，先提交后再执行 `git push origin main`。新主机获取后
用 `git log -1 --oneline` 确认已经拿到最新提交。

### 3.2 备份机械臂校准文件

校准文件位于用户主目录缓存，不属于 `arm_ws`，也不应提交 Git。当前配置使用：

```text
类型：so101_follower_5dof
校准 ID：jiebang_follower_arm
串口：/dev/ttyACM0
文件：~/.cache/huggingface/lerobot/calibration/robots/so101_follower_5dof/jiebang_follower_arm.json
```

旧主机先检查并复制到安全介质或安全的主机间传输位置：

```bash
test -f ~/.cache/huggingface/lerobot/calibration/robots/so101_follower_5dof/jiebang_follower_arm.json
mkdir -p /path/to/private-backup
cp -p ~/.cache/huggingface/lerobot/calibration/robots/so101_follower_5dof/jiebang_follower_arm.json \
  /path/to/private-backup/
chmod 600 /path/to/private-backup/jiebang_follower_arm.json
```

如果换了机械臂、舵机、机械安装姿态或校准 ID，不要直接复用旧 JSON，应在新主机上
重新校准。校准和首次点动都必须空载、低速，并保持急停或断电手段可用。

### 3.3 不要迁移的内容

- `build/`、`build-v7/`、`install/`、`install-v7/`、`log/` 等编译产物；
- `src/arm_urdf`、`src/so-101_description` 等仓库外的旧重复包；
- LeRobot Conda 环境目录、外部 `vendor/lerobot` 源码、模型权重、数据集和临时缓存；
- 任何口令、SSH 私钥、校准 JSON 的 Git 提交。

`Jetson_Setup.md` 只作为 Jetson Orin/JetPack/Ubuntu 基础配置参考，其中的初始化
凭据示例不能直接沿用；新主机应使用单独生成的账户、密码和 SSH 密钥，并在首次登录
后修改口令。

## 4. 新主机基础环境

当前项目按 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11 和 Python 3.10/3.12
组织。Jetson 目标按 `Jetson_Setup.md` 的 JetPack 6.0 / Ubuntu 22.04 方案准备；
具体 JetPack 版本以目标设备实际兼容矩阵为准。

### 4.1 克隆仓库

```bash
mkdir -p /path/to/arm_ws/src
cd /path/to/arm_ws/src
git clone git@github.com:woyebuzhidaocaonm-gif/HaiYing-System-V1.git
cd /path/to/arm_ws
git -C src/HaiYing-System-V1 switch main
```

如果新主机尚未配置 GitHub SSH key，可先配置后使用上面的 SSH 地址；也可使用同一
仓库的 HTTPS 地址克隆。不要把个人访问令牌写入脚本或文档。

### 4.2 安装 ROS 和系统依赖

先按目标 Ubuntu/JetPack 安装 ROS 2 Humble，然后安装项目运行所需的常用包：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip python3-pytest python3-lxml \
  ros-humble-ros-base \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rqt-image-view
```

初始化 rosdep（已初始化过的系统跳过 `rosdep init`）：

```bash
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/HaiYing-System-V1/control \
  src/HaiYing-System-V1/simulation --ignore-src -r -y
```

`python3-lxml` 是 `gazebo_ros/spawn_entity.py` 的实际运行依赖。缺少它会出现
`ModuleNotFoundError: No module named 'lxml'`，导致实体一直停在
`Waiting for service /spawn_entity`。

启用 MAVROS/PX4 前再安装 GeographicLib 数据：

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

## 5. ROS 2 构建与验证

### 5.1 查看应发现的包

```bash
cd /path/to/arm_ws
source /opt/ros/humble/setup.bash
colcon list --base-paths src/HaiYing-System-V1/control \
  src/HaiYing-System-V1/simulation --names-only
```

当前正式源码应能发现以下包：

```text
arm
arm_uav_joint
attitude_cmd
haiying_zhixun_bridge
so-101_description
uav_control
```

如果同时出现另一个 `arm_urdf` 或另一个 `so-101_description`，说明把旧副本放进了
colcon 搜索路径，应先移出搜索路径。

### 5.2 构建全部核心 ROS 包

```bash
cd /path/to/arm_ws
source /opt/ros/humble/setup.bash
colcon build \
  --base-paths src/HaiYing-System-V1/control src/HaiYing-System-V1/simulation \
  --symlink-install
source install/setup.bash
```

仅验证联合仿真及其依赖时：

```bash
colcon build \
  --base-paths src/HaiYing-System-V1/simulation \
  --packages-up-to arm_uav_joint --symlink-install
source install/setup.bash
```

基础验证命令：

```bash
ros2 pkg prefix arm_uav_joint
ros2 launch arm_uav_joint arm_uav_joint.launch.py --show-args
colcon test --packages-select arm_uav_joint
colcon test-result --verbose
```

验证结果应至少包括：`arm_uav_joint` 能被发现、launch 参数可解析、包测试通过。

桥接包的测试会导入 LeRobot 适配层，必须在 `haiying` 环境执行；系统 Python 中没有
LeRobot 属于正常的环境隔离：

```bash
conda run --no-capture-output -n haiying \
  python -m pytest -q src/HaiYing-System-V1/control/haiying_zhixun_bridge/tests
```

## 6. 联合仿真操作

### 6.1 默认静态展示

默认不加载 PX4 飞行插件，并把组合模型固定在空中，用于先检查 Gazebo、四旋翼网格、
挂载位置和机械臂几何：

```bash
cd /path/to/arm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_uav_joint arm_uav_joint.launch.py \
  gui:=false use_rviz:=false use_camera_view:=false
```

需要桌面显示时去掉 `gui:=false`、`use_rviz:=false`。当前入口已经在实际 Gazebo
无 GUI 探测中成功生成 `custom_quad_333_display` 并解除暂停。

### 6.2 PX4 SITL + MAVROS

只有目标主机已安装并构建 PX4 Classic SITL 时才使用。必须显式关闭静态模型，并把
路径替换为新主机实际路径：

```bash
ros2 launch arm_uav_joint arm_uav_joint.launch.py \
  use_px4_plugins:=true custom_quad_static:=false auto_takeoff:=false \
  px4_autopilot_dir:=/path/to/PX4-Autopilot \
  px4_build_dir:=/path/to/PX4-Autopilot/build/px4_sitl_default \
  gui:=false use_rviz:=false use_camera_view:=false
```

自动起飞只在完成手动 SITL 检查后启用：

```bash
ros2 launch arm_uav_joint arm_uav_joint.launch.py \
  use_px4_plugins:=true custom_quad_static:=false \
  auto_takeoff:=true handoff_to_qgc:=true \
  px4_autopilot_dir:=/path/to/PX4-Autopilot \
  px4_build_dir:=/path/to/PX4-Autopilot/build/px4_sitl_default \
  gui:=false use_rviz:=false use_camera_view:=false
```

重要限制：当前 PX4 模式为飞行链路验证配置，发布脚本会将内嵌 SO-101 的五个关节
固定，避免组合 SDF 与独立 `gazebo_ros2_control` 重复控制。因此当前版本不能宣称
“飞行过程中可控制机械臂五轴”；机械臂实体控制仍走下面的 LeRobot/桥接链路。

### 6.3 单独调试 SO-101

如果只测试原有机械臂 Gazebo + MoveIt + ros2_control：

```bash
ros2 launch arm_uav_joint arm_uav_joint.launch.py \
  show_custom_quad:=false show_so101_arm:=true \
  use_px4_plugins:=false custom_quad_static:=true
```

联合模式下不要再单独生成 `simulation/models/real_uav/iris.sdf`，也不要再生成一个
独立 SO-101，否则 Gazebo 中会出现重复实体。`real_uav` 只是旧的独立 UAV 模型，
不包含当前 `custom_quad_333.sdf` 中的挂载机械臂。

## 7. `haiying` Python 环境与 LeRobot

### 7.1 环境边界

当前工作区外的 `environment-haiying.yml` 只定义了 `haiying` 环境的基础框架：
Python 3.12、pip、`PYTHONPATH=""` 和 `PYTHONNOUSERSITE="1"`；它不携带 LeRobot
源码或校准文件。新主机应重新创建环境：

```bash
conda create -n haiying python=3.12 -y
conda activate haiying
python -m pip install --upgrade pip
```

本项目已验证的关键版本约束：

| 依赖 | 版本约束 | 用途 |
|---|---:|---|
| Python | `>=3.12` | LeRobot/实机服务运行时 |
| `lerobot` | `==0.6.1` | SO-101 follower、IK、校准 |
| `draccus` | `>=0.11.6,<0.12` | 配置解析 |
| `feetech-servo-sdk` | `>=1.0,<2` | STS3215 舵机通信 |
| `pyserial` | `>=3.5,<4` | `/dev/ttyACM0` 串口 |
| `numpy` | `>=2.0,<2.3` | 数值计算 |
| `huggingface-hub` | `>=1.6,<2` | LeRobot 校准目录 |
| `deepdiff` | `>=7,<9` | 配置/校准依赖 |
| `PyYAML` | `>=5.4` | 桥接配置 |
| `tqdm` | `>=4.66,<5` | LeRobot CLI 依赖 |

优先使用外部 LeRobot 源码；如果新主机没有该源码，可安装对应版本发布包：

```bash
conda activate haiying
python -m pip install -e /path/to/lerobot
# 或：python -m pip install "lerobot[feetech]==0.6.1"
python -m pip install -e /path/to/arm_ws/src/HaiYing-System-V1/control/haiying_zhixun_bridge --no-deps
```

`--no-deps` 是为了不把 ROS 2 Python 依赖混入 Python 3.12；如环境中没有对应依赖，
按上表和仓库内 `control/haiying_zhixun_bridge/README.md` 补齐。

检查 LeRobot 侧命令，不会打开串口：

```bash
conda activate haiying
python -c 'import lerobot, draccus, serial; from lerobot.robots.so_follower import SO101Follower5DOF; print("SO-101 LeRobot API: OK")'
python -m pip check
command -v lerobot-ik-sim lerobot-ik-real lerobot-calibrate haiying-moveit-real-server
```

ROS 终端不要激活 `haiying`；LeRobot/实机终端不要用系统 Python 直接启动实机服务。

## 8. 机械臂桥接操作顺序

### 8.1 启动 IK 服务

终端 A 使用普通 shell 或系统 ROS shell，不需要激活 `haiying`：

```bash
cd /path/to/arm_ws/src/HaiYing-System-V1
./control/haiying_zhixun_bridge/scripts/start_ik_server.sh
```

脚本会自动执行 `conda run -n haiying lerobot-ik-sim`，服务监听
`http://127.0.0.1:8766`，默认使用仓库内 `simulation/arm` 的 SO-101 模型，
不访问串口。健康检查：

```bash
curl http://127.0.0.1:8766/api/health
```

### 8.2 规划和 dry-run

终端 B 激活 `haiying`，规划后保存输出的 `plan_id`：

```bash
cd /path/to/arm_ws/src/HaiYing-System-V1
conda activate haiying
python control/haiying_zhixun_bridge/scripts/plan_target.py \
  --x 0.005534 --y -0.179839 --z 0.171219
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  dry-run --plan-id <PLAN_ID>
```

`dry-run` 检查轨迹、碰撞、帧间步长和执行锁，不打开串口。

### 8.3 ROS 2 桥接和 MoveIt 仿真

终端 C 使用系统 ROS Python，不激活 `haiying`：

```bash
cd /path/to/arm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run haiying_zhixun_bridge haiying-arm-bridge-node
```

需要 MoveIt/Gazebo 实机场景联调时，另开终端启动：

```bash
cd /path/to/arm_ws/src/HaiYing-System-V1
./control/haiying_zhixun_bridge/scripts/start_moveit_real_server.sh
```

再在系统 ROS 终端运行：

```bash
ros2 launch haiying_zhixun_bridge moveit_real_gui.launch.py
```

服务启动和轨迹验证阶段不应连接串口；最终执行仍必须经过 GUI 确认和硬件安全开关。

### 8.4 ROS 接口约定

接口定义以 `docs/ROS2_Interface_V1.md` 为准，当前关键接口如下：

| 方向 | Topic | 消息类型 | 规则 |
|---|---|---|---|
| 订阅 | `/system/current_state` | `std_msgs/msg/String` | 只有 `BRUSHING` 接受目标 |
| 订阅 | `/arm/target_pose` | `geometry_msgs/msg/PoseStamped` | `frame_id` 必须为 `base_footprint` |
| 订阅 | `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | GUI 规划轨迹 |
| 订阅 | `/joint_states` | `sensor_msgs/msg/JointState` | 终点与反馈验证 |

当前五轴方向映射为 `[+1,+1,-1,+1,+1]`，零偏和安全阈值位于
`control/haiying_zhixun_bridge/config/arm_bridge.yaml`。更换机械臂或调整安装后，
不要只改单个脚本；需要同步检查 YAML 和 `lerobot_adapter.py`。

## 9. 首次接入实体机械臂

先确认 USB 串口及权限：

```bash
ls -l /dev/ttyACM0
sudo usermod -aG dialout "$(id -un)"
```

加入 `dialout` 后重新登录系统。首次使用或更换机械臂时重新校准：

```bash
conda activate haiying
lerobot-calibrate \
  --robot.type=so101_follower_5dof \
  --robot.port=/dev/ttyACM0 \
  --robot.id=jiebang_follower_arm
```

推荐首次动作顺序：

```bash
# 只读反馈检查，会连接舵机但不发送目标位置
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  --allow-hardware inspect

# 只做单轴 1° 点动
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  --allow-hardware jog --joint shoulder_pan --delta-deg 1

# 只执行已经 dry-run 且现场确认的轨迹
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  --allow-hardware execute --plan-id <PLAN_ID> --confirm-execute
```

`--allow-hardware` 与 `--confirm-execute` 不得省略。首次接入必须空载、小角度、低速，
操作者要能随时断电；出现串口、校准、起始姿态、反馈误差或碰撞异常时立即停止。

## 10. 当前验证边界和未完成事项

已验证的内容：

- `arm_uav_joint` 能被 colcon 发现并构建；
- `custom_quad_333.sdf` XML、联合链接/关节和四旋翼网格完整；
- SO-101 Xacro 可展开并通过 `check_urdf`；
- 默认无 GUI Gazebo 能生成联合模型并解除暂停；
- 联合包资源测试和 launch 参数解析通过。

尚未由这份交接说明替代的验收：

- 新主机上的 PX4 SITL 实际起飞、MAVROS 链路和 QGroundControl 交接；
- 新 Jetson 的 GPU/相机/YOLO 性能和驱动适配；
- 新主机连接实体机械臂后的校准、1° 点动和完整轨迹验收；
- 无人机挂载机械臂后的真实质量、质心、惯量、摩擦和安全限位标定；
- 飞行过程中五轴机械臂控制。当前 PX4 联合模式固定机械臂关节，不能把静态展示或
  PX4 SITL 启动结果当作实体联合控制验收。

遇到问题时先执行：

```bash
source /opt/ros/humble/setup.bash
source /path/to/arm_ws/install/setup.bash
ros2 pkg prefix arm_uav_joint
ros2 pkg prefix so-101_description
ros2 pkg prefix haiying_zhixun_bridge
```

然后确认没有旧工作区的 `install/setup.bash` 被错误地 source，确认 `lxml`、LeRobot
环境、校准文件和 `/dev/ttyACM0` 分别属于正确的运行环境。

## 11. 详细资料索引

- 仿真总览：`simulation/README.md`
- 联合仿真操作：`simulation/arm_uav_joint/README.md`
- 桥接与 LeRobot 依赖：`control/haiying_zhixun_bridge/README.md`
- ROS 2 对接：`control/haiying_zhixun_bridge/docs/ROS2_INTEGRATION.md`
- IK 到实机：`control/haiying_zhixun_bridge/docs/LOCAL_IK_TO_REAL.md`
- MoveIt 图形交接：`control/haiying_zhixun_bridge/docs/MOVEIT_REAL_GUI.md`
- Jetson 基础环境：`Jetson_Setup.md`
