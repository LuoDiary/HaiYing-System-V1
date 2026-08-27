# arm_uav_joint 联合仿真迁移设计

## 1. 目标

从 `src/SO101_COMPLETE/V7_SO101_SEND_TO_TEAMMATE` 中筛选并整理 SO-101 机械臂与
custom quad 联合仿真所需内容，在 `HaiYing-System-V1/simulation` 下新增一个可由
`colcon` 发现和构建的 `arm_uav_joint` ROS 2 包。

联合仿真需要支持以下两类模式：

1. custom quad 搭载 SO-101 作为一个动力学组合体，在 Gazebo Classic/PX4 SITL 中
   进行飞行验证；
2. 可选地单独生成原始 SO-101，用于 Gazebo 控制器、MoveIt 和 RViz 调试。

联合仿真包不承担实体机械臂控制，也不把 PX4、MAVROS、QGroundControl 或 LeRobot
源码提交进仓库。

## 2. 现状与边界

源目录中已经包含一套完整的 V7 联合仿真实现，但混有备份、缓存和绝对路径。目标
仓库中已有：

- `simulation/arm`：轻量、自包含的机械臂模型；
- `simulation/so-101_description`：MoveIt、Gazebo、ros2_control 和相机仿真包；
- `simulation/models/real_uav` 与 `simulation/launch/offshore.launch.py`：较早的
  无人机仿真内容，路径硬编码，暂不在本次迁移中覆盖或删除。

本次以目标仓库现有 `so-101_description` 的机械臂宏、控制器和 MoveIt 配置为准，
不复制源目录中已有的机械臂 STL、宏文件、SRDF 或通用控制器配置。

## 3. 新包目录

新增目录为：

```text
simulation/arm_uav_joint/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── arm_uav_joint.launch.py
├── models/
│   └── custom_quad_333/
│       ├── model.config
│       ├── custom_quad_333.sdf
│       ├── custom_quad_333.sdf.jinja
│       └── meshes/
│           ├── iris.stl
│           ├── iris_prop_ccw.dae
│           └── iris_prop_cw.dae
├── scripts/
│   ├── publish_custom_quad_display.py
│   └── px4_takeoff.py
└── urdf/
    └── so101_arm_uav_gazebo.urdf.xacro
```

### 3.1 迁移内容

- `custom_quad_333.sdf`：包含 custom quad、0.45 kg 载荷和嵌入式 SO-101 几何、质量、
  惯量及安装固定关节的最终 Gazebo 模型；
- `custom_quad_333.sdf.jinja`：保留为模型生成源，方便后续重新生成 SDF；
- `model.config` 和三份四旋翼网格：保证 Gazebo 模型资源可独立定位；
- `so101_arm_uav_gazebo.urdf.xacro`：单独 SO-101 的 Gazebo ros2_control 包装入口；
- `publish_custom_quad_display.py`：发布并清理 custom quad SDF，支持静态展示和
  PX4 飞行插件模式；
- `px4_takeoff.py`：保留 PX4 起飞、悬停和可选 QGroundControl 交接逻辑；
- 联合启动文件：由新包拥有，避免与现有 `so-101_description` 的同名启动文件
  冲突。

### 3.2 排除内容

不迁移以下内容：

- `BEFORE_*`、`backup_*`、`last_generated` 等快照和备份；
- `__pycache__`、`.pyc`；
- 已存在于目标仓库的机械臂网格、机械臂宏、MoveIt 配置、控制器配置和世界文件；
- 不参与联合仿真的 `initialize_joint_controller.py`、风机场景脚本以及重复的普通
  SO-101 启动入口。

## 4. 依赖与路径规则

`arm_uav_joint` 依赖目标仓库现有的 `so-101_description`，复用其：

- `so101_arm_macro.urdf.xacro`；
- `config/ros2_controllers.yaml`；
- MoveIt 配置和规划参数；
- Gazebo 世界与机械臂模型网格。

新包自身安装 `models`、`urdf`、`launch` 和 `scripts`。启动文件通过
`ament_index_python` 获取包安装路径，并将新包的 `models` 目录加入
`GAZEBO_MODEL_PATH`；不依赖当前用户的桌面目录或仓库绝对路径。

源 SDF 中的以下路径必须在迁移时修正：

- `/home/ljj/HaiYing-System-V1-so101-control/...` 改为目标包共享目录或启动时解析的
  `so-101_description` 共享目录；
- `model://custom_quad_333/...` 指向新包的 `models/custom_quad_333`；
- `model://so-101_description/...` 保持指向目标仓库已存在的
  `so-101_description` 资源；
- PX4 源码和构建目录通过 `PX4_AUTOPILOT_DIR`、`PX4_BUILD_DIR` 或 launch 参数指定。

## 5. 运行语义

默认启动 custom quad 联合仿真，不额外生成独立机械臂。组合体中的 SO-101 作为无人机
载荷参与视觉、碰撞、质量和惯量计算；飞行模式下不应再同时生成一份独立 SO-101。

独立机械臂模式通过 `show_so101_arm:=true` 开启，主要用于控制器和 MoveIt 调试。
PX4 模式通过 `use_px4_plugins:=true` 开启；自动起飞默认关闭，只有显式设置
`auto_takeoff:=true` 才启动起飞节点。所有模式都必须提供 headless 参数，便于 Jetson
或 CI 环境运行。

README 将说明：

- Ubuntu 22.04、ROS 2 Humble、Gazebo Classic、MoveIt 2、PX4 SITL、MAVROS 的外部
  安装要求；
- 构建、source 和包发现命令；
- 仅展示、独立机械臂、custom quad、PX4 自动起飞和 QGC 交接的命令；
- `custom_quad_x/y/z`、`custom_quad_static`、`show_custom_quad`、`show_so101_arm`
  等关键参数；
- 模型质量、挂载位姿、悬停推力和仿真结果不等同于实体飞行安全参数。

## 6. 验证方案

实现后按以下顺序验证：

1. `colcon list --base-paths simulation` 能发现 `arm_uav_joint`，且无重复包名；
2. 构建 `arm_uav_joint` 及其目标仓库依赖；
3. 展开 `so101_arm_uav_gazebo.urdf.xacro`，检查 XML、link/joint 和网格路径；
4. 解析 custom quad SDF，检查模型、嵌入式机械臂安装关节、质量载荷和模型 URI；
5. 对 Python 脚本执行语法检查，扫描源文件中不得出现 `/home/ljj`、`桌面/` 等硬编码
   路径；
6. 使用 `ros2 launch ... --show-args` 检查启动参数；
7. 在具备 Gazebo、PX4 和 MAVROS 的环境中，再进行实际 SITL 启动、起飞和 QGC 交接
   验证。缺少这些外部依赖时，只报告静态和构建验证结果，不宣称联合仿真已运行验收。

## 7. 不在本次范围内的工作

- 修改实体机械臂 LeRobot 控制链路；
- 重新设计 SO-101 机械臂运动学、惯量或安装位姿；
- 删除现有 `simulation/models/real_uav` 和 `simulation/launch/offshore.launch.py`；
- 自动安装 PX4、MAVROS、Gazebo、QGroundControl 或 LeRobot；
- 在没有操作者确认和真实硬件条件的情况下连接或驱动实体机械臂。
