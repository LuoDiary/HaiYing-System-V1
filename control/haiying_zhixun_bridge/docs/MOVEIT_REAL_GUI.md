# MoveIt 仿真轨迹到 SO-101 实机

本文只覆盖当前阶段的机械臂链路，不包含无人机协同。整体结构如下：

```text
RViz 目标 -> MoveIt Plan & Execute -> Gazebo /joint_states
                |
                +-> /display_planned_path -> 海鹰实机控制 GUI
                                              |
                                    localhost:8767（轨迹验证/执行）
                                              |
                                  LeRobot Python 3.12 -> /dev/ttyACM0
```

ROS 2 Humble、MoveIt、Gazebo 和 GUI 使用系统 Python 3.10；LeRobot 实机服务使用
`haiying` Conda 环境的 Python 3.12。两侧只通过本机 HTTP 通信，避免 Conda 覆盖 ROS 的
Python 环境。正式五轴顺序为 `J1_Rotation`、`J2_Shoulder_Pitch`、
`J3_Elbow_Pitch`、`J4_Wrist_Pitch`、`J5_Wrist_Roll`；MoveIt group 为 `arm`，根坐标为
`base_footprint`，TCP 为 `end_effector`。

## 1. 启动

终端 A 启动实机服务：

```bash
cd <HaiYing-System-V1 工作空间>
./control/haiying_zhixun_bridge/scripts/start_moveit_real_server.sh
```

服务只监听 `127.0.0.1:8767`。启动时不会打开串口；只有 GUI 最终确认执行后才连接
`/dev/ttyACM0`。
GUI 对健康检查和轨迹验证等待 10 s；对实体轨迹执行等待 180 s，因为执行接口会在
整条轨迹、起始对齐和末端稳定完成后才返回结果。

终端 B 启动 Gazebo、MoveIt、RViz 和实机控制 GUI：

```bash
cd <HaiYing-System-V1 工作空间>
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch haiying_zhixun_bridge moveit_real_gui.launch.py
```

## 2. 操作顺序

1. 在 RViz 的 MotionPlanning 面板选择 planning group `arm`。
2. 将起点设为当前状态，设置一个保守且工作区无障碍的目标。默认速度和加速度缩放
   均为 `0.2`；首次实机验收只规划单关节 `5°～10°` 小动作。
3. 点击 **Plan & Execute**，等待 RViz 与 Gazebo 中机械臂到达相同终点。
4. GUI 应显示“已捕获 MoveIt 轨迹”和“Gazebo 终点：已到达”。
5. 点击“验证最新 MoveIt 轨迹（不连接实机）”。这一步仅检查关节名、限位、速度、
   时间、帧间步长和轨迹哈希，不打开串口。
6. 确认实体机械臂起始姿态与仿真起点一致，周围无障碍且可以立即断电，再勾选安全确认框。
7. 点击红色“控制实机到 Gazebo 已到达的位置”，阅读二次确认内容后再确认。

实机执行前还会重新读取五个关节。误差不超过 `20°` 时，服务先按速度和单帧限制
平滑对齐到 MoveIt 轨迹首帧，再执行正式轨迹；任一关节超过 `20°` 则拒绝运动。
执行期间以 50 Hz 发送重采样轨迹，限制单帧 `5°`、速度 `25°/s`；超过速度的
MoveIt 轨迹会在 5 倍范围内自动拉长时间。反馈判断补偿 `0.15 s` 舵机跟随延迟，
每 5 帧读取一次（10 Hz），普通关节允许 `8°`、`wrist_roll` 允许 `15°`，只有连续 5 次
超差才中止；结束后仍必须在 2 秒内稳定到终点容差。校准、人工确认和硬件互斥
保护保持不变。实际限位按 URDF 逐关节执行：J1/J4 为
`±89.954°`，J2 为 `-30.000°～166.158°`，J3 为 `0°～179.909°`，J5 为
`±179.909°`；实机服务校验在每个上下限外再保留 `1°` 数值容错，即 J1/J4 为
`±90.954°`，J2 为 `-31.000°～167.158°`，J3 为 `-1.000°～180.909°`，J5 为
`±180.909°`。该容错不改变 Gazebo/MoveIt 的 URDF 模型限位。执行成功后轨迹编号
作废，不会自动回程。
连接实机时会统一写入舵机稳定参数：位置环 PID `24/0/32`、加速度 `100`、死区 `5`。

## 3. 不连接实机的冒烟验证

先启动终端 A 和终端 B，然后执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run haiying_zhixun_bridge haiying-moveit-real-smoke
```

该命令会让 MoveIt 在 Gazebo 中 Plan & Execute，验证 Gazebo 终点，并调用实机服务的
`health` 和 `validate`；它不会调用 `execute`，因此不会连接或移动实体机械臂。

## 4. 故障定位

- `Address already in use`：8767 已有服务；启动脚本会复用健康的现有服务。可用
  `ss -ltnp '( sport = :8767 )'` 查看占用者。
- GUI 显示“执行已禁用”：当前 8767 服务不是由本项目启动脚本以显式硬件开关启动，
  停止旧进程后重新运行终端 A 的命令。
- “映射不一致”：停止执行，核对 YAML 与 LeRobot 服务返回的方向和零偏，不要绕过联锁。
- “Gazebo 尚未到达”：必须使用 **Plan & Execute**，并等待 `/joint_states` 到达轨迹终点。
- “首帧不匹配”：不要强行执行；先让仿真起点与实体当前姿态一致，重新规划并验证。
- 仿真或轨迹验证阶段报错：不会连接串口，也不会控制实体机械臂；修正后重新规划。
- 实机执行阶段反馈持续超差或串口异常：服务停止继续下发，在 `finally` 中断开机械臂；
  不会自动安全回收。立即准备断电，并检查机械结构、供电、串口和零偏后再测试。
- 运动过程中抖动：先确认使用本版本的 50 Hz 服务且 RViz 缩放为 `0.2`，不要通过
  连续点击执行或继续提高速度掩盖问题。

## 5. 当前校准映射

当前校准 ID 为 `jiebang_follower_arm`。MoveIt/URDF 到 LeRobot 的方向为
`[+1, +1, -1, +1, +1]`，零偏为
`[-0.175824, -0.747253, -0.527473, 16.967033, -11.208791]°`。
GUI 会逐项比对 YAML 和服务端映射；任何差异都会禁用实机按钮。
