# /uav/cmd_vel → PX4/H743 冻结链 — 正式接口规格 (Interface Spec V1.0)

> 适用提交: 核心仓库 `luodiary/port-freeze-cmd-vel-control-chain` @ `5354600` + 本轮接口修订
> (未提交)。测试环境: 远程 Ubuntu 24.04 + ROS2 Jazzy;目标联调: ROS2 Humble(见 §9)。
> 本文件回答 9 个正式接口问题;与 `docs/FROZEN_CONTROL_CHAIN.md`、`docs/ROS2_Interface_V1.md`
> 共同构成唯一契约。

---

## Q1. 最终联合仿真的唯一启动入口、节点列表、参数文件、启动顺序

**唯一入口**: `ros2 launch attitude_cmd freeze_chain.launch.py`
(源码 `control/ros-package/attitude_cmd/launch/freeze_chain.launch.py`,
参数 `control/ros-package/attitude_cmd/config/freeze_params.yaml`,
两者已注册进 `setup.py` data_files,随包安装)。

节点列表与启动顺序(launch 内固定):

| 顺序 | 节点 | 说明 |
|---|---|---|
| 1 | `fake_px4`(仿真) | 模拟飞控,SITL 替代;真机时改连 TELEM 串口并去掉本节点 |
| 2 | `attitude_cmd_node` | MAVLink 属主(唯一),UDP 回环 `udpin:14550` / 真机 `/dev/ttyS2` |
| 3 | `cmd_vel_to_attitude` | 唯一运动指令转换节点 |
| 4 | `approach_controller` | 唯一 `/uav/cmd_vel` 生产者(脚本路径 `scripts/approach_controller.py`) |

启动命令:

```bash
source /opt/ros/jazzy/setup.bash   # 或 humble
source <ws>/install/setup.bash
ros2 launch attitude_cmd freeze_chain.launch.py          # 正式: 链路+转换
ros2 launch attitude_cmd freeze_chain.launch.py use_fake_px4:=true   # 回环冒烟
```

**禁止并行**(与旧控制器互斥): `simulation/uav_control` 的 `takeoff.py` /
`vision_control.py`(MAVROS setpoint_position 控制)、MAVROS 进程、
`hover_demo_node`、手动 `ros2 topic pub /attitude_setpoint`、`offshore.launch.py`。
唯一速度控制器 = `approach_controller`(→转换节点→链路节点)。

## Q2. /uav/cmd_vel 坐标约定

**冻结约定: NED 世界系(North-East-Down)。**

| 字段 | 约定 | 单位 |
|---|---|---|
| `linear.x` | 北向速度 v_N | m/s |
| `linear.y` | 东向速度 v_E | m/s |
| `linear.z` | 下向速度 v_D(上升为负) | m/s |
| `angular.z` | 偏航角速度,NED 正方向 = 从上方俯视**顺时针**(绕 +z_down 右手系),右转正 | rad/s |
| `angular.x/y` | 忽略(必须为 0) | — |

说明:
- **不是机体系**: 转换节点按 NED 世界系解析,与 `vehicle_velocity`(LOCAL_POSITION_NED)反馈同系;
- **ENU↔NED**: `v_ned = (v_enu.y, v_enu.x, −v_enu.z)`,`ωz_ned = −ωz_enu`;
- 视觉链(`/vision/target_point`、`/mavros/local_position/pose`)为 **world/ENU**;
  `approach_controller`(当前唯一生产者)在 ENU 中计算,发布前按上式转 NED
  (已实现:`scripts/approach_controller.py::_publish_cmd_vel`);
- **mission_fsm_node**(仓库中尚无此节点,待决策组实现)应**按 NED 发布**
  `/uav/cmd_vel`;若其输入为 ENU,发布前做同样转换。

## Q3. 速度 → 四元数/推力 正式数学公式

实现: `control/ros-package/attitude_cmd/attitude_cmd/cmd_vel_to_attitude.py`
(50 Hz)。坐标系: 输入 NED;姿态 FRD(四元数 w,x,y,z);推力归一化 0~1。

1. 速度误差:`e_v = v_cmd − v_est`(`v_est` 来自 `/vehicle_velocity`,
   即 MAVLink `LOCAL_POSITION_NED.vx,vy,vz`)
2. 期望加速度(比例环): `a_des = K_v · e_v`,`K_v = vel_gain = 1.8`
   (对应 PX4 `MPC_XY_VEL_P_ACC`)
3. 限幅(饱和):
   - `|a_xy| ≤ max_accel = 2.0 m/s²`(等比缩放)
   - `|a_z| ≤ 2.0 m/s²`(对称钳制)
4. 期望机体 Z 轴(推力反方向,向上,NED):
   `body_z = normalize([−a_x, −a_y, g − a_z])`,`g = 9.81`
   (与 PX4 `PositionControl.cpp:217` 一致)
5. 倾角饱和: `angle(body_z, [0,0,1]) ≤ max_tilt = 0.5 rad`(≈28.6°)
6. 航向: `ψ = (ψ + ωz·dt) mod 2π`,dt = 20 ms
7. 四元数构造(与 PX4 `ControlMath::bodyzToAttitude` 逐行一致):
   - `y_C = (−sinψ, cosψ, 0)`,`body_x = y_C × body_z`(倒置修正),
     `body_y = body_z × body_x`
   - 旋转矩阵 `R_sp = [body_x | body_y | body_z]`(机体系→NED,列 = 机体轴在
     NED 中的坐标)→ 四元数 `q_d`
   - **欧拉构造顺序: Z-Y-X(yaw → pitch → roll)**;小角度(悬停附近):
     `pitch ≈ −a_x/g`(负俯仰 = 机头下沉,向北加速)、`roll ≈ a_y/g`
     (正横滚 = 右翼下沉,向东加速)
8. 推力(方向: 沿 −body_z,即 NED 向上):
   `thrust = (g − a_z) · hover_thrust / g`,`hover_thrust = 0.5`
   (对应 PX4 `MPC_THR_HOVER`;`PositionControl.cpp:220`)
   饱和: `thrust ∈ [thrust_min, thrust_max] = [0.1, 0.9]`
9. 积分/抗饱和: **本节点无积分器**(纯速度 P 环),无积分饱和问题;PX4
   姿态环的积分/ARW 由固件内部处理,不受本层影响。
10. 非 ACTIVE 态(超时/FAULT): 输出水平 `[1,0,0,0, hover_thrust]`,
    即"零速度保持"指令。

## Q4. 速度反馈超时(>1 s)行为 — 正式安全行为

**结论: 原"置零估计并继续控制"** 仅作联调临时行为,**不作为正式安全行为**;
本轮已改为:

- 反馈超时(`vel_feedback_timeout = 1.0 s`)且收到有效 cmd_vel 时:
  `cmd_state = FAULT`,输出水平悬停设定值,**不再生成基于置零估计的控制量**
  (`cmd_vel_to_attitude.py:_update`);
- `attitude_cmd_node` 将 `cmd_state ∈ {HOLD, FAULT}` 同等对待 → 进入
  SAFETY_HOLD(停止转发、内部水平 + hover_thrust),并按 §7 阶梯继续;
- 恢复: 反馈恢复 1 s 内 → `ACTIVE`(自动恢复)。

远程已验证: 断开链路节点后 1 s 内 `cmd_state=FAULT`,设定值锁定水平。

## Q5. /mavros/local_position/pose — NED→ENU 是否完成?

**结论: 已完成 NED→ENU 转换,不再是纯话题别名。** 实现于
`mavlink_link_node.py`(收到 `LOCAL_POSITION_NED` 时):

- 位置: `p_enu = (p_ned.y, p_ned.x, −p_ned.z)`
- 姿态: `q_enu = q_frame ⊗ q_ned`,`q_frame = (0, √2/2, √2/2, 0)`
  (即绕 NED 的 (1,1,0)/√2 轴旋转 180°,其旋转矩阵恰为 ENU 映射矩阵
  `[[0,1,0],[1,0,0],[0,0,−1]]`;数值已核对)
- `header.frame_id = "map"`;时间戳 = 接收时刻
- 同一数据源同时发布: `/vehicle_local_position`(PointStamped,NED)、
  `/vehicle_velocity`([vx vy vz],NED,供转换节点)

远程已验证: fake 飞控 NED(x=0.143, y=−0.071, z=−1) → pose (x=−0.071,
y=0.143, z=+1.0),姿态 = (0.7071, 0.7071, 0, 0)(水平)。

## Q6. 故障上报职责表(/system/current_state=ERROR)

`/system/current_state` 现为**双发布者**: 决策层(approach_controller)发布任务
状态;链路层(`attitude_cmd_node`,新增发布器)只发布故障状态。职责表:

| 故障 | 上报节点(→/uav/flight_fault) | 触发条件 | 状态 |
|---|---|---|---|
| MAVLink 断开 | `attitude_cmd_node` | 3 s 无 HEARTBEAT | **已实现**(远程验证 ERROR) |
| 遥测超时 | `attitude_cmd_node` | 1 s 无 LOCAL_POSITION_NED | **已实现** |
| 姿态指令发送失败 | `attitude_cmd_node` | `set_attitude_target_send` 抛异常 | **已实现**(try/except → ERROR) |
| OFFBOARD 拒绝 | `attitude_cmd_node` | `COMMAND_ACK result≠0`(DO_SET_MODE) | **已实现**(ACK result≠0 → ERROR) |
| 解锁拒绝 | `attitude_cmd_node` | `COMMAND_ACK result≠0`(ARM_DISARM) | 同上(已实现) |
| 速度反馈超时 | `cmd_vel_to_attitude` | §4 → `cmd_state=FAULT`(链路上游据此进入 HOLD) | **已实现** |

恢复语义: 故障恢复后链路节点静默(不发布 NORMAL,避免与决策层状态竞争),
`_err_published` 复位可再次上报。**注**: 链路节点只上报 ERROR;任务级状态
(SEARCHING 等)仍由决策层负责。

## Q7. 超时优先级与触发次序(冻结)

| 次序 | 事件 | 判定/执行节点 | 动作 |
|---|---|---|---|
| ① | `/uav/cmd_vel` 3 s 无新指令 | `cmd_vel_to_attitude` | `cmd_state=HOLD`,输出水平+悬停推力(零速度保持,链路存活) |
| ② | `cmd_state` 丢失 5 s(`cmd_state_timeout`) | `attitude_cmd_node` | 视同 HOLD → SAFETY_HOLD |
| ③ | 速度反馈 1 s 超时(§4) | `cmd_vel_to_attitude` | `cmd_state=FAULT` → 链路节点 SAFETY_HOLD |
| ④ | `/system/current_state=ERROR` | `attitude_cmd_node` | SAFETY_HOLD |
| ⑤ | SAFETY_HOLD 持续 3 s(`hold_mode_timeout`) | `attitude_cmd_node` | 按 `error_mode_action` 发 `MAV_CMD_DO_SET_MODE`(p1=1,p2=主,p3=子): 0=保持 offboard,1=**Hold**(p2=4,p3=3),2=POSCTL(p2=3,p3=0),3=RTL(p2=4,p3=5),4=Land(p2=4,p3=6) → SAFETY_SWITCHED(停止发布) |
| ⑥ | MAVLink 链路丢失 1 s | PX4 固件 | `COM_OF_LOSS_T` → `COM_OBL_RC_ACT=0` = Position 悬停(最终兜底) |

**ERROR/零速度 vs 模式切换优先级**: 零速度保持(水平+hover_thrust)是
**过渡态**,从触发起立即执行并持续至模式切换;模式切换是**终端动作**
(SAFETY_SWITCHED 后链路节点停止发布,不再回退);PX4 原生兜底最后生效。
ERROR 持续期间若 3 s 内恢复(NOMINAL/任务状态),回到 NORMAL,不切换模式。

## Q8. udpin:14550 端口所有权与冲突

- **所有权**: 该端口属冻结链**回环测试专用**(`attitude_cmd_node` 绑定
  `0.0.0.0:14550`,`fake_px4` 以 `udpout:127.0.0.1:14550` 回连);真机路径走
  TELEM 串口,不使用 UDP。
- **冲突**: 同机运行的 QGC、PX4 SITL、V9.2 仿真或其他 MAVLink 客户端若绑定
  /连接 14550 会冲突(只能一端绑定)。**规则**: 冻结链回环独占 14550;需要
  与 QGC 同机共存时,改 `freeze_params.yaml` 中 `port` 为其它端口(如
  `udpin:14551`)并同步改 `fake_px4.py` 的 `udpout:127.0.0.1:14551`;或链路
  走串口、QGC 走 USB/其它链路。
- V9.2 仿真: 其 PX4 SITL 默认占用 14550,与回环链冲突——联调时二者不可同机
  同端口,建议 V9.2 的 MAVLink 改走其它端口或由冻结链 fake_px4 替代。

## Q9. 实测版本、命令与测试输出

**实测环境**(远程 192.168.137.128):
- OS / ROS: Ubuntu 24.04 / **ROS2 Jazzy**(ros-jazzy-ros-base)
- Python 依赖: pymavlink 2.4.49、pyserial 3.5、matplotlib 3.6.3
- 固件: PX4 fork v1.18 系(`PX4-Autopilot` 分支
  `luodiary/freeze-cmd-vel-control-chain` @ `874c6847e0`;
  固件改动 = `mavlink_receiver.cpp/.h` 消息 82 处理 +
  `mavlink_main.cpp:1683` VIBRATION 1 Hz)
- 包版本: `attitude_cmd 0.1.0`

命令:

```bash
cd /home/luodiary/Desktop/ROSProject/20260812
colcon build --packages-select attitude_cmd
source install/setup.bash
ros2 launch attitude_cmd freeze_chain.launch.py          # 正式: 链路+转换
ros2 launch attitude_cmd freeze_chain.launch.py use_fake_px4:=true   # 回环冒烟
```

关键测试输出(节选,2026-08-28):

```
# 转换(目标 ENU(10,0,-1) → cmd_vel NED):
/uav/cmd_vel        linear: (x=-0.023, y=0.981, z=0.194)   # v_N,v_E,v_D
/attitude_setpoint  [0.9956, 0.0932, 0.0031, -0.0003, 0.482]  # roll≈0.185rad, thrust≈0.482
fake_px4 RX         SET_ATTITUDE_TARGET q=[0.9956, 0.0932, ...] thrust=0.4822
# pose 别名 ENU:
/mavros/local_position/pose  position (x=-0.071, y=0.143, z=1.0)   # 源 NED(x=0.143,y=-0.071,z=-1)
# 超时/故障:
cmd_vel 停发 3 s        → /uav/cmd_state = HOLD, 设定值 = [1,0,0,0,0.5]
链路断开(反馈停) 1 s    → /uav/cmd_state = FAULT, 设定值锁定水平
/system/current_state=ERROR(手动) → SAFETY_HOLD → 3 s 后 COMMAND_LONG cmd=176 p2=4(ACK result=0)
fake_px4 被杀(心跳断)  → /system/current_state = ERROR(自动上报)
```

**Humble 兼容性说明**: 代码未使用 Jazzy 专有 API(std_msgs/Header 已在
Humble 可用;`rclpy.executors.ExternalShutdownException` Humble 存在;绘图/
pymavlink 与发行版无关)。在 Humble 上的预期差异仅为 `colcon build` 环境与
`setup.cfg` 的 `lib/<pkg>` 安装布局(两者 Humble 行为一致)。Humble 上未做
实测,首次联调建议先跑 `ros2 launch attitude_cmd freeze_chain.launch.py`
冒烟(见本表各话题应出现)。

---


## 输入校验与故障语义(修订 V1.1,2026-08-29)

发现并修复确定性安全缺陷(隔离 DDS 稳态测试证据:
/home/zoey/haiying_received/cmd_vel_to_attitude_synchronized_v2_20260829.txt,
SHA256 582c1a154546bf71ef54c27c339ce6c26959344da6314c2da34a9e704d49b6a5):
NaN 指令曾被当作有效指令,cmd_state=ACTIVE 且 ttitude_setpoint 含非有限值。
**修复前该转换节点不得接入 PX4 或正式 V9.2 启动器**(现修复已完成并验证)。

正式修复规则(已确认并实现于 cmd_vel_to_attitude.py):

1. /uav/cmd_vel 六个字段(linear.x/y/z、angular.x/y/z)任一为 NaN/±Inf
   → **立即拒绝**(不更新 _vel_cmd/_yaw_rate);
2. **不刷新有效指令时间戳**(_last_cmd 保持上次有效值);
3. 立即输出**有限水平悬停设定值** [1,0,0,0, hover_thrust] 并发布
   cmd_state=HOLD;
4. /vehicle_velocity 含 NaN/±Inf 或字段数 ≠ 3 → **拒绝**(不更新估计与
   时间戳;连续超时 1 s 后走既有 FAULT 阶梯);
5. **是,需要独立上报**: 拒绝时经 /system/current_state 发布 ERROR
   (每轮事件一次,收到有效指令后静默复位);
6. 自动测试: 	est/test_cmd_vel_to_attitude.py(pytest,27 项)覆盖
   NaN/+Inf/-Inf × 6 字段、异常反馈(值非法/长度非法)、时间倒退
   (单调时钟注入)、限幅有限性与 HOLD/ERROR/恢复语义。

附加加固(同轮实施): ttitude_cmd_node::_on_setpoint 拒绝非有限
/attitude_setpoint(忽略 + 上报 ERROR + 进入 SAFETY_HOLD),保证链路层不向
MAVLink 转发非有限值。计时基准统一为 	ime.monotonic()(注入式,测试可模拟
时间倒退;墙钟回拨不再影响超时判定)。

验证(Jazzy 实测): 有效指令 ACTIVE → NaN 指令 HOLD + [1,0,0,0,0.5] +
/system/current_state=ERROR(一次) → 有效指令恢复 ACTIVE;链路对端仅收到
有限设定值(0 个 nan/inf)。单元测试 27/27 通过。


## 链路层四项安全缺口与故障接口架构(修订 V1.2,2026-08-29)

Humble 内存 MAVLink 测试(官方 main e5023f9,冻结报告 SHA256
9dfb4b9a330a6135b113de99344a0de0c360f779ec6d79b83e043cac1a828440)
确认四项缺口,均已修复:

1. /attitude_setpoint 长度 ≠ 5: 原来仅 return(继续发旧值),现与非法值
   **统一拒绝**——不更新设定值、当拍 _enter_safety_hold()、上报故障;
2. set_attitude_target_send() 异常: 原来仅 _report_error()(依赖异步回环),
   现**当拍直接 _enter_safety_hold()**(不依赖自身 ERROR 话题回环);
3. LOCAL_POSITION_NED(x/y/z/vx/vy/vz)与 ATTITUDE_QUATERNION(q1..q4)
   新增 **isfinite 校验**: 非法遥测拒绝(不污染 /vehicle_velocity、
   /mavros/local_position/pose、姿态缓存),上报故障,并计入超时判定
   (_last_pos 不刷新 → 走遥测超时阶梯);
4. 链路节点安全计时**统一改为单调时钟**(_time_fn,注入式,与转换节点一致),
   并补充时间倒退测试。

架构(与 FSM 解耦): 飞控链路层(cmd_vel_to_attitude、
mavlink_link_node)**不再发布 /system/current_state**,改由独立故障接口
/uav/flight_fault(String,ERROR 每轮事件一次、恢复静默)上报;
/system/current_state 由决策层(FSM)唯一发布。reeze_chain.launch.py
正式联仿只启动链路+转换节点(决策层单独运行;ake_px4 仅 use_fake_px4:=true
回环冒烟用)。

验证: pytest 35/35 通过(转换节点 27 + 链路节点 8);Jazzy 实机: 长度错误/
NaN 设定值 → HOLD + /uav/flight_fault=ERROR,NaN cmd_vel → 转换节点
light_fault=ERROR,飞控层不再发布 /system/current_state(echo 为空)。

## 修订记录

| 日期 | 内容 |
|---|---|
| 2026-08-28 | V1.0: 冻结坐标系(cmd_vel=NED, pose=ENU)、FAULT 语义、ERROR 上报职责表、launch+params、超时阶梯、端口规则、实测记录 |
| 2026-08-29 | V1.1: NaN/Inf 指令拒绝(HOLD+ERROR)、反馈校验、单调时钟、链路层设定值守卫、自动化测试(27 项) |
| 2026-08-29 | V1.2: 链路层四项缺口(设定值长度拒绝/发送失败当拍 HOLD/遥测 isfinite/单调时钟)+ 独立故障接口 /uav/flight_fault + launch 正式范围;测试 35 项 |