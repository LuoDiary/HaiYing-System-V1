# /uav/cmd_vel → PX4/H743 冻结链 — 现状确认问答(核心仓库版)

> 本文基于**核心仓库** `F:\Compete-Project\2026-summer\HaiYing-System-V1` 的当前
> 代码回答 6 个问题。与冻结规格(`PX4-Autopilot` 仓库提交
> `874c6847e0`)逐项对照,明确标注"已实现 / 已移植 / 待办"。
>
> **移植状态(2026-08-28)**: 冻结链四件套已合入本仓库并远程联调验证通过 —
> `control/ros-package/attitude_cmd/attitude_cmd/cmd_vel_to_attitude.py`
> (转换节点)、`mavlink_link_node.py`(安全状态机 + `/mavros/local_position/pose`
> 别名 + `/vehicle_velocity`,并新增 `/system/current_state` 订阅适配核心仓库
> 决策层)、`setup.py`(入口注册)、`docs/FROZEN_CONTROL_CHAIN.md`(冻结规格)。
> 远程验证: 转换四元数/推力、别名反馈、3 s 超时 HOLD、ERROR→安全保持→
> DO_SET_MODE 全部通过。

## 版本基线

| 项 | 值 |
|---|---|
| 核心仓库 | `F:\Compete-Project\2026-summer\HaiYing-System-V1` |
| 分支 / HEAD | `main` / `ac2848a4a72ef08c8153e220215bb602c81e4aa4`(2026-08-13,`feat(attitude_cmd): add plotting tools, fake px4 mock and telemetry logging`) |
| 接口契约 | `docs/ROS2_Interface_V1.md`(`/uav/cmd_vel`、`/system/current_state`、`/mavros/local_position/pose` 等官方 Topic) |
| 冻结规格(另一仓库) | `PX4-Autopilot` 分支 `luodiary/freeze-cmd-vel-control-chain` @ `874c6847e08240005a4a3c5e3feb5deac73c90ed` |

**核心仓库 `attitude_cmd` 包 = 已验证基础版**(与远程
`/home/luodiary/Desktop/ROSProject/20260812` 部署联调版本一致,MD5 相同);
**冻结链新增内容已合入本仓库(2026-08-28 移植),未提交 git,以工作区为准**。

---

## Q1. 哪个正式节点负责把 `/uav/cmd_vel` 的 Twist 转换为 `/attitude_setpoint`?

**答案: `cmd_vel_to_attitude`(已移植到核心仓库并验证)。**

- `/uav/cmd_vel`(Twist)**唯一生产者** = `scripts/approach_controller.py`
  (决策接近控制器,状态机 SEARCHING→APPROACHING→BRUSHING→HOVERING;
  第 57-58 行发布,第 150-156 行 `_publish_cmd_vel`)。
- **转换节点(唯一消费者)**: `control/ros-package/attitude_cmd/attitude_cmd/
  cmd_vel_to_attitude.py`(移植自 `PX4-Autopilot@874c6847e0`,2026-08-28 合入
  本仓库工作区;`setup.py` 已注册入口 `cmd_vel_to_attitude`)。
- 转换节点订阅 `/uav/cmd_vel` + `/vehicle_velocity`,发布 `/attitude_setpoint`
  与 `/uav/cmd_state`,50 Hz。

启动命令:

```bash
ros2 run attitude_cmd cmd_vel_to_attitude   # 订阅 /uav/cmd_vel + /vehicle_velocity
```

## Q2. Twist → 四元数/thrust 转换规则、坐标系、限幅、超时

**答案: 已移植并在远程联调验证(倾斜四元数与推力输出正确)。规则即冻结版
(`cmd_vel_to_attitude.py`,已合入本仓库):**

参数(默认值即冻结值):

| 参数 | 默认 | 说明 |
|---|---|---|
| `rate` | 50 Hz | 转换/发布周期 |
| `cmd_vel_timeout` | 3.0 s | `/uav/cmd_vel` 超时 → `cmd_state=HOLD` |
| `vel_gain` | 1.8 | 速度 P 增益(= PX4 `MPC_XY_VEL_P_ACC`) |
| `max_accel` | 2.0 m/s² | 期望加速度限幅(水平等比缩放,垂直对称限幅) |
| `max_tilt` | 0.5 rad | 倾角限幅(≈28.6°) |
| `hover_thrust` | 0.5 | 悬停推力(= `MPC_THR_HOVER`) |
| `thrust_min/max` | 0.1 / 0.9 | 推力限幅 |
| `vel_feedback_timeout` | 1.0 s | 速度反馈过期按 0 处理并告警 |

公式(与 PX4 `mc_pos_control` 一致):
1. `e_v = v_cmd − v_est`(反馈来自 `/vehicle_velocity`,即 `LOCAL_POSITION_NED.vx,vy,vz`)
2. `a_des = K_v·e_v`,`|a_xy| ≤ 2.0`、`|a_z| ≤ 2.0`
3. `body_z = normalize([−a_x, −a_y, g − a_z])`,`g = 9.81`(对应 `PositionControl.cpp:217`)
4. 倾角限幅 `angle(body_z, [0,0,1]) ≤ 0.5 rad`
5. `ψ = (ψ + ω_z·dt) mod 2π`
6. `q_d = bodyzToAttitude(body_z, ψ)`(对应 `ControlMath.cpp:70-114` 移植)
7. `thrust = (g − a_z)·hover_thrust / g`,clamp [0.1, 0.9]
8. HOLD 态输出 `[1,0,0,0, hover_thrust]`

坐标系: 线速度 **NED**(m/s);姿态 **FRD**(四元数 w,x,y,z);推力归一化 **0~1**。
小角度近似: `pitch ≈ −atan(a_x/g)`(机头下沉为负)、`roll ≈ atan(a_y/g)`。

## Q3. `attitude_cmd` 是否已实际运行验证并作为最终版本?

**答案: 已运行验证(基础版),冻结链完整版已移植合入(2026-08-28),仍待提交 git。**
核心仓库 `main@ac2848a4` 的 `attitude_cmd` 包已远程(Ubuntu 24.04 + ROS2 Jazzy)
构建并端到端联调通过(50 Hz 姿态指令、OFFBOARD/ARM、振动/位置/漂移遥测、CSV
与绘图),MD5 与远程部署版一致;移植后的冻结链(转换节点/安全状态机/别名)已在
远程构建并全链联调通过。

唯一有效版本:

| 版本 | 位置 | 状态 |
|---|---|---|
| 冻结链完整版(当前工作区) | 核心仓库 `control/ros-package/attitude_cmd/`(未提交,以工作区为准;`cmd_vel_to_attitude.py` 为新增文件) | **已移植 + 已验证** |
| 链路基础版(历史) | 核心仓库 `main@ac2848a4` 原文件(MD5: `mavlink_link_node.py=8201133214FCE9E1752E843739B94352`,`fake_px4.py=36391FF995FD74E19B74581B31901B86`,`setup.py=7CB605F542A769835312A7623F3BFECD`,`package.xml=B373D1635A90262587509D2A668DD95E`) | 已被冻结版取代 |

## Q4. 无 MAVROS 时 `/mavros/local_position/pose` 由谁发布?是否已实现?

**答案: 已由 `attitude_cmd_node` 实现别名发布(无需 MAVROS),远程已验证。**

- 接口契约(`docs/ROS2_Interface_V1.md`)规定: `/mavros/local_position/pose`
  由 "H743 飞控底层 或 Gazebo/Mavros 仿真发布" —— 现由本仓库
  `attitude_cmd_node` 满足(契约无需修改)。
- 实现位置: `mavlink_link_node.py`(发布器 + `LOCAL_POSITION_NED` 处理),
  `PoseStamped`,frame=`map`,NED 位置 + 最新 `ATTITUDE_QUATERNION` 姿态;
  同步发布 `/vehicle_velocity`(`[vx vy vz]`)供 `cmd_vel_to_attitude` 使用。
- 仿真侧旧 MAVROS 路径(`simulation/uav_control` 的 takeoff/vision_control)
  冻结后禁止运行(与直连链路冲突)。

## Q5. ERROR 或 `/uav/cmd_vel` 超时 3 s,由哪个节点执行 Hold/POSCTL/RTL/Land?

**答案: 已实现安全阶梯(移植后),远程已验证 3 s 超时与 ERROR 两条路径。**

现状(核心仓库,移植后):

| 事件 | 现状 |
|---|---|
| `/uav/cmd_vel` 超时 3 s | `cmd_vel_to_attitude` 判定:`cmd_state=HOLD`,输出水平 + 悬停推力(已验证) |
| `cmd_state=HOLD` / 丢失 5 s | `attitude_cmd_node` 进入 SAFETY_HOLD(已验证) |
| `/system/current_state=ERROR` | `attitude_cmd_node` 进入 SAFETY_HOLD(已验证) |
| `/system/current_state=RTL/LAND` | `attitude_cmd_node` 立即 `DO_SET_MODE` 切换 |
| 安全保持超时 3 s | 按 `error_mode_action`(默认 1)发 `MAV_CMD_DO_SET_MODE`(已验证 cmd=176) |
| PX4 原生兜底 | 有效:`COM_OF_LOSS_T=1 s` 后 `COM_OBL_RC_ACT=0` → Position 模式悬停(固件侧) |

状态机(`mavlink_link_node.py`,移植版): `NORMAL →(HOLD 触发)→ SAFETY_HOLD
→(hold_mode_timeout=3 s)→ SAFETY_SWITCHED`;SAFETY_HOLD 期间停止转发外部
`/attitude_setpoint`,内部水平 + `hover_thrust`,保持 offboard 链路存活。

`MAV_CMD_DO_SET_MODE` 编码(本 fork:p1=1,p2=主,p3=子): 0=保持 offboard,
1=Hold(p2=4,p3=3),2=POSCTL(p2=3,p3=0),3=RTL(p2=4,p3=5),4=Land(p2=4,p3=6)。

最终优先级(冻结): ① 停止发布前进指令(立即)→ ② 零速度/水平保持(3 s)
→ ③ 模式切换(按 `error_mode_action`)→ ④ PX4 原生兜底
(`COM_OF_LOSS_T`/`COM_OBL_RC_ACT`)。

## Q6. 最终联合仿真应启动哪些控制节点?

**答案: 冻结清单已可运行(全链远程验证通过),只保留一条控制路径。**

当前仿真(核心仓库,`simulation/launch/offshore.launch.py` +
`scripts/README.md`):

```
PX4 SITL + Gazebo(iris.sdf)
 ├─ 路径A(旧,冻结后禁用): MAVROS + uav_control(takeoff / vision_control,
 │             /mavros/setpoint_position/local 位置控制)
 └─ 路径B(冻结): 视觉链 yolo → target_localizer → approach_controller
                └─ /uav/cmd_vel → cmd_vel_to_attitude → /attitude_setpoint
                   → attitude_cmd_node → MAVLink → PX4  ✅已全链验证
```

冻结后唯一启动清单(避免多速度控制器):

```bash
# 仿真(无硬件): 唯一运动指令源 = approach_controller;唯一 MAVLink 属主 = attitude_cmd_node
ros2 run attitude_cmd fake_px4                                  # 模拟飞控(SITL 替代)
ros2 run attitude_cmd attitude_cmd_node --ros-args -p port:=udpin:14550   # 链路属主
ros2 run attitude_cmd cmd_vel_to_attitude                       # 唯一转换节点
python3 scripts/approach_controller.py                          # 决策/接近(唯一 /uav/cmd_vel 生产者)
```

**禁止同时运行**: `hover_demo_node`(直发 `/attitude_setpoint`)、
`simulation/uav_control` 的 takeoff/vision_control(MAVROS 位置控制,与
`/uav/cmd_vel` 链构成双速度控制器)、MAVROS 进程、手动
`ros2 topic pub /attitude_setpoint`、任何直开串口进程。

---

## 结论汇总

| # | 问题 | 核心仓库现状(工作区,已移植) |
|---|---|---|
| 1 | Twist 转换节点 | **已移植**: `cmd_vel_to_attitude.py`(合入本仓库,远程验证通过) |
| 2 | 转换规则 | **已移植**: 冻结版参数/公式见 Q2(已验证倾斜四元数与推力输出) |
| 3 | 验证与唯一版本 | 冻结链完整版已移植 + 远程全链验证;基础版已被取代,见 Q3 表 |
| 4 | `/mavros/local_position/pose` | **已实现**: attitude_cmd_node 别名发布(PoseStamped, frame=map),无需 MAVROS |
| 5 | ERROR/3 s 超时 | **已实现**: 3 s 无 cmd_vel→HOLD;/system/current_state=ERROR→安全保持→3 s 后 DO_SET_MODE(已验证 cmd=176) |
| 6 | 联合仿真清单 | 冻结清单可运行(fake_px4 + attitude_cmd_node + cmd_vel_to_attitude + approach_controller 全链已验证);严禁 MAVROS/hover_demo 同跑 |