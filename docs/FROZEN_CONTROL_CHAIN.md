# /uav/cmd_vel → PX4/H743 控制链冻结规格 (FREEZE)

> 冻结日期: 2026-08
> 移植状态: 已合入核心仓库 HaiYing-System-V1(2026-08-28), 源码路径见下表
> 冻结范围: ROS2 上位机 → MAVLink 直连 → PX4 (x-mav/ap-h743r1) 的唯一控制链。
> 本文档为唯一有效规格;与本文档冲突的旧行为(如直接发布 `/attitude_setpoint`
> 的 `hover_demo_node` 参与联调)一律废弃。

## 1. 冻结节点拓扑

```
决策层 ── /uav/cmd_vel (Twist) ──────────┐
决策层 ── /uav/decision_state (String) ──┤
                                         ▼
                          cmd_vel_to_attitude (转换节点, 无 MAVLink)
                            │ /attitude_setpoint [qw qx qy qz thrust]
                            │ /uav/cmd_state (ACTIVE/HOLD)
                            ▼
                          attitude_cmd_node (MAVLink 直连节点, 独占 TELEM 串口)
                            | SET_ATTITUDE_TARGET (msg 82) → PX4
                            | /mavros/local_position/pose (别名, ENU) ← LOCAL_POSITION_NED
                            | /vehicle_velocity /vehicle_local_position /vibration ...
                            | /uav/flight_fault (独立故障接口, 飞控层上报)
                          (系统状态 /system/current_state 仅由决策层/FSM 发布)
                          PX4 (H743R1) → mc_att_control → 执行机构
```

**控制链上只允许一个运动指令源(`cmd_vel_to_attitude`)和一个 MAVLink 属主
(`attitude_cmd_node`);严禁同时运行 `hover_demo_node`、手动 `ros2 topic pub`
`/attitude_setpoint`、MAVROS 等其它控制/链路实体。**

## 2. 节点、源码、启动命令

| 节点 | 源码路径 | 职责 |
|---|---|---|
| `cmd_vel_to_attitude` | `control/ros-package/attitude_cmd/attitude_cmd/cmd_vel_to_attitude.py` | Twist → 四元数/推力 → `/attitude_setpoint`;cmd_vel 超时判定 |
| `attitude_cmd_node` | `control/ros-package/attitude_cmd/attitude_cmd/mavlink_link_node.py` | 串口/MAVLink 属主;安全状态机;遥测与标准话题别名 |
| `fake_px4`(仅仿真) | `control/ros-package/attitude_cmd/attitude_cmd/fake_px4.py` | SITL 替代对端(UDP 回环) |
| `plot_vibration` / `plot_hover_drift` | 同包 | 离线绘图(数据来自 `~/.px4_viz/*.csv`) |

启动命令(真实硬件):

```bash
# 终端1: 链路节点(独占 TELEM1)
ros2 run attitude_cmd attitude_cmd_node --ros-args -p port:=/dev/ttyS2 -p baud:=57600

# 终端2: 唯一运动指令转换节点
ros2 run attitude_cmd cmd_vel_to_attitude
```

正式联仿(唯一入口, 决策层单独运行):

```bash
ros2 launch attitude_cmd freeze_chain.launch.py              # 链路+转换节点
# 决策层/FSM 单独启动(不在 launch 内): mission_fsm_node / approach_controller
```

回环冒烟(无硬件):

```bash
ros2 launch attitude_cmd freeze_chain.launch.py use_fake_px4:=true
```
ros2 run attitude_cmd attitude_cmd_node --ros-args -p port:=udpin:14550
# 终端3: 转换节点
ros2 run attitude_cmd cmd_vel_to_attitude
# 终端4: 决策层模拟(发布速度指令)
ros2 topic pub -r 20 /uav/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

## 3. Twist → 姿态四元数 / thrust 转换规则(冻结)

坐标系: NED(线速度)、FRD(姿态);单位 m/s、rad/s。

1. 速度误差: `e_v = v_cmd − v_est`(`v_est` 来自 `/vehicle_velocity`,
   由 attitude_cmd_node 从 `LOCAL_POSITION_NED` 转发;反馈超时 1 s 内无新值
   按 0 处理并告警)
2. 期望加速度: `a_des = K_v·e_v`,`K_v = vel_gain`(默认 1.8,对应 PX4
   `MPC_XY_VEL_P_ACC`);`|a_xy| ≤ max_accel = 2.0 m/s²`,`|a_z| ≤ 2.0 m/s²`
3. 期望机体 Z 轴(推力反方向): `body_z = normalize([−a_x, −a_y, g − a_z])`,
   `g = 9.81` —— 与 PX4 `PositionControl.cpp:217` 完全一致
4. 倾角限幅: `angle(body_z, [0,0,1]) ≤ max_tilt = 0.5 rad`
5. 航向: `ψ += ω_z·dt`(角速度积分,周期 50 Hz)
6. 四元数: `q_d = bodyzToAttitude(body_z, ψ)` —— 与 PX4
   `ControlMath.cpp:70-114` 逐行一致的移植(身体轴三列成旋转矩阵 → 四元数)
7. 推力: `thrust = (g − a_z)·hover_thrust/g`,`hover_thrust = 0.5`
   (对应 `MPC_THR_HOVER`);限幅 `[thrust_min, thrust_max] = [0.1, 0.9]`
8. 小角度近似(PX4 欧拉角约定,机头下沉 = 负 pitch):

   ```
   pitch ≈ −atan(a_x/g)     roll ≈ atan(a_y/g)     thrust ≈ hover_thrust
   ```

## 4. 超时与安全状态机(冻结)

参数(attitude_cmd_node): `hold_mode_timeout = 3.0 s`、
`error_mode_action = 1`(0=保持 offboard 悬停,1=Hold,2=POSCTL,3=RTL,4=Land)、
`cmd_state_timeout = 5.0 s`、`hover_thrust = 0.5`。
参数(cmd_vel_to_attitude): `cmd_vel_timeout = 3.0 s`。

触发源:

| 事件 | 判定节点 | 动作 |
|---|---|---|
| `/uav/cmd_vel` 超时 3 s | cmd_vel_to_attitude | `cmd_state=HOLD`,输出水平+悬停推力设定值 |
| `/uav/cmd_state` 丢失 5 s | attitude_cmd_node | 视同 HOLD |
| `/uav/decision_state=HOLD/ERROR` | attitude_cmd_node | 进入安全保持 |
| `/uav/decision_state=RTL/LAND` | attitude_cmd_node | 立即模式切换 |

状态机(attitude_cmd_node):

```
NORMAL ──(HOLD 触发)──▶ SAFETY_HOLD ──(hold_mode_timeout 超时)──▶ SAFETY_SWITCHED
  ▲                        │ (期间: 停止转发外部 /attitude_setpoint,
  └────(decision NOMINAL)──┘  改为内部水平+悬停推力设定值)
```

优先级(冻结,ERROR/超时时按序执行):

1. **停止发布前进指令**: 立即丢弃外部 `/attitude_setpoint`,内部改发
   水平姿态 + `hover_thrust`(保持 offboard 链路存活,不触发链路丢失)
2. **零速度保持**: 上述水平悬停设定值持续 `hold_mode_timeout`(3 s)
3. **模式切换**: 超时后按 `error_mode_action` 发送 `MAV_CMD_DO_SET_MODE`
   (本 fork 编码: p1=1, p2=主模式, p3=子模式 —— 见
   `Commander.cpp:925-928`):
   - Hold: p2=4, p3=3 (AUTO+LOITER)
   - POSCTL: p2=3, p3=0
   - RTL: p2=4, p3=5
   - Land: p2=4, p3=6
4. **兜底(PX4 原生)**: 若链路丢失,`COM_OF_LOSS_T=1 s` 后
   `COM_OBL_RC_ACT=0`(默认)进入 Position 模式悬停

## 5. 标准反馈话题(冻结)

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/mavros/local_position/pose` | `PoseStamped` | attitude_cmd_node(别名) | 兼容 MAVROS 命名;frame=`map`,NED;orientation=最新姿态 |
| `/vehicle_local_position` | `PointStamped` | attitude_cmd_node | NED 位置 |
| `/vehicle_velocity` | `Float32MultiArray` | attitude_cmd_node | `[vx vy vz]` NED |
| `/vehicle_attitude` | `Quaternion` | attitude_cmd_node | 最新姿态 |
| `/vehicle_state` | `String` | attitude_cmd_node | armed + 模式 |
| `/vibration` | `Float32MultiArray` | attitude_cmd_node | 振动 + 削波 |

`/mavros/local_position/pose` **已实现**(attitude_cmd_node 内直接发布),无需
MAVROS。禁止再引入 MAVROS 进程(串口互斥)。

## 6. 固件配套改动(已冻结)

| 文件 | 改动 |
|---|---|
| `src/modules/mavlink/mavlink_receiver.cpp/.h` | `handle_message_attitude_setpoint` + 共享 `process_attitude_target`(消息 82 处理) |
| `src/modules/mavlink/mavlink_main.cpp:1683` | NORMAL 模式 `VIBRATION` 0.1→1.0 Hz(QGC 振动曲线) |

## 7. 联合仿真唯一启动清单

```bash
# 1) 模拟飞控   2) 链路节点   3) 转换节点   4) 决策模拟(可选)
ros2 run attitude_cmd fake_px4
ros2 run attitude_cmd attitude_cmd_node --ros-args -p port:=udpin:14550
ros2 run attitude_cmd cmd_vel_to_attitude
# 决策层/手动: /uav/cmd_vel 与 /uav/decision_state
```

**禁止同时运行**: `hover_demo_node`、`plot_*` 脚本(离线,互不影响但勿与实时
链混用)、MAVROS、任何直发 `/attitude_setpoint` 或直开串口的进程。

## 8. 版本

- 唯一有效版本: 冻结提交(commit)后 ROS2/ 目录内容,见 git log
- 远程已验证工作区: `/home/luodiary/Desktop/ROSProject/20260812`