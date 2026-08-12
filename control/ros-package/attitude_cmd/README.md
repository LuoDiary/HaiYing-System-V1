# ROS2 — TELEM 串口 + MAVLink 姿态控制样例 (x-mav/ap-h743r1)

上位机(ROS2)通过飞控 TELEM 串口 + MAVLink 协议下发飞行姿态指令的样例节点包。

## 1. 姿态控制相关变量与函数 (固件侧)

PX4 的姿态控制链路:

```
ROS2 节点 --(MAVLink msg 82 SET_ATTITUDE_TARGET)--> MavlinkReceiver
  -> offboard_control_mode (attitude=true)
  -> vehicle_attitude_setpoint (q_d / thrust_body)   [uORB]
  -> mc_att_control (MulticopterAttitudeControl)     [姿态控制器]
  -> control_allocator -> 执行机构
```

| 名称 | 类型/位置 | 说明 |
|---|---|---|
| `vehicle_attitude_setpoint` | uORB 话题 | 姿态期望值,核心变量 |
| `q_d[4]` | `msg/versioned/VehicleAttitudeSetpoint.msg` | 期望姿态四元数 (w,x,y,z,FRD 系) |
| `thrust_body[3]` | 同上 | 归一化推力 [-1,1] |
| `yaw_sp_move_rate` | 同上 | 期望偏航角速度 (rad/s) |
| `offboard_control_mode` | `msg/OffboardControlMode.msg` | `attitude=true` 表示姿态外环 offboard |
| `handle_message_set_attitude_target` | `src/modules/mavlink/mavlink_receiver.cpp:1793` | 原 MAVLink 接收函数 |
| `MavlinkReceiver::handle_message_attitude_setpoint` | `src/modules/mavlink/mavlink_receiver.cpp:1793` | **新增**接收方法(见下) |
| `MavlinkReceiver::process_attitude_target` | `src/modules/mavlink/mavlink_receiver.cpp:1802` | **新增**共享处理函数 |
| `MulticopterAttitudeControl` | `src/modules/mc_att_control/mc_att_control_main.cpp` | 订阅 `vehicle_attitude_setpoint` 的姿态控制器 |
| `NAVIGATION_STATE_OFFBOARD` | commander / `ModeUtil/control_mode.cpp:111` | 接收姿态指令必须处于 OFFBOARD 模式 |

## 2. 固件新增方法

`mavlink_receiver` 中新增了命名清晰的接收入口(逻辑与标准路径一致,保持 OFFBOARD
门控):

- `handle_message_attitude_setpoint(mavlink_message_t *msg)`: 解码消息 82
  `SET_ATTITUDE_TARGET`(payload 与标准消息完全一致),调用共享处理函数。
- `process_attitude_target(const mavlink_set_attitude_target_t &)`: 原
  `handle_message_set_attitude_target` 的完整逻辑(校验 target_system、解析
  type_mask、发布 `offboard_control_mode` 与 `vehicle_attitude_setpoint`),
  仅在 `nav_state == OFFBOARD` 时发布期望值。

字段约定(与 MAVLink 标准一致):

- `type_mask = 0b00000111`:忽略体轴角速度,纯姿态控制
- `q = [qw, qx, qy, qz]`:期望四元数 (FRD 机体系)
- `thrust`:归一化油门 (0~1)

## 3. 硬件与链路

- TELEM1 口 = USART3 = 飞控上 `/dev/ttyS2`(见 `boards/x-mav/ap-h743r1/default.px4board`)
- MAVLink 实例 0 默认绑定 TELEM1(`MAV_0_CONFIG=TELEM1`),默认波特率 57600
  (`SER_TEL1_BAUD`);如需改波特率,NSH 下 `param set SER_TEL1_BAUD 115200`
  并重启,上位机参数保持一致
- 接线:飞控 TELEM1 TX/RX/GND 与上位机串口 TX/RX/GND 交叉连接(TTL 电平 3.3V,
  禁止直连 RS232 电平)
- `MAV_FWDEXTSP` 默认 1,已开启外部期望值接收,无需改动

## 4. 构建与运行

依赖: ROS2 (Humble 及以上), `pip install pymavlink pyserial`

```bash
# 构建
cd ROS2
colcon build --symlink-install
source install/setup.bash

# 终端1: 链路节点(串口按实际设备填写, Windows 如 COM5)
ros2 run attitude_cmd attitude_cmd_node --ros-args \
  -p port:=/dev/ttyS2 -p baud:=57600

# 终端2: 演示发布者(悬停 + 小幅横滚摆动, 默认 20 s, 推力 0.5)
ros2 run attitude_cmd hover_demo_node

# 或手动发布
ros2 topic pub -r 50 /attitude_setpoint std_msgs/msg/Float32MultiArray \
  "{data: [1.0, 0.0, 0.0, 0.0, 0.5]}"
```

节点接口:

| 接口 | 类型 | 说明 |
|---|---|---|
| `/attitude_setpoint` (订阅) | `Float32MultiArray` | 5 个元素 `[qw qx qy qz thrust]`, 50 Hz |
| `/mavlink/offboard` (服务) | `SetBool` | true=切 OFFBOARD, false=切回 MANUAL |
| `/mavlink/arm` (服务) | `SetBool` | true=解锁, false=上锁 |
| `/vehicle_attitude` (发布) | `Quaternion` | 飞控当前姿态 (MAVLink ATTITUDE_QUATERNION) |
| `/vehicle_state` (发布) | `String` | 解锁状态与飞行模式 |

## 5. 使用流程

1. 解锁前先关闭电机/卸桨,或用 `mavlink/arm` 服务解锁
2. 调用 `ros2 service call /mavlink/offboard std_srvs/srv/SetBool "{data: true}"`
   切换到 OFFBOARD(需满足进入条件: EKF 有效、RC 安全、模式切换约束)
3. 以不低于 10 Hz 的频率持续发布 `/attitude_setpoint`(节点默认 50 Hz;
   PX4 在 `COM_OF_LOSS_T`(默认 1 s)内收不到期望值会自动退出 offboard)
4. 完成后切回 MANUAL 并上锁

## 6. 安全警告

- OFFBOARD 模式下飞控完全按期望值执行,任何发布中断/错误姿态均可能失控,
  务必先卸桨台架测试
- 本样例不包含任何安全逻辑(急停、超时保护等),仅作连接与基本控制演示
- 飞行测试请保留 RC 遥控器并配置好遥控器切回/急停通道
