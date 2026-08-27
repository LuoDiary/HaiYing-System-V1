# 本机 IK：仿真规划到实机测试

## 已准备的本机结构

- Python 3.12 环境：`haiying`
- LeRobot：按项目约定安装在独立的 `haiying` Conda 环境，不提交到本仓库
- IK 使用的无相机模型：由仿真组放入 `simulation/arm`，或通过 `HAIYING_ARM_ROOT` 指定
- IK 服务：`http://127.0.0.1:8766`
- 当前机械臂串口：`/dev/ttyACM0`
- 校准 ID：`jiebang_follower_arm`

`haiying` 只承载 LeRobot、IK 和桥接脚本。ROS 2 Humble 仍使用系统的
Python 3.10 环境；不要在 `haiying` 环境中安装或运行 `rclpy`。该 Conda
环境会清空继承的 ROS `PYTHONPATH`，避免 Python 3.10 包污染 Python 3.12。

## 1. 启动本机 IK 服务

终端 A：

```bash
cd <HaiYing-System-V1 工作空间>
./control/haiying_zhixun_bridge/scripts/start_ik_server.sh
```

服务只绑定回环地址，不访问串口。浏览器可打开
`http://127.0.0.1:8766`，健康检查为：

```bash
curl http://127.0.0.1:8766/api/health
```

## 2. 规划并做 dry-run（不连接机械臂）

终端 B：

```bash
cd <HaiYing-System-V1 工作空间>
conda activate haiying
python control/haiying_zhixun_bridge/scripts/plan_target.py \
  --x 0.005534 --y -0.179839 --z 0.171219
```

这是从受限执行参考 TCP 沿 `+Z` 移动约 10 mm 的首测目标。保存输出中的
`plan_id`，然后运行：

```bash
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  dry-run --plan-id <PLAN_ID>
```

dry-run 会再次检查 91 帧轨迹、碰撞、帧间最大步长和实机执行锁，不打开
串口。`plan_id` 只存在当前 IK 服务进程的内存中，服务重启后需重新规划。

## 3. 校准（由操作者执行）

以下命令会连接机械臂、关闭扭矩并进入交互式校准。开始前移除末端负载，
保证每个关节有完整安全活动空间，并准备随时断电：

```bash
conda activate haiying
lerobot-calibrate \
  --robot.type=so101_follower_5dof \
  --robot.port=/dev/ttyACM0 \
  --robot.id=jiebang_follower_arm
```

成功后文件应位于：

```text
~/.cache/huggingface/lerobot/calibration/robots/so101_follower_5dof/jiebang_follower_arm.json
```

本次准备过程不会运行该命令。

## 4. 校准后的首次实机测试

先做只读状态检查。该步骤会连接并配置舵机，但不会发送目标位置：

```bash
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  --allow-hardware inspect
```

确认五个关节反馈合理后，只做 1° 单轴点动：

```bash
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  --allow-hardware jog --joint shoulder_pan --delta-deg 1
```

最后才允许执行已经 dry-run 通过的受限轨迹：

```bash
python control/haiying_zhixun_bridge/scripts/arm_control.py \
  --allow-hardware execute \
  --plan-id <PLAN_ID> \
  --confirm-execute
```

执行器还会强制检查：全零 URDF 起点、实际关节起点误差不超过 2°、仅
`+Z 8–12 mm`、XY 漂移不超过 0.5 mm、单关节总行程不超过 10°、轨迹
无碰撞且反馈误差不超过 3°。任一条件不满足都会在发出轨迹前拒绝执行。

## 5. ROS 2 桥接

IK 服务保持在终端 A。另开未激活 Conda 的系统 ROS 终端：

```bash
cd <HaiYing-System-V1 工作空间>
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run haiying_zhixun_bridge haiying-arm-bridge-node
```

节点订阅 `/arm/target_pose`，且只有系统状态 `/system/current_state` 为
`BRUSHING` 时才接受目标。默认 `simulation_only: true`，ROS 节点只请求
规划，不会直接驱动实体机械臂。
