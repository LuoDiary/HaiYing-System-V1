# ROS 2 Humble 与 HaiYing-System-V1 集成说明

## 运行边界

开发机已使用 ROS 2 Humble、colcon、Gazebo Classic 和 MoveIt 2 完成桥接节点、MoveIt 与 Gazebo 的无界面冒烟测试。LeRobot 使用独立 Python 3.12 Conda 环境，不能用仿真结果代替硬件验收。

## 复制到主仓库

推荐放置：

```text
HaiYing-System-V1/
├─ decision/
├─ control/
├─ scripts/
├─ simulation/
│  └─ haiying_zhixun_bridge/
└─ hardware/
```

也可以将包放入独立 ROS 工作空间的 `src/`。不要同时维护两个不同 Topic 版本。

## 构建

```bash
mkdir -p ~/haiying_ws/src
cp -r haiying_zhixun_bridge ~/haiying_ws/src/
cd ~/haiying_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select so-101_description haiying_zhixun_bridge --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 run haiying_zhixun_bridge haiying-arm-bridge-node
```

配置安装到：

```text
share/haiying_zhixun_bridge/config/arm_bridge.yaml
```

可覆盖配置路径：

```bash
ros2 run haiying_zhixun_bridge haiying-arm-bridge-node \
  --ros-args -p config_path:=/absolute/path/arm_bridge.yaml
```

## 固定接口

| 输入 | 类型 | 规则 |
|---|---|---|
| `/system/current_state` | `std_msgs/msg/String` | 只有 `BRUSHING` 接受机械臂目标 |
| `/arm/target_pose` | `geometry_msgs/msg/PoseStamped` | `frame_id` 必须为 `base_footprint` |

示例：

```bash
ros2 topic pub --once /system/current_state std_msgs/msg/String "{data: BRUSHING}"

ros2 topic pub --once /arm/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: base_footprint},
  pose: {
    position: {x: 0.005534, y: -0.179839, z: 0.171219},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

当前节点只使用 position XYZ。orientation 必须是有效四元数，但不进入五自由度 IK。
IK 在后台单工作线程执行；已有目标规划时拒绝第二个目标。若规划期间系统状态离开
`BRUSHING`，即使 IK 随后成功，结果也会作为过期结果丢弃。

## 本地 MoveIt/Gazebo 验证

桥接配置通过以下可移植 URI 复用本地模型：

```text
package://arm_urdf/urdf/so101_arm.urdf.xacro
```

无 Gazebo 的 MoveIt 验证：

```bash
ros2 launch so-101_description moveit.launch.py use_rviz:=false
ros2 run so-101_description plan_execute_smoke_test.py
```

Gazebo 风机场景验证：

```bash
ros2 launch so-101_description gazebo_moveit.launch.py \
  gui:=false use_rviz:=false use_camera_view:=false
ros2 run so-101_description plan_execute_smoke_test.py
ros2 topic echo /so101/wrist_camera/camera_info --once
```

2026-08-04 两条 Plan+Execute 均成功并产生 57 个轨迹点；Gazebo 相机信息为
640×480。该结果验证了本地关节空间规划与控制链，不代表
`/arm/target_pose` 已直接接入 MoveIt，也不代表实机验收。

## 与各组对接

### 陈晓瑜：状态机与 MoveIt

- 保持状态字符串完全一致；
- 非 `BRUSHING` 状态不得发布可执行机械臂目标；
- MoveIt 接入后由统一控制适配器替换当前原型执行端；
- `/arm/target_pose` 的 frame 和时间戳必须来自统一 TF。

### 郑经纬：TF、Gazebo 与 RViz

- 确认 `base_footprint` 在无人机挂载坐标树中的父子关系；
- 加载无相机机械臂 URDF、collision 和单一毛刷模型；
- 验证所有 joint axis、零位和 mesh 尺度；
- 提供风机/叶片环境碰撞模型。

### 张韬：控制底层

- 本包不实现 FOC/PID；
- MoveIt 或轨迹适配器只调用张韬统一发布的底层接口；
- 需要团队统一定义关节目标、反馈、频率和错误码后再接入。

## Gazebo/MoveIt 剩余待办

1. 完成质量、质心和惯量实测并写入 URDF。
2. 为 `end_effector`/毛刷 TCP 补齐实测 collision 包络。
3. 将 `/arm/target_pose` 的 XYZ 位置目标接入 MoveIt 规划场景，或定义 IK 轨迹到 ROS 控制器的唯一适配路径。
4. 将机械臂挂载到无人机 frame，复核重心和载荷。
5. 在现有塔筒/叶片碰撞体上补充毛刷接触模型与力学参数。
6. 验证 TF、RViz、Gazebo、MoveIt 与实机的关节方向一致。
7. 排查本机 MoveIt 2.5.9 在 Ctrl+C 清理阶段的 `class_loader` 段错误。
8. 连续运行五分钟并保存日志、视频和指标。

完成这些步骤前，不得把本包描述为 Gazebo/MoveIt 联合仿真已验收。
