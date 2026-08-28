# 视觉组代码 (Vision Module)

海鹰智巡 — 视觉感知模块 V2

## 文件说明

| 文件 | 任务 | 功能 |
|------|------|------|
| `yolo_detector.py` | Task 26, 71 | YOLOv5 GPU推理 → `/vision/detection` |
| `target_localizer.py` | Task 27, 72 | 像素→深度→TF→世界坐标 → `/vision/target_point` |
| `calibration.py` | Task 29 | 相机-LiDAR外参标定TF |
| `approach_controller.py` | 动作3 | 【LEGACY】接近状态机，已被决策组 `mission_fsm_node` 取代，联合仿真不启动 |
| `gz_camera_bridge.py` | 辅助 | Gazebo RGB相机 → ROS2 |
| `gz_depth_bridge.py` | 辅助 | Gazebo 深度相机 → ROS2 (32FC1) |
| `tf_publisher.py` | 辅助 | world→base_link→camera_frame TF发布 |
| `live_turbine_5shots.py` | 验收 | Gazebo实时检测，保存5张不同缺陷/视角截图 |
| `live_turbine_detect.py` | 验收 | 实时检测 + 相机多视角扫描，检出即存截图 |
| `test_turbine_model.py` | 验收 | 模型自检：训练集/实时画面跑检测 |
| `collect_turbine_data.py` | 数据 | Gazebo渲染图采集 + 缺陷坐标投影自动标注YOLO标签 |
| `pose_gt_bridge.py` | 辅助 | Gazebo真值位姿 → `/drone/pose_gt`（MAVROS EKF不可靠时的备选位姿源，`approach_controller.py` 设 `pose_source:=gt` 使用） |
| `lidar_bridge.py` | 辅助 | Gazebo LiDAR → ROS2 PointCloud2（`target_localizer.py` 雷达深度输入） |
| `wind_turbine_interfaces/` | 接口包 | 自定义消息/服务 (DefectDetection, DefectDetectionArray, StartInspection)，`yolo_detector.py`/`target_localizer.py` 依赖，运行前需 colcon build |

## 运行方式

```bash
# 1. 启动Gazebo + PX4 (headless)
gz sim -r -s default.sdf &
PX4_SYS_AUTOSTART=4002 ./px4 -d &

# 2. 启动桥接器
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 gz_camera_bridge.py &
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 gz_depth_bridge.py &
python3 tf_publisher.py &

# 3. 启动视觉节点
python3 yolo_detector.py --ros-args -p image_topic:=/drone/camera/image_raw &
python3 target_localizer.py &
python3 calibration.py &
```

## 话题接口

| 话题 | 类型 | 方向 |
|------|------|------|
| `/vision/detection` | DefectDetectionArray | 发布：YOLO检测结果 |
| `/vision/target_point` | PointStamped | 发布：目标绝对XYZ坐标 (world) |

> 决策组话题（视觉组不再发布）：`/arm/target_pose` (PoseStamped)、`/uav/cmd_vel` (Twist)、
> `/system/current_state` (String) 由决策组 `mission_fsm_node` 唯一发布，详见下文「最终节点职责确认」。

### `/vision/target_point` 接口约定

- **frame**: `world`（`world_frame` 参数可配），**单位**: 米（绝对 XYZ）
- **频率**: 事件驱动，随检测逐帧发布（仿真链路 ≈ 相机帧率 3.3 FPS），无固定频率/心跳
- **目标丢失协议**: **停止发布**（不发布 NaN、无有效位标志）。消费端以 3 秒超时兜底（决策组 `mission_fsm_node` 已实现；`approach_controller.py` 为 legacy，不参与联合仿真）

## 最终节点职责确认（2026-08-28）

视觉组已与组长确认最终节点职责，结论如下：

1. `target_localizer.py` 只发布 `/vision/target_point`，不再发布 `/arm/target_pose`（发布器、`arm_frame` 参数与发布代码已删除）。
2. 最终联合仿真不启动 `approach_controller.py`；`/system/current_state`、`/uav/cmd_vel`、`/arm/target_pose` 由决策组 `mission_fsm_node` 唯一发布。`approach_controller.py` 保留于仓库并标记 legacy。
3. 决策组 FSM 待改事项（转达）：
   - `/arm/target_pose` 的 `frame_id` 需为 `base_footprint`（当前为 `world`）；
   - BRUSHING 后需等待 `/arm/execution_status`：`EXEC_DONE` → RETURNING，`EXEC_FAIL` → ERROR（当前不订阅该话题、立即切 RETURNING）；
   - `/arm/execution_status` 目前全仓库无人发布，需机械臂桥接（`control/haiying_zhixun_bridge/ros_node.py`，当前纯订阅）新增 EXEC_DONE/EXEC_FAIL 发布。
4. 系统状态集删除 HOVERING，最终六态：SEARCHING / TARGET_FOUND / APPROACHING / BRUSHING / RETURNING / ERROR（决策组 FSM 现状即为六态）。
5. ERROR 状态下 `/uav/cmd_vel` 持续发布零速度 Twist（与 FSM 及 `approach_controller.py` 现状一致），不采用「停止发布 + 底盘超时」方案。

## 依赖

- ROS2 Humble
- YOLOv5 + PyTorch (CUDA)
- Gazebo Transport 13 Python绑定
- cv_bridge, OpenCV
- tf2_ros, MAVROS

先编译接口包再运行脚本：

```bash
cd <ros2_ws>
colcon build --packages-select wind_turbine_interfaces
source install/setup.bash
```
