# 视觉组代码 (Vision Module)

海鹰智巡 — 视觉感知模块 V2

## 文件说明

| 文件 | 任务 | 功能 |
|------|------|------|
| `yolo_detector.py` | Task 26, 71 | YOLOv5 GPU推理 → `/vision/detection` |
| `target_localizer.py` | Task 27, 72 | 像素→深度→TF→世界坐标 → `/vision/target_point` |
| `calibration.py` | Task 29 | 相机-LiDAR外参标定TF |
| `approach_controller.py` | 动作3 | 状态机: SEARCHING→APPROACHING→BRUSHING→HOVERING |
| `gz_camera_bridge.py` | 辅助 | Gazebo RGB相机 → ROS2 |
| `gz_depth_bridge.py` | 辅助 | Gazebo 深度相机 → ROS2 (32FC1) |
| `tf_publisher.py` | 辅助 | world→base_link→camera_frame TF发布 |

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
| `/arm/target_pose` | PoseStamped | 发布：机械臂目标位姿 (arm_base + 刷子姿态) |
| `/uav/cmd_vel` | Twist | 发布：无人机速度指令 (接近/悬停) |
| `/system/current_state` | String | 发布：系统状态 (SEARCHING/APPROACHING/BRUSHING/HOVERING) |

## 依赖

- ROS2 Humble
- YOLOv5 + PyTorch (CUDA)
- Gazebo Transport 13 Python绑定
- cv_bridge, OpenCV
- tf2_ros, MAVROS
