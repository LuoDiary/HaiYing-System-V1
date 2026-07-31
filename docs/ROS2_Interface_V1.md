# 海鹰智巡 - ROS 2 全局接口规范 V1.0

> **⚠️ 统筹指令**：所有模块通信强制使用以下 Topic 名称与 Message Type。未经梁玮珩允许，严禁私自修改。联调时如因私改接口导致系统崩溃，修改方负全责。

| 发送方 (Publisher) | 接收方 (Subscriber) | Topic (话题名称，代码必抄) | Message Type (官方数据包类型) | 业务作用说明 |
| :--- | :--- | :--- | :--- | :--- |
| **视觉组** (章天毅) | **控制组-状态机** (陈晓瑜) | `/vision/target_point` | `geometry_msgs/msg/PointStamped` | 视觉发布：YOLO识别到的风机目标 XYZ 坐标 |
| **控制组-状态机** (陈晓瑜) | **控制组-飞控** (张韬) | `/uav/cmd_vel` | `geometry_msgs/msg/Twist` | 大脑指挥：无人机前后左右飞行速度指令 |
| **控制组-状态机** (陈晓瑜) | **控制组-机械臂** (陈晓瑜/曹圆圆) | `/arm/target_pose` | `geometry_msgs/msg/PoseStamped` | 大脑指挥：机械臂末端应该伸向的姿态和位置 |
| **控制组-状态机** (陈晓瑜) | **全系统各节点** (全员) | `/system/current_state` | `std_msgs/msg/String` | 系统广播：当前处于哪个任务阶段 (如 SEARCHING) |
| **底层硬件** (H743飞控) | **控制/视觉/仿真** | `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | 飞控反馈：无人机当前真实的绝对位置 |