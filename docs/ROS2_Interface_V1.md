# AuV 海鹰智巡项目 - 全局 ROS 2 接口规范 V1.0

> **注意**：本文件是系统各模块通信的唯一官方契约。任何人都**严禁私自修改**以下 Topic 名称与消息类型！如需变更，必须经 PM 确认后统一更新。

| 发送方（出版商） | 接收方（订阅者） | Topic 主题名称（代码必抄） | 消息类型（官方数据包类型） | 业务作用说明 |
| :--- | :--- | :--- | :--- | :--- |
| 章天毅/张天翼 (视觉) | 陈晓瑜/章天毅 (决策大脑) | `/vision/target_point` | `geometry_msgs/msg/PointStamped` | **目标位置**：发布 YOLO 识别到的目标 XYZ 绝对坐标 |
| 陈晓瑜/章天毅 (决策) | 张韬 (控制飞控) | `/uav/cmd_vel` | `geometry_msgs/msg/Twist` | **飞行指令**：大脑指挥无人机前后左右速度与姿态指令 |
| 陈晓瑜/章天毅 (决策) | 曹圆圆 (机械臂) | `/arm/target_pose` | `geometry_msgs/msg/PoseStamped` | **机械臂指令**：大脑指挥机械臂末端伸向的目标位姿 |
| 陈晓瑜/章天毅 (决策) | 全系统各节点 (全员) | `/system/current_state` | `std_msgs/msg/String` | **系统广播**：当前处于哪个任务阶段（如 SEARCHING 等） |
| H743 飞控底层 | 视觉/控制/仿真 | `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | **飞控反馈**：无人机当前真实的绝对位置与姿态反馈 |