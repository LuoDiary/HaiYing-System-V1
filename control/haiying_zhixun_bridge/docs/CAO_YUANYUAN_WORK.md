# 曹圆圆：机械臂建模仿真与桥接工作说明

## 角色目标

把手头机械臂实物转换成可验证、可共享、可被状态机和控制程序使用的数字模型，并让仿真结果能够在安全边界内映射到原型实机。

## 输入

- 机械臂结构件、舵机和安装件；
- 陈子龙/诸浩然提供的装配状态和毛刷模块；
- 陈晓瑜提供的 `/arm/target_pose` 与轨迹控制需求；
- 郑经纬提供的全系统 TF、Gazebo 和 RViz 接入要求；
- 张韬提供的统一 FOC/PID 上层调用接口；
- 梁玮珩发布的接口规范和里程碑要求。

## 已完成输出

- 五轴无相机 URDF/STL 模型接入；
- 五关节名称、轴、限位和 collision 解析；
- 三维 FK、IK、91 帧轨迹和自碰撞检测；
- 实机近似尺寸 `55/135/135/55/10 mm`；
- 五轴方向和零位偏移映射；
- LeRobot 原型机 inspect/jog/dry-run/restricted execute；
- `/arm/target_pose` 与 `/system/current_state` 桥接代码、非阻塞规划和过期结果保护；
- 无 ROS Mock 流程，以及已在 Humble 构建并启动验证的 `ament_python` 包；
- 本地 SRDF、MoveIt、ros2_control 与 Gazebo 风机场景的 Plan+Execute 冒烟验证。
- `MoveIt → Gazebo → GUI 人工确认 → SO-101 实机`正式执行链路；
- 仓库内 `control/lerobot` Python 3.12/Feetech 运行时，Jetson 不再依赖工作空间外源码；
- 逐关节 URDF 限位及上下各 `1°` 数值容错、轨迹哈希、起点对齐和反馈超差中止；
- 默认 MoveIt 速度/加速度缩放 `0.2`，实机 50 Hz 下发、25°/s 安全上限和 10 Hz 反馈检查；
- 实机执行客户端 180 秒超时，健康检查和不接硬件的轨迹验证保持 10 秒。

## 当前已确认基线

- 五轴顺序：`J1_Rotation, J2_Shoulder_Pitch, J3_Elbow_Pitch, J4_Wrist_Pitch, J5_Wrist_Roll`；
- MoveIt group：`arm`；root：`base_footprint`；TCP：`end_effector`；
- 正式实机入口：RViz `Plan & Execute` 后由 `moveit_real_gui.launch.py` 验证 Gazebo
  终点，再调用本机 `127.0.0.1:8767` 实机服务；历史 `127.0.0.1:8766` IK 只保留兼容，
  不属于当前正式链路；
- 舵机方向：`[+1,+1,-1,+1,+1]`；唯一有效零偏：
  `[-0.175824,-0.747253,-0.527473,16.967033,-11.208791]°`；
- 仿真或轨迹验证报错时不会连接实体机械臂；实机执行中持续反馈超差或串口异常时停止
  后续下发并断开，不自动安全回收，现场必须保留急停/断电能力。

## M3 必做工作

1. 用卡尺复测所有关节轴心距离，记录测量工具、日期和照片编号。
2. 称量每个 link、舵机、线束和毛刷模块，给出质量与估算误差。
3. 测量每个 link 的质心；不能测量时记录 CAD 估计方法和待验证标记。
4. 明确 `base_footprint`、每个 joint frame、毛刷 TCP 和无人机安装 frame。
5. 补全 URDF inertial、collision 简化体和毛刷单一末端模型。
6. 与郑经纬完成 RViz 图形验收、无人机挂载 TF 和 Gazebo 连续运行验收（无界面加载与执行已通过）。
7. 与陈晓瑜确定 `/arm/target_pose` 到 MoveIt 的正式自主规划入口；当前 GUI 实机链路已
   可用，但状态机自动执行仍未启用。
8. 与硬件组完成原型机角度、实际 TCP 位移、重复定位和异常中止测试。

## 验收证据

- URDF/STL 文件及版本号；
- TF 树截图和 frame 说明；
- Gazebo/RViz 模型截图；
- 碰撞拒绝与安全规划各一组日志；
- 实测尺寸、质量、质心和 TCP 表；
- `pytest` 输出；
- ROS Topic 发布/订阅日志；
- 原型机小位移视频和命令记录；
- 仿真预测与实机反馈误差表。

## 跨组交付

| 接收人 | 交付内容 | 验收方式 |
|---|---|---|
| 郑经纬 | URDF、collision、TCP、质量/惯量、frame 定义 | RViz/Gazebo 加载与 TF 无断链 |
| 陈晓瑜 | `/arm/target_pose` 约束、状态门控、规划脚本 | BRUSHING 接受，其他状态拒绝 |
| 张韬 | 上层目标/轨迹数据格式和频率需求 | 只调用统一 FOC/PID 接口 |
| 陈子龙/诸浩然 | 测量清单和装配偏差需求 | 实物复测记录完整 |
| 梁玮珩 | 截图、数据、测试结论和限制 | 可直接纳入报告与答辩材料 |
