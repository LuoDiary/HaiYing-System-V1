# 控制与执行组

本目录存放飞行控制与机械臂控制 ROS 2 功能包，通信接口必须遵循
[`docs/ROS2_Interface_V1.md`](../docs/ROS2_Interface_V1.md)。

## 现有功能包

| 路径 | 负责人 | 作用 |
|---|---|---|
| `haiying_zhixun_bridge/` | 曹圆圆 | 订阅 `/arm/target_pose` 与 `/system/current_state`，完成机械臂规划门控、MoveIt 仿真轨迹验证和 LeRobot 实机安全执行 |

机械臂包默认只在系统状态为 `BRUSHING` 时接受目标，并且 ROS 节点默认只规划、
不自动连接或移动实体机械臂。构建和实机操作步骤见
[`haiying_zhixun_bridge/README.md`](haiying_zhixun_bridge/README.md)。
