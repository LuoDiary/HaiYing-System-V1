# 建模仿真组

本目录存放 Gazebo 世界、URDF/Xacro、STL、MoveIt、ros2_control、RViz 与启动文件。

## ROS 2 功能包

| 路径 | 作用 |
|---|---|
| `arm/` | 轻量、自包含的 SO-101 五自由度 URDF；供 IK 服务、TF 和模型交付使用 |
| `so-101_description/` | Gazebo Classic、MoveIt 2、ros2_control、腕部相机和风机场景联调包 |

两个包均可由工作空间根目录直接构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths simulation --packages-select arm so-101_description --symlink-install
source install/setup.bash
```

仅查看轻量模型：

```bash
ros2 launch arm display.launch.py
```

启动 Gazebo、MoveIt 和 RViz：

```bash
ros2 launch so-101_description gazebo_moveit.launch.py
```

无图形冒烟测试：

```bash
ros2 launch so-101_description gazebo_moveit.launch.py gui:=false use_rviz:=false
ros2 run so-101_description plan_execute_smoke_test.py
```

为控制仓库体积，`so-101_description` 只提交运行时实际引用的 STL；CAD 转换临时工程、
重复网格、ZIP 和 RViz 备份未提交。模型质量和惯量仍是 CAD 初始值，只用于几何、TF、
规划和接口联调，不能直接作为无人机挂载后的高可信动力学参数。
