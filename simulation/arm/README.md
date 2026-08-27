# arm

这是从本仓库 `src/so-101_description` 整理出来的轻量交付副本，保留了现有模型的
5DOF 运动链、坐标系、末端和惯性参数。为方便单独使用，只改了包名和网格路径，没有改
关节、几何或质量。

`urdf/` 里只保留两个自包含 Xacro，不再依赖其他宏文件：

- `urdf/so101_arm.urdf.xacro`：机械臂本体和比赛末端；
- `urdf/so101_arm_camera.urdf.xacro`：机械臂、比赛末端、腕部相机和支架。

两个文件都可以单独交给 `xacro` 展开，没有 include 链，也没有配套宏文件需要一起找。

在工作空间里构建和查看：

```bash
colcon build --packages-select arm --symlink-install
source install/setup.bash
ros2 launch arm display.launch.py
```

不带相机：

```bash
ros2 launch arm display.launch.py with_camera:=false
```

模型自检：

```bash
python3 scripts/validate_delivery.py
```

