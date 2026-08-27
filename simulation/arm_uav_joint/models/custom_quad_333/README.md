# V7 联合模型说明

本目录承接
`src/SO101_COMPLETE/V7_SO101_SEND_TO_TEAMMATE/custom_quad_333_so101_v7_belly_final`
中的有效运行资源，备份文件和生成缓存不迁移。

## 模型入口

`custom_quad_333.sdf` 是唯一有效的联合模型，内部同时包含：

- custom_quad_333 四旋翼本体、旋翼、IMU、GPS、深度相机及 PX4 Gazebo 插件；
- 0.45 kg 配重；
- SO-101 的视觉、碰撞、质量、惯量和五个关节；
- `base_link -> base_footprint` 的 `so101_mount_joint` 固定挂载关系。

联合模型采用 SDF，而不是 URDF。PX4 的电机、MAVLink、IMU/GPS 等 Gazebo Classic
插件是 SDF 模型的一部分；机械臂的独立 URDF、MoveIt 和 ros2_control 配置继续由
`so-101_description` 提供，本包不复制第二份。

## 文件来源与维护约束

- `custom_quad_333.sdf` 来源于 V7 的最终人工合并模型，是运行时权威文件；
- `custom_quad_333.sdf.jinja` 是 V7 留下的四旋翼模板，不包含最终人工合并的 SO-101
  段，不能直接生成后覆盖 `custom_quad_333.sdf`；
- `meshes/` 和 `model.config` 来源于 V7 custom quad 模型；
- V7 SDF 中硬编码到 `/home/ljj/.../ros2_controllers.yaml` 的
  `gazebo_ros2_control` 插件没有迁移。联合飞行模式按 V7 既定语义固定五个机械臂
  关节；独立机械臂控制使用 `so-101_description`。

不要把 `BEFORE_*`、`*.last_generated`、缓存或另一份 SO-101 网格复制进本目录。
