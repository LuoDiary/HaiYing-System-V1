# SO-101 5DOF 三维 IK 仿真服务器使用说明

本文说明如何在 Windows 上启动本仓库的 SO-101 五自由度浏览器仿真器、输入末端坐标、播放规划轨迹并完成基本测试。

## 1. 当前模型与运行边界

仿真器直接读取以下用户模型，不使用仓库自建的简化机械臂：

```text
D:\0Project_drone+arm\arm resource\arm_urdf\urdf\so101_arm.urdf.xacro
```

该文件是不带相机的完整 URDF，引用同目录下的 12 个 STL visual。运动链为：

```text
base_footprint
  -> J1_Rotation
  -> J2_Shoulder_Pitch
  -> J3_Elbow_Pitch
  -> J4_Wrist_Pitch
  -> J5_Wrist_Roll
  -> end_effector
```

当前功能仅进行运动学仿真，不打开 `COM5`，不连接舵机，也不会向实机发送角度。

## 2. 首次准备

打开 Anaconda Prompt 或 PowerShell：

```powershell
conda activate lerobot
cd E:\lerobot
python -m pip install -e . --no-deps
```

检查命令是否已经注册：

```powershell
lerobot-ik-sim --help
```

帮助信息中应包含：

```text
--host
--port
--open_browser
--model_root
--model_file
```

浏览器页面通过 CDN 加载 Three.js。第一次打开时需要可以访问互联网；Python 后端、URDF 和 STL 均在本地运行。

## 3. 启动服务器

模型位于上述默认位置时，直接运行：

```powershell
lerobot-ik-sim
```

启动后终端会显示类似信息：

```text
SO-101 5DOF IK simulator: http://127.0.0.1:8766
Model: D:\0Project_drone+arm\arm resource\arm_urdf\urdf\so101_arm.urdf.xacro
Simulation only: no serial port or robot hardware is accessed.
```

如果浏览器没有自动打开，手动访问：

```text
http://127.0.0.1:8766
```

也可以显式指定模型：

```powershell
lerobot-ik-sim `
  --model_root="D:\0Project_drone+arm\arm resource\arm_urdf" `
  --model_file=so101_arm.urdf.xacro
```

不自动打开浏览器：

```powershell
lerobot-ik-sim --open_browser=false
```

端口被占用时改用其他端口：

```powershell
lerobot-ik-sim --port=8767
```

随后访问 `http://127.0.0.1:8767`。

## 4. 页面操作

页面中央是真实 STL 机械臂三维场景：

- 鼠标左键拖动：旋转视角。
- 鼠标滚轮：缩放。
- 鼠标右键拖动：平移视角。
- 红、绿、蓝轴分别表示 `+X`、`+Y`、`+Z`。
- 紫色球表示目标位置。
- 青色曲线表示规划后 TCP 的运动路径。

目标坐标使用米，参考坐标系是 URDF 的 `base_footprint`。操作步骤：

1. 在左侧输入目标 `X/Y/Z`。
2. 点击“计算 IK 并规划”。
3. 确认右侧显示“规划成功”。
4. 查看五个关节目标角、终点误差和迭代次数。
5. 确认“自碰撞检查”显示 `91 / 91 帧通过`。
6. 点击“播放”，观察真实网格沿 91 帧轨迹运动。
7. 使用“暂停”“复位”或时间轴检查任意一帧。

页面首次加载时会自动填入一个接近零位 TCP 的测试目标，可以直接点击“计算 IK 并规划”。已验证的一组输入为：

```text
X =  0.0305
Y = -0.1798
Z =  0.1762
```

该目标的浏览器测试结果约为：

```text
规划状态：成功
位置误差：0.02 mm
轨迹帧数：91
```

`J5_Wrist_Roll` 会正常显示。由于当前 IK 只约束 TCP 位置，而滚转关节通常不改变 TCP 位置，所以它可能保持当前角度或接近 `0°`；这不表示第五个关节缺失。

### 自碰撞目标示例

以下目标曾经被旧规划器错误判为成功：

```text
X = 0.0900
Y = 0.1050
Z = 0.2900
```

旧解会让底座和手臂结构相交。当前版本会检查用户 URDF 的 collision STL 包络和完整 91 帧轨迹；如果没有安全轨迹，页面会显示首次碰撞帧和 link 对，不允许播放碰撞轨迹。

## 5. 不打开页面的服务器测试

保持服务器运行，另开一个 PowerShell 窗口。

### 5.1 健康检查

```powershell
Invoke-RestMethod http://127.0.0.1:8766/api/health
```

预期结果：

```text
status hardware_connected
------ ------------------
ok                  False
```

`hardware_connected=False` 是预期行为，表示当前只运行仿真。

### 5.2 检查模型来源

```powershell
$model = Invoke-RestMethod http://127.0.0.1:8766/api/model
$model.model_name
$model.model_source
$model.joint_names
```

应看到：

```text
so101_arm（用户提供，无相机）
D:\0Project_drone+arm\arm resource\arm_urdf\urdf\so101_arm.urdf.xacro
J1_Rotation
J2_Shoulder_Pitch
J3_Elbow_Pitch
J4_Wrist_Pitch
J5_Wrist_Roll
```

### 5.3 调用 IK API

```powershell
$body = @{
  target_position_m = @(0.0305, -0.1798, 0.1762)
  initial_joint_angles_deg = @(0, 0, 0, 0, 0)
} | ConvertTo-Json

$result = Invoke-RestMethod `
  -Method Post `
  -Uri http://127.0.0.1:8766/api/plan `
  -ContentType "application/json" `
  -Body $body

$result.success
$result.message
$result.error_m
$result.joint_angles_deg
$result.trajectory_deg.Count
$result.collision_free
$result.first_collision_frame
$result.collision_link_pairs
```

安全目标预期 `success=True`、`collision_free=True`，轨迹数量为 `91`，误差应小于 `0.002 m`。

## 6. 自动化测试

安装开发测试依赖后运行：

```powershell
cd E:\lerobot
python -m pytest tests\simulators\test_so101_5dof_simulator.py -q
```

当前已验证结果：

```text
10 passed
```

MATLAB 交叉验证脚本位于：

```text
E:\lerobot\examples\so101_5dof\verify_ik_matlab.m
```

它使用相同的 `base_footprint`、`end_effector` 和无相机 URDF。当前代表性测试结果为：

```text
Python 规划角的 MATLAB FK 目标误差：0.240815 mm
MATLAB 独立 IK 误差：约 0.000001 mm
```

## 7. 停止服务器

服务器在当前终端前台运行时，按：

```text
Ctrl+C
```

浏览器页面随即失去后端连接。关闭浏览器标签页本身不会停止服务器。

## 8. 常见问题

### `lerobot-ik-sim` 不是内部或外部命令

确认已经激活正确环境并刷新可编辑安装：

```powershell
conda activate lerobot
cd E:\lerobot
python -m pip install -e . --no-deps
```

也可以直接使用完整路径：

```powershell
D:\software\anaconda\envs\lerobot\Scripts\lerobot-ik-sim.exe
```

### 提示端口已占用

改用其他端口，例如：

```powershell
lerobot-ik-sim --port=8767
```

### 页面打开但机械臂不显示

依次检查：

1. 终端是否仍在运行。
2. `/api/model` 是否返回正确模型路径。
3. 模型目录和 `meshes` 子目录是否仍存在。
4. 浏览器是否可以加载 Three.js CDN。
5. 浏览器控制台是否出现网络或 WebGL 错误。

### 提示模型或 STL 不存在

仿真器不会复制模型。如果移动了 `arm_urdf` 文件夹，启动时必须通过 `--model_root` 指向新目录。

### 目标不可达

不可达、超出关节限位或数值迭代未收敛时，后端会返回失败和最小误差，不会伪造轨迹。先选择更靠近当前 TCP 的目标逐步测试。

### 目标位置可达但提示自碰撞

这表示 IK 找到了满足 TCP 位置的关节角，但从当前姿态到候选角的轨迹至少有一帧发生结构包络相交。页面不会生成播放轨迹。当前碰撞模型使用 STL 的 OBB 保守包络并保留 2 mm 总安全余量，因此比只看画面更严格。

## 9. 安全提醒

本页面当前不能控制实机。现有自碰撞检查也不等于完整实机安全系统；后续接入实机仍应增加舵机角度映射校验、速度和步长限制、环境碰撞、急停以及空载小幅动作验证，不能把仿真角度未经检查直接发送到机械臂。
