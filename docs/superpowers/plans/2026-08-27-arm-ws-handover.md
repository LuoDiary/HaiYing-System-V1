# arm_ws 主机迁移交接说明实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 根据当前 `arm_ws` 的实际内容，在 `src/HaiYing-System-V1/docs/` 生成一份可供新主机重建、仿真和实机交接使用的说明文档。

**Architecture:** 交接说明以 Git 仓库 `HaiYing-System-V1` 为唯一正式源码，区分系统 ROS 2 环境、独立 `haiying` Python 环境、可选 PX4 SITL 和实体机械臂校准数据。文档同时列出仓库树、目录职责、安装顺序、验证命令、安全边界和不应迁移的旧目录。

**Tech Stack:** Markdown、ROS 2 Humble、Gazebo Classic、MoveIt 2、PX4 SITL、MAVROS、Python 3.10/3.12、LeRobot 0.6.1。

**Spec:** 当前工作区 `/home/fog/arm_ws`、仓库 `src/HaiYing-System-V1` 及其现有 README、package.xml、启动脚本和配置文件。

## Global Constraints

- 新主机只克隆 `HaiYing-System-V1` 的 `main` 分支，不复制工作区中的历史重复包和编译产物。
- ROS 2 Humble 使用系统 Python 3.10；LeRobot、IK 和实机服务使用独立 Python 3.12 `haiying` 环境。
- 不把校准 JSON、串口凭据、模型权重、数据集或任何口令提交到 Git。
- 联合仿真使用 `simulation/arm_uav_joint/models/custom_quad_333/custom_quad_333.sdf`。
- 所有实机动作必须保留 `--allow-hardware`、`--confirm-execute` 和急停措施。

---

### Task 1: 记录当前工作区事实

**Files:**
- Inspect: `/home/fog/arm_ws/src/HaiYing-System-V1`
- Inspect: `/home/fog/arm_ws/environment-haiying.yml`

- [x] 核对远程仓库、分支、最新提交、ROS 2 包清单、联合仿真入口、桥接入口、端口、串口和校准 ID。
- [x] 区分正式仓库内容、仓库外历史目录、`build/install/log` 产物和外部 LeRobot 源码。

### Task 2: 编写主机迁移交接文档

**Files:**
- Create: `docs/ARM_WS_HANDOVER.md`

- [x] 写入完整仓库树和各核心目录职责。
- [x] 写入 Git 克隆、ROS 依赖、`rosdep`、构建、环境刷新和验证命令。
- [x] 写入 `arm_uav_joint` 静态展示、PX4/MAVROS、独立机械臂和 `haiying_zhixun_bridge` 的操作路径。
- [x] 写入 LeRobot 版本、独立 Python 环境、IK 服务、MoveIt 实机服务、串口和校准文件迁移说明。
- [x] 写入安全边界、当前未验收事项、Jetson 注意事项和旧目录清理规则。

### Task 3: 校验并提交

**Files:**
- Verify: `docs/ARM_WS_HANDOVER.md`
- Commit: Git history on `main`

- [x] 检查文档中的命令、路径、包名和仓库树与实际文件一致。
- [x] 检查文档不包含密码、校准内容和不可迁移的本机绝对路径。
- [x] 使用 Markdown 内容检查和 Git diff 检查后提交。
