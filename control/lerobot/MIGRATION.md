# Haiying LeRobot runtime snapshot

This directory was migrated from `/home/fog/lerobot` on 2026-08-04 so the
Haiying workspace does not depend on files outside `/home/fog/arm_ws`.

The source snapshot contains the custom SO-101 5DoF robot, local IK server,
browser simulator and guarded real-arm executor. Workspace-specific changes:

- the default model root resolves to `src/arm_urdf` (or
  `HAIYING_ARM_URDF_ROOT` when set);
- the default Linux serial port is `/dev/ttyACM0`;
- the package dependency set and console scripts are reduced to the local IK,
  calibration and Feetech-arm runtime; ML training, datasets and cameras are
  intentionally not installed;
- PyTorch is optional for the runtime-only type aliases, and Torch-specific
  device helpers are not re-exported by the lightweight `lerobot.utils` API.
- Three.js 0.185.1 (`three.module.js` and its `three.core.js` runtime module),
  OrbitControls and STLLoader are served from local vendored assets so the
  simulator page does not depend on jsDelivr or internet access.

For the current Ubuntu/Jetson workflow, this snapshot is installed from the
HaiYing repository root with `python -m pip install -e ./control/lerobot` in the
Python 3.12 `haiying` Conda environment. The official real-arm path is the ROS
MoveIt/Gazebo GUI plus `haiying-moveit-real-server`; the local IK server remains
only for compatibility and requires `HAIYING_ARM_URDF_ROOT` when the historical
`src/arm_urdf` tree is absent.

The original `pyproject.toml`, `uv.lock`, documentation and focused tests are
kept in the snapshot for traceability. `COLCON_IGNORE` prevents ROS colcon from
treating this vendored tree as a workspace package.
