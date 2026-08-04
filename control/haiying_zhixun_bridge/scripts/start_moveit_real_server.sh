#!/usr/bin/env bash
set -euo pipefail

health_url="http://127.0.0.1:8767/api/health"
if health_response="$(curl --fail --silent --show-error --max-time 2 "${health_url}" 2>/dev/null)" \
  && [[ "${health_response}" == *'"status": "ok"'* ]] \
  && [[ "${health_response}" == *'"hardware_execution_enabled": true'* ]]; then
  echo "MoveIt 实机服务已在运行：${health_url}"
  exit 0
fi

if ss -ltnH '( sport = :8767 )' 2>/dev/null | grep -q .; then
  echo "错误：127.0.0.1:8767 已被其他程序占用。" >&2
  ss -ltnp '( sport = :8767 )' >&2 || true
  exit 1
fi

conda_command="${CONDA_EXE:-$(command -v conda || true)}"
if [[ -z "${conda_command}" ]]; then
  echo "错误：找不到 conda；请先初始化 Conda 或设置 CONDA_EXE。" >&2
  exit 1
fi

conda_environment="${HAIYING_CONDA_ENV:-haiying}"
exec "${conda_command}" run --no-capture-output -n "${conda_environment}" \
  haiying-moveit-real-server \
  --host=127.0.0.1 \
  --port=8767 \
  --robot_port=/dev/ttyACM0 \
  --robot_id=jiebang_follower_arm \
  --hardware_execution_enabled=true \
  --execution_rate_hz=10 \
  --maximum_duration_s=100 \
  --maximum_joint_speed_deg_s=30 \
  --maximum_frame_step_deg=5 \
  --maximum_command_clip_deg=0.5 \
  --maximum_start_error_deg=10 \
  --maximum_feedback_error_deg=3 \
  --joint_limit_deg=89.9
