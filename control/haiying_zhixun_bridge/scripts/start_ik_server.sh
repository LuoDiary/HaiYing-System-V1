#!/usr/bin/env bash
set -euo pipefail

bridge_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
repository_root="$(cd -- "${bridge_root}/../.." && pwd)"
health_url="http://127.0.0.1:8766/api/health"

if health_response="$(curl --fail --silent --show-error --max-time 2 "${health_url}" 2>/dev/null)" \
  && [[ "${health_response}" == *'"status": "ok"'* ]] \
  && [[ "${health_response}" == *'"hardware_connected": false'* ]]; then
  echo "IK 服务已在运行：${health_url}"
  echo "仿真页面：http://127.0.0.1:8766/"
  exit 0
fi

if ss -ltnH '( sport = :8766 )' 2>/dev/null | grep -q .; then
  echo "错误：127.0.0.1:8766 已被其他程序占用，但不是健康的海鹰 IK 服务。" >&2
  ss -ltnp '( sport = :8766 )' >&2 || true
  exit 1
fi

conda_command="${CONDA_EXE:-$(command -v conda || true)}"
if [[ -z "${conda_command}" ]]; then
  echo "错误：找不到 conda；请先初始化 Conda 或设置 CONDA_EXE。" >&2
  exit 1
fi

conda_environment="${HAIYING_CONDA_ENV:-haiying}"
model_root="${HAIYING_ARM_URDF_ROOT:-${repository_root}/simulation/arm_urdf}"
exec "${conda_command}" run --no-capture-output -n "${conda_environment}" lerobot-ik-sim \
  --host=127.0.0.1 \
  --port=8766 \
  --open_browser=false \
  --model_root="${model_root}" \
  --model_file=so101_arm.urdf.xacro
