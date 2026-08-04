from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path


BRIDGE_ROOT = Path(__file__).resolve().parents[1]
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

from haiying_zhixun_bridge.config import load_bridge_config
from haiying_zhixun_bridge.contracts import ACCEPTED_FRAME_ID, TargetPose
from haiying_zhixun_bridge.ik_client import IkClient


DEFAULT_CONFIG_PATH = BRIDGE_ROOT / "config" / "arm_bridge.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="向海鹰智巡本机 IK 服务提交机械臂 XYZ 目标")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--initial-deg", type=float, nargs=5, default=[0.0] * 5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_bridge_config(args.config)
    target = TargetPose(ACCEPTED_FRAME_ID, args.x, args.y, args.z, 0.0, 0.0, 0.0, 1.0)
    client = IkClient(config.ik)
    client.health()
    summary = client.plan_target(target, tuple(args.initial_deg))
    print(json.dumps(asdict(summary), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
