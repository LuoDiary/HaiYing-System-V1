from __future__ import annotations

import argparse
import sys
from pathlib import Path


BRIDGE_ROOT = Path(__file__).resolve().parents[1]
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

from haiying_zhixun_bridge.config import load_bridge_config
from haiying_zhixun_bridge.real_arm import RealArmRequest, build_real_arm_command, run_real_arm_command


DEFAULT_CONFIG_PATH = BRIDGE_ROOT / "config" / "arm_bridge.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="海鹰智巡 LeRobot 五轴实机安全控制入口")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--executable", default="lerobot-ik-real")
    parser.add_argument("--allow-hardware", action="store_true")
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("inspect")
    jog = subparsers.add_parser("jog")
    jog.add_argument("--joint", required=True)
    jog.add_argument("--delta-deg", type=float, required=True)
    dry_run = subparsers.add_parser("dry-run")
    dry_run.add_argument("--plan-id", required=True)
    execute = subparsers.add_parser("execute")
    execute.add_argument("--plan-id", required=True)
    execute.add_argument("--confirm-execute", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_bridge_config(args.config)
    if config.safety.simulation_only and args.action != "dry-run" and not args.allow_hardware:
        raise ValueError("当前配置默认 simulation_only；连接实机必须显式提供 --allow-hardware")
    request = RealArmRequest(
        action=args.action,
        joint=getattr(args, "joint", None),
        delta_deg=getattr(args, "delta_deg", None),
        plan_id=getattr(args, "plan_id", None),
        confirm_execute=bool(getattr(args, "confirm_execute", False)),
    )
    command = build_real_arm_command(args.executable, config.robot, config.safety, request)
    raise SystemExit(run_real_arm_command(command))


if __name__ == "__main__":
    main()
