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
from haiying_zhixun_bridge.contracts import ACCEPTED_FRAME_ID, MissionState, PlanSummary, TargetPose
from haiying_zhixun_bridge.coordinator import ArmCoordinator, StateGateError
from haiying_zhixun_bridge.ik_client import IkClient


DEFAULT_CONFIG_PATH = BRIDGE_ROOT / "config" / "arm_bridge.yaml"


class OfflinePlanner:
    def plan_target(self, target: TargetPose, initial_joint_angles_deg: tuple[float, ...]) -> PlanSummary:
        return PlanSummary(
            plan_id="offlineplan001",
            target_position_m=target.position_m,
            reached_position_m=target.position_m,
            error_m=0.0,
            joint_angles_deg=initial_joint_angles_deg,
            trajectory_frames=91,
            collision_free=True,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="模拟海鹰智巡五阶段任务与机械臂状态门控")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--x", type=float, default=0.005534)
    parser.add_argument("--y", type=float, default=-0.179839)
    parser.add_argument("--z", type=float, default=0.171219)
    parser.add_argument("--offline", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_bridge_config(args.config)
    planner = OfflinePlanner() if args.offline else IkClient(config.ik)
    coordinator = ArmCoordinator(planner, (0.0,) * 5)
    target = TargetPose(ACCEPTED_FRAME_ID, args.x, args.y, args.z, 0.0, 0.0, 0.0, 1.0)
    events: list[dict[str, object]] = []
    for state in MissionState:
        coordinator.update_state(state.value)
        event: dict[str, object] = {"state": state.value}
        try:
            event["plan"] = asdict(coordinator.plan_target(target))
            event["arm_target_accepted"] = True
        except StateGateError as error:
            event["arm_target_accepted"] = False
            event["reason"] = str(error)
        events.append(event)
    print(json.dumps(events, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
