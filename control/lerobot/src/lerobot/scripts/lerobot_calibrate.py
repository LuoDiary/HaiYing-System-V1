# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Workspace-local helper to calibrate the SO-101 5DoF follower.

Example:

```shell
lerobot-calibrate \
    --robot.type=so101_follower_5dof \
    --robot.port=/dev/ttyACM0 \
    --robot.id=jiebang_follower_arm
```
"""

import logging
from dataclasses import asdict, dataclass
from pprint import pformat

import draccus

from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot.robots import so_follower  # noqa: F401 - registers SO follower configs
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging


@dataclass
class CalibrateConfig:
    robot: RobotConfig

    def __post_init__(self):
        if self.robot.type != "so101_follower_5dof":
            raise ValueError("This workspace runtime calibrates only so101_follower_5dof.")


@draccus.wrap()
def calibrate(cfg: CalibrateConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    device = make_robot_from_config(cfg.robot)

    device.connect(calibrate=False)

    try:
        device.calibrate()
    finally:
        device.disconnect()


def main():
    register_third_party_plugins()
    calibrate()


if __name__ == "__main__":
    main()
