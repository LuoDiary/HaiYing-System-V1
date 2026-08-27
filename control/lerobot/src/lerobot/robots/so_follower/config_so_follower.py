#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@dataclass
class SOFollowerConfig:
    """Base configuration class for SO Follower robots."""

    # Port to connect to the arm
    port: str

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a dictionary that maps motor
    # names to the max_relative_target value for that motor.
    max_relative_target: float | dict[str, float] | None = None

    # cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = True

    # Position-mode PID gains written to Feetech STS3215 motors at connect time.
    # 实机稳定参数：适度降低 P 避免高刚度振荡，保留 D 阻尼抑制回摆。
    # I=0 避免积分饱和与低速爬行抖动；如仍有静态余差可逐步增大到 2~4。
    position_p_coefficient: int = 24
    position_i_coefficient: int = 0
    position_d_coefficient: int = 32

    # 舵机内部运动平滑（抑制阶跃冲击导致的抖动）：
    # - goal_time_ms: 写入 Goal_Time(0x2C)，单位毫秒，默认 0 关闭。
    #   注意：当控制频率高于 1/goal_time 时，新目标会在上一段运动完成前到达，
    #   舵机会被反复重定向反而更抖。优先通过提高控制频率 + 减小单帧步进实现平滑；
    #   若在低频(≤20Hz)下想限定单次运动时长，可设为此帧周期稍大的毫秒值。
    # - acceleration: 写入 Acceleration(0x29)，0~255。configure_motors 默认 254（最大加速）
    #   会加剧阶跃冲击；降到 80~160 可限制启动加速度，明显减少过冲与到位回摆。
    # - dead_zone: 写入 CW/CCW_Dead_Zone(0x1A/0x1B)，单位舵机刻度(1/4096 圈)，
    #   消除到位后由量化误差触发的微小高频修正（微振）。
    goal_time_ms: int = 0
    acceleration: int = 100
    dead_zone: int = 5

    # Number of extra attempts when a `sync_read` of the motors fails. Feetech buses can occasionally
    # return a corrupted status packet ("Incorrect status packet!"), especially when several joints move
    # at once, which otherwise aborts the control loop. Retries are immediate (no sleep) and only happen on
    # failure, so the steady-state read cost is unchanged.
    num_read_retries: int = 2


@RobotConfig.register_subclass("so101_follower")
@RobotConfig.register_subclass("so100_follower")
@dataclass
class SOFollowerRobotConfig(RobotConfig, SOFollowerConfig):
    pass


@RobotConfig.register_subclass("so101_follower_5dof")
@dataclass
class SO101Follower5DOFConfig(SOFollowerRobotConfig):
    pass


SO100FollowerConfig = SOFollowerRobotConfig
SO101FollowerConfig = SOFollowerRobotConfig
