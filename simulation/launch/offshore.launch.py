#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    home = os.path.expanduser('~')
    
    return LaunchDescription([

        # ============================================================
        # 1. 启动 PX4 + Gazebo（后台运行）
        # ============================================================
        ExecuteProcess(
            cmd=['make', 'px4_sitl', 'gazebo'],
            cwd=os.path.join(home, '桌面/PX4-Autopilot'),
            output='screen',
            shell=True,
        ),

        # ============================================================
        # 2. 等待 6 秒后加载无人机模型
        # ============================================================
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                        '-file', os.path.join(home, '桌面/HaiYing-System-V1/simulation/models/real_uav/iris.sdf'),
                        '-entity', 'real_uav',
                        '-x', '0', '-y', '0', '-z', '0.2'
                    ],
                    output='screen',
                    shell=True,
                ),
            ],
        ),

        # ============================================================
        # 3. 等待 8 秒后启动 MAVROS
        # ============================================================
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch', 'mavros', 'px4.launch',
                        'fcu_url:=udp://:14540@127.0.0.1:14540'
                    ],
                    output='screen',
                    shell=True,
                ),
            ],
        ),

        # ============================================================
        # 4. 等待 12 秒后启动 TF 树节点
        # ============================================================
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'python3',
                        os.path.join(home, '桌面/HaiYing-System-V1/simulation/scripts/tf_broadcaster.py')
                    ],
                    output='screen',
                    shell=True,
                ),
            ],
        ),

        # ============================================================
        # 5. 等待 15 秒后启动 vision_control 节点
        # ============================================================
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'cd ~/桌面/uav_control && source install/setup.bash && ros2 run uav_control vision_control'
                    ],
                    output='screen',
                    shell=True,
                ),
            ],
        ),

    ])
