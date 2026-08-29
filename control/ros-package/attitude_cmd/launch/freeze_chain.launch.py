#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

PKG = 'attitude_cmd'


def generate_launch_description():
    home = os.path.expanduser('~')
    core_repo = os.path.join(home, 'Desktop/HaiYing-System-V1')
    approach_script = os.path.join(core_repo, 'scripts/approach_controller.py')
    params = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', 'config', 'freeze_params.yaml')
    params = os.path.abspath(params)

    ld = LaunchDescription()

    ld.add_action(Node(
        package=PKG, executable='fake_px4', name='fake_px4',
        output='screen'))

    ld.add_action(Node(
        package=PKG, executable='attitude_cmd_node', name='attitude_cmd_node',
        parameters=[params], output='screen'))

    ld.add_action(Node(
        package=PKG, executable='cmd_vel_to_attitude',
        name='cmd_vel_to_attitude', parameters=[params], output='screen'))

    ld.add_action(ExecuteProcess(
        cmd=['python3', approach_script],
        output='screen', shell=False))

    return ld