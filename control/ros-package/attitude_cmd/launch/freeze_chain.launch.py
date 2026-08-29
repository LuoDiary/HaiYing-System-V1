#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'attitude_cmd'


def generate_launch_description():
    params = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', 'config', 'freeze_params.yaml')
    params = os.path.abspath(params)

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_fake_px4', default_value='false',
        description='start the loopback fake_px4 peer (standalone smoke tests only)'))

    ld.add_action(Node(
        package=PKG, executable='attitude_cmd_node', name='attitude_cmd_node',
        parameters=[params], output='screen'))

    ld.add_action(Node(
        package=PKG, executable='cmd_vel_to_attitude',
        name='cmd_vel_to_attitude', parameters=[params], output='screen'))

    ld.add_action(Node(
        package=PKG, executable='fake_px4', name='fake_px4',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_fake_px4'))))

    return ld