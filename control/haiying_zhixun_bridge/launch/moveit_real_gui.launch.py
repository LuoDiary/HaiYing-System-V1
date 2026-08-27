from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("so-101_description")
    use_camera_view = LaunchConfiguration("use_camera_view")
    use_rviz = LaunchConfiguration("use_rviz")
    show_control_gui = LaunchConfiguration("show_control_gui")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    verbose = LaunchConfiguration("verbose")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, "launch", "gazebo_moveit.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "use_camera_view": use_camera_view,
            "gui": gazebo_gui,
            "verbose": verbose,
        }.items(),
    )
    gui = Node(
        package="haiying_zhixun_bridge",
        executable="haiying-moveit-real-gui",
        name="haiying_moveit_real_gui",
        condition=IfCondition(show_control_gui),
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_camera_view", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("show_control_gui", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="false"),
            simulation,
            TimerAction(period=8.0, actions=[gui]),
        ]
    )
