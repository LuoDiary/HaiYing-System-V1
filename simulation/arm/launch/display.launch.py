import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    package_path = get_package_share_directory("arm_urdf")
    with_camera = LaunchConfiguration("with_camera").perform(context).lower() == "true"
    model_name = (
        "so101_arm_camera.urdf.xacro"
        if with_camera
        else "so101_arm.urdf.xacro"
    )
    model_path = os.path.join(package_path, "urdf", model_name)
    robot_description = xacro.process_file(model_path).toprettyxml(indent="  ")

    common_parameters = {
        "robot_description": ParameterValue(robot_description, value_type=str)
    }
    rviz_path = os.path.join(package_path, "rviz", "arm.rviz")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[common_parameters],
            output="screen",
        ),
        TimerAction(
            period=0.5,
            actions=[
                Node(
                    package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    parameters=[common_parameters],
                ),
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", rviz_path],
                    output="screen",
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "with_camera",
                default_value="true",
                description="是否显示仓库里的腕部相机与支架",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
