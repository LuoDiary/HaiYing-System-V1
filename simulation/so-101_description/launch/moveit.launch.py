"""SO-101 的标准 MoveIt 规划与执行入口（Fake ros2_control）."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory('so-101_description')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_share, 'urdf', 'so101_arm_camera.urdf.xacro'), ' ',
        'use_fake_hardware:=true',
    ]), value_type=str)
    robot_description_param = {'robot_description': robot_description}

    moveit_config = (
        MoveItConfigsBuilder('so101_arm', package_name='so-101_description')
        .robot_description(Path('config') / 'so101_arm_camera.urdf.xacro')
        .robot_description_semantic(Path('config') / 'so101_arm_camera.srdf')
        .joint_limits(Path('config') / 'joint_limits.yaml')
        .trajectory_execution(Path('config') / 'moveit_controllers.yaml')
        .robot_description_kinematics(Path('config') / 'kinematics.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    controller_config = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'moveit.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description_param],
        output='screen',
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_param, controller_config],
        output='screen',
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            robot_description_param,
            {'publish_robot_description_semantic': True},
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        output='log',
        parameters=[moveit_config.to_dict(), robot_description_param],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        robot_state_publisher,
        controller_manager,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_footprint',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'world',
                '--child-frame-id', 'base_footprint',
            ],
        ),
        # 严格按状态源 → 轨迹控制器 → MoveIt/RViz 启动，消除启动竞态。
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[
                    move_group,
                    TimerAction(period=1.0, actions=[rviz]),
                ],
            )
        ),
    ])
