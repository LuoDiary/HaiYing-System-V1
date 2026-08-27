"""
SO-101 机械臂 RViz 显示 launch 文件.

启动: ros2 launch so-101_description display.launch.py
带前缀: ros2 launch so-101_description display.launch.py prefix:=my_arm_

节点:
  - joint_state_publisher_gui — 提供滑块控制各关节角度
  - robot_state_publisher    — 发布 robot_description 到 TF
  - rviz2                    — 3D 可视化
"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def get_robot_description(context, *args, **kwargs):
    """加载并处理 xacro 文件，返回 robot_description."""
    pkg_path = get_package_share_directory('so-101_description')

    prefix = LaunchConfiguration('prefix').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)

    # 使用独立的相机版模型，原始 so101_arm.urdf.xacro 保持不变。
    xacro_file = os.path.join(pkg_path, 'urdf', 'so101_arm_camera.urdf.xacro')

    doc = xacro.process_file(xacro_file, mappings={
        'prefix': prefix,
        'use_sim': use_sim,
        'use_fake_hardware': 'false',
    })

    robot_desc = doc.toprettyxml(indent='  ')

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'so101_arm.rviz')

    return {
        'robot_description': ParameterValue(robot_desc, value_type=str),
        'rviz_config_file': rviz_config_file,
    }


def generate_launch_description():
    # 声明启动参数
    arguments = [
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='link 和 joint 名称前缀'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='是否启用 Gazebo 仿真模式'
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        params = get_robot_description(context)

        # 先让 robot_state_publisher 建立 transient-local robot_description 话题；
        # GUI 随后再读取模型，避免首次启动时滑块未初始化。
        nodes = [
            # 机器人状态发布器 — 计算 TF
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': params['robot_description']}],
                name='robot_state_publisher',
                output='screen',
            ),
            TimerAction(period=0.5, actions=[
                # GUI 关节状态发布器 — 拖动滑块控制各关节
                Node(
                    package='joint_state_publisher_gui',
                    executable='joint_state_publisher_gui',
                    name='joint_state_publisher_gui',
                    parameters=[{'robot_description': params['robot_description']}],
                ),
                # RViz 自动打开，仅展示 RobotModel 与 TF。
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', params['rviz_config_file']],
                    output='screen',
                ),
            ]),
        ]
        return nodes

    return LaunchDescription([
        *arguments,
        OpaqueFunction(function=launch_setup),
    ])
