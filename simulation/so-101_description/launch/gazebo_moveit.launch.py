"""Gazebo Classic + ros2_control + MoveIt 的 SO-101 联调入口."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory('so-101_description')
    pkg_prefix = get_package_prefix('so-101_description')
    gazebo_share = get_package_share_directory('gazebo_ros')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera_view = LaunchConfiguration('use_camera_view')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')

    robot_description = ParameterValue(Command([
        os.path.join(pkg_prefix, 'lib', 'so-101_description', 'xacro_strip_comments.py'), ' ',
        os.path.join(pkg_share, 'urdf', 'so101_arm_camera_gazebo.urdf.xacro'),
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_share, 'worlds', 'offshore_wind_turbine.world'),
            'gui': gui,
            # 先冻结物理环境，等状态与轨迹控制器激活后再解除，防止自由下落
            # 造成初始自碰撞和 MoveIt 起点跳变。
            'pause': 'true',
            'verbose': verbose,
        }.items(),
    )
    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=[
            '-entity', 'so101_arm',
            '-topic', 'robot_description',
            # 与 SRDF 的固定虚拟关节 world -> base_footprint 保持同一零位。
            '-x', '0', '-y', '0', '-z', '0',
        ],
    )
    controller_loader = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'joint_state_broadcaster', 'arm_controller',
            '--inactive',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )
    unpause_physics = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/unpause_physics', 'std_srvs/srv/Empty', '{}',
        ],
        output='screen',
    )
    activate_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--activate', 'joint_state_broadcaster', 'arm_controller',
            '--strict',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )
    initialize_controller = Node(
        package='so-101_description',
        executable='initialize_joint_controller.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[
            moveit_config.to_dict(),
            robot_description_param,
            {'use_sim_time': True, 'publish_robot_description_semantic': True},
        ],
    )
    wind_turbine_scene = Node(
        package='so-101_description', executable='wind_turbine_scene.py', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_moveit',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'moveit.rviz')],
        output='log',
        parameters=[moveit_config.to_dict(), robot_description_param, {'use_sim_time': True}],
    )
    # RViz 的 Image 显示会在部分 Xwayland/GLX 环境中与 MoveIt 的 OGRE
    # 主渲染窗口发生 drawable 上下文冲突。使用独立 Qt 图像窗口显示单目画面，
    # 避免 OGRE 抛出误导性的 "Vertex Buffer: Out of memory"。
    camera_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='wrist_camera_view',
        condition=IfCondition(PythonExpression([
            "'", use_rviz, "' == 'true' and '",
            use_camera_view, "' == 'true'",
        ])),
        arguments=['/so101/wrist_camera/image_raw'],
        output='log',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_camera_view',
            default_value='true',
            description='是否打开独立的单目相机图像窗口',
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        # 场景自包含，禁止 Gazebo 在后台请求在线模型库而阻塞实体插件初始化。
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        # 将 package:// 转换后的 model://so-101_description/... 指向包的 share 根目录。
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_prefix, 'share')),
        gazebo,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description_param, {'use_sim_time': True}],
        ),
        spawn_robot,
        # spawn_entity 成功后再等待 Gazebo 内部 controller_manager；不依赖固定秒数。
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[controller_loader],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_loader,
                on_exit=[unpause_physics],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=unpause_physics,
                on_exit=[activate_controllers],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=activate_controllers,
                on_exit=[initialize_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=initialize_controller,
                on_exit=[
                    move_group,
                    wind_turbine_scene,
                    TimerAction(period=1.0, actions=[rviz]),
                    TimerAction(period=2.0, actions=[camera_view]),
                ],
            )
        ),
    ])
