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
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory('so-101_description')
    pkg_prefix = get_package_prefix('so-101_description')
    joint_pkg_share = get_package_share_directory('arm_uav_joint')
    gazebo_share = get_package_share_directory('gazebo_ros')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera_view = LaunchConfiguration('use_camera_view')
    show_custom_quad = LaunchConfiguration('show_custom_quad')
    show_so101_arm = LaunchConfiguration('show_so101_arm')
    custom_quad_x = LaunchConfiguration('custom_quad_x')
    custom_quad_y = LaunchConfiguration('custom_quad_y')
    custom_quad_z = LaunchConfiguration('custom_quad_z')
    use_px4_plugins = LaunchConfiguration('use_px4_plugins')
    custom_quad_static = LaunchConfiguration('custom_quad_static')
    auto_takeoff = LaunchConfiguration('auto_takeoff')
    handoff_to_qgc = LaunchConfiguration('handoff_to_qgc')
    takeoff_altitude = LaunchConfiguration('takeoff_altitude')
    hover_thrust = LaunchConfiguration('hover_thrust')
    px4_sim_model = LaunchConfiguration('px4_sim_model')
    px4_autopilot_dir = LaunchConfiguration('px4_autopilot_dir')
    px4_build_dir = LaunchConfiguration('px4_build_dir')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    px4_plugin_dir = PathJoinSubstitution([px4_build_dir, 'build_gazebo-classic'])
    px4_model_dir = PathJoinSubstitution([
        px4_autopilot_dir,
        'Tools', 'simulation', 'gazebo-classic', 'sitl_gazebo-classic', 'models',
    ])
    px4_active = PythonExpression([
        "'", use_px4_plugins, "' == 'true' and '", show_custom_quad, "' == 'true'",
    ])
    auto_takeoff_active = PythonExpression([
        "'", auto_takeoff, "' == 'true' and '", use_px4_plugins,
        "' == 'true' and '", show_custom_quad, "' == 'true'",
    ])
    so101_active = PythonExpression(["'", show_so101_arm, "' == 'true'"])
    custom_quad_only_active = PythonExpression([
        "'", show_custom_quad, "' == 'true' and '", show_so101_arm, "' == 'false'",
    ])
    px4_binary = PathJoinSubstitution([px4_build_dir, 'bin', 'px4'])
    px4_etc_dir = PathJoinSubstitution([px4_build_dir, 'etc'])
    px4_rootfs = PathJoinSubstitution([px4_build_dir, 'rootfs'])
    mavros_share = get_package_share_directory('mavros')

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
        condition=IfCondition(so101_active),
        arguments=[
            '-entity', 'so101_arm',
            '-topic', 'robot_description',
            # 与 SRDF 的固定虚拟关节 world -> base_footprint 保持同一零位。
            '-x', '0', '-y', '0', '-z', '0',
        ],
    )
    custom_quad_description = Node(
        package='arm_uav_joint',
        executable='publish_custom_quad_display.py',
        condition=IfCondition(show_custom_quad),
        output='screen',
    )
    spawn_custom_quad = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        condition=IfCondition(show_custom_quad),
        output='screen',
        arguments=[
            '-entity', 'custom_quad_333_display',
            '-topic', '/custom_quad_333_display_description',
            '-x', custom_quad_x,
            '-y', custom_quad_y,
            '-z', custom_quad_z,
        ],
    )
    unpause_custom_quad = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/unpause_physics', 'std_srvs/srv/Empty', '{}',
        ],
        condition=IfCondition(PythonExpression([
            "'", show_custom_quad, "' == 'true' and '",
            show_so101_arm, "' == 'false'",
        ])),
        output='screen',
    )
    px4_sitl = ExecuteProcess(
        cmd=[px4_binary, px4_etc_dir],
        cwd=px4_rootfs,
        additional_env={
            'PX4_SIM_MODEL': px4_sim_model,
            'PX4_HOME_LAT': '47.397742',
            'PX4_HOME_LON': '8.545594',
            'PX4_HOME_ALT': '488.0',
            'NO_PXH': '1',
        },
        condition=IfCondition(px4_active),
        output='screen',
    )
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(mavros_share, 'launch', 'px4.launch')),
        launch_arguments={
            'fcu_url': 'udp://:14540@127.0.0.1:14557',
            'gcs_url': '',
            'tgt_system': '1',
            'tgt_component': '1',
            'fcu_protocol': 'v2.0',
            'namespace': 'mavros',
            'log_output': 'screen',
        }.items(),
        condition=IfCondition(px4_active),
    )
    takeoff = Node(
        package='arm_uav_joint',
        executable='px4_takeoff.py',
        parameters=[
            {
                'takeoff_altitude': takeoff_altitude,
                'hover_thrust': hover_thrust,
                'handoff_to_qgc': ParameterValue(handoff_to_qgc, value_type=bool),
            }
        ],
        condition=IfCondition(auto_takeoff_active),
        output='screen',
    )
    controller_loader = Node(
        package='controller_manager', executable='spawner', output='screen',
        condition=IfCondition(so101_active),
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
        condition=IfCondition(so101_active),
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        condition=IfCondition(so101_active),
        parameters=[
            moveit_config.to_dict(),
            robot_description_param,
            {'use_sim_time': True, 'publish_robot_description_semantic': True},
        ],
    )
    wind_turbine_scene = Node(
        package='so-101_description', executable='wind_turbine_scene.py', output='screen',
        condition=IfCondition(so101_active),
        parameters=[{'use_sim_time': True}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_moveit',
        condition=IfCondition(PythonExpression([
            "'", show_so101_arm, "' == 'true' and '", use_rviz, "' == 'true'",
        ])),
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
            "'", show_so101_arm, "' == 'true' and '", use_rviz, "' == 'true' and '",
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
        DeclareLaunchArgument(
            'show_custom_quad',
            default_value='true',
            description='是否在 Gazebo 中显示 V7 custom quad 挂载模型',
        ),
        DeclareLaunchArgument(
            'show_so101_arm',
            default_value='false',
            description='是否额外生成原始 SO-101 机械臂；PX4 custom quad 模式默认关闭',
        ),
        DeclareLaunchArgument(
            'custom_quad_x',
            default_value='2.57',
            description='custom quad 在 Gazebo world 中的 X 坐标（米）',
        ),
        DeclareLaunchArgument(
            'custom_quad_y',
            default_value='1.17',
            description='custom quad 在 Gazebo world 中的 Y 坐标（米）',
        ),
        DeclareLaunchArgument(
            'custom_quad_z',
            default_value='0.2',
            description='custom quad 在 Gazebo world 中的 Z 坐标（米）',
        ),
        DeclareLaunchArgument(
            'use_px4_plugins',
            default_value='false',
            description='是否加载 PX4 Gazebo Classic 飞行器插件',
        ),
        DeclareLaunchArgument(
            'custom_quad_static',
            default_value='true',
            description='是否将 custom quad 固定为静态模型；PX4 起飞时必须为 false',
        ),
        DeclareLaunchArgument(
            'auto_takeoff',
            default_value='false',
            description='是否自动进入 Offboard、解锁并飞到目标高度',
        ),
        DeclareLaunchArgument(
            'handoff_to_qgc',
            default_value='true',
            description='自动起飞到目标高度后切换 AUTO.LOITER 并释放控制权给 QGC；设为 false 则持续保持 Offboard 定点',
        ),
        DeclareLaunchArgument(
            'takeoff_altitude',
            default_value='2.0',
            description='自动起飞目标高度（MAVROS ENU，米）',
        ),
        DeclareLaunchArgument(
            'hover_thrust',
            default_value='0.65',
            description='完整桥接机械臂负载的 PX4 悬停推力参数（MPC_THR_HOVER）',
        ),
        DeclareLaunchArgument(
            'px4_sim_model',
            default_value='gazebo-classic_iris',
            description='PX4 POSIX airframe 名称；custom quad 使用 Iris 四旋翼参数',
        ),
        DeclareLaunchArgument(
            'px4_autopilot_dir',
            default_value=os.environ.get('PX4_AUTOPILOT_DIR', ''),
            description='PX4-Autopilot 源码目录',
        ),
        DeclareLaunchArgument(
            'px4_build_dir',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('px4_autopilot_dir'), 'build', 'px4_sitl_default',
            ]),
            description='PX4 SITL 构建目录',
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        # 场景自包含，禁止 Gazebo 在后台请求在线模型库而阻塞实体插件初始化。
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('SO101_CUSTOM_QUAD_USE_PX4_PLUGINS', use_px4_plugins),
        SetEnvironmentVariable('SO101_CUSTOM_QUAD_STATIC', custom_quad_static),
        AppendEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH', px4_plugin_dir,
            condition=IfCondition(use_px4_plugins),
        ),
        AppendEnvironmentVariable(
            'GAZEBO_MODEL_PATH', px4_model_dir,
            condition=IfCondition(use_px4_plugins),
        ),
        AppendEnvironmentVariable(
            'LD_LIBRARY_PATH', px4_plugin_dir,
            condition=IfCondition(use_px4_plugins),
        ),
        # 组合 SDF 的四旋翼网格由本包提供；机械臂网格由 so-101_description 提供。
        AppendEnvironmentVariable(
            'GAZEBO_MODEL_PATH', os.path.join(joint_pkg_share, 'models'),
        ),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_prefix, 'share')),
        gazebo,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            condition=IfCondition(so101_active),
            parameters=[robot_description_param, {'use_sim_time': True}],
        ),
        spawn_robot,
        # 默认只生成 custom quad；原始 SO-101 通过 show_so101_arm 显式开启。
        custom_quad_description,
        spawn_custom_quad,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_custom_quad,
                on_exit=[unpause_custom_quad],
            ),
            condition=IfCondition(custom_quad_only_active),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=unpause_custom_quad,
                on_exit=[px4_sitl],
            ),
            condition=IfCondition(custom_quad_only_active),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_custom_quad,
                on_exit=[px4_sitl],
            ),
            condition=IfCondition(so101_active),
        ),
        # 给 PX4 的 TCP/UDP 链路留出启动时间；MAVROS 自身会等待飞控心跳。
        TimerAction(period=8.0, actions=[mavros]),
        TimerAction(period=14.0, actions=[takeoff]),
        # spawn_entity 成功后再等待 Gazebo 内部 controller_manager；不依赖固定秒数。
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[controller_loader],
            ),
            condition=IfCondition(so101_active),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_loader,
                on_exit=[unpause_physics],
            ),
            condition=IfCondition(so101_active),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=unpause_physics,
                on_exit=[activate_controllers],
            ),
            condition=IfCondition(so101_active),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=activate_controllers,
                on_exit=[initialize_controller],
            ),
            condition=IfCondition(so101_active),
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
            ),
            condition=IfCondition(so101_active),
        ),
    ])
