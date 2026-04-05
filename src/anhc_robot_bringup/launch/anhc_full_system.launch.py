import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, RegisterEventHandler,
    ExecuteProcess, LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, Command,
    PathJoinSubstitution, FindExecutable, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_sim         = get_package_share_directory('anhc_robot_simulation')
    pkg_description = get_package_share_directory('anhc_robot_description')
    pkg_nav         = get_package_share_directory('anhc_robot_navigation')

    nav_config  = os.path.join(pkg_nav,         'config', 'anhc_navigation.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz',   'anhc_robot.rviz')
    world_file  = os.path.join(pkg_sim,         'worlds', 'anhc_simple_world.sdf')
    xacro_file  = os.path.join(pkg_description, 'urdf',   'main_robot.urdf.xacro')

    # ── Args ─────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_x',   default_value='-7.0'),
        DeclareLaunchArgument('robot_y',   default_value='0.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_x',    default_value='7.0'),
        DeclareLaunchArgument('goal_y',    default_value='0.0'),
        DeclareLaunchArgument('auto_goal', default_value='true'),
        DeclareLaunchArgument('headless',  default_value='false'),
        DeclareLaunchArgument('rviz',      default_value='true'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_x   = LaunchConfiguration('robot_x')
    robot_y   = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')
    goal_x    = LaunchConfiguration('goal_x')
    goal_y    = LaunchConfiguration('goal_y')
    auto_goal = LaunchConfiguration('auto_goal')
    headless  = LaunchConfiguration('headless')
    show_rviz = LaunchConfiguration('rviz')

    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ',
                     xacro_file, ' use_sim:=true']),
            value_type=str,
        )
    }

    # ── Gazebo ────────────────────────────────────────────────
    gz_args = PythonExpression([
        '"-r --headless-rendering -s " if "',
        headless, '" == "true" else "-r "',
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
        ])]),
        launch_arguments={
            'gz_args': [gz_args, world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Clock bridge — FIRST, before everything else ─────────
    # FIX: clock bridge must be up before RSP and tf_bridge start
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='anhc_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        # No use_sim_time here — this node IS the clock source
    )

    # ── RSP — starts AFTER clock bridge is confirmed up ──────
    # FIX: use_sim_time=true, but RSP waits until T=4s
    #      so /clock is already publishing before RSP starts
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='anhc_robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ── Sensor bridge ─────────────────────────────────────────
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='anhc_sensor_bridge',
        arguments=[
            '/anhc/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/anhc/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/anhc/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/anhc/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/anhc/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/anhc/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── TF bridge ─────────────────────────────────────────────
    # FIX: use_sim_time=true so TF timestamps match Gazebo clock
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='anhc_tf_bridge',
        arguments=[
            '/model/anhc_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[('/model/anhc_robot/tf', '/tf')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Joint state bridge ────────────────────────────────────
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='anhc_joint_state_bridge',
        arguments=[
            '/world/anhc_simple_world/model/anhc_robot/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[(
            '/world/anhc_simple_world/model/anhc_robot/joint_state',
            '/joint_states',
        )],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Spawn robot ───────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='anhc_robot_spawner',
        arguments=[
            '-name', 'anhc_robot', '-topic', '/robot_description',
            '-x', robot_x, '-y', robot_y, '-z', '0.15', '-Y', robot_yaw,
        ],
        output='screen',
    )

    # ── Controllers ───────────────────────────────────────────
    jsb = Node(
        package='controller_manager', executable='spawner',
        name='anhc_jsb_spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    ddc = Node(
        package='controller_manager', executable='spawner',
        name='anhc_dd_spawner',
        arguments=['anhc_diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawn_jsb = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[TimerAction(period=3.0, actions=[jsb,
            LogInfo(msg='[ANHC] ⚙️  joint_state_broadcaster spawned')])]
    ))
    spawn_dd = RegisterEventHandler(OnProcessExit(
        target_action=jsb,
        on_exit=[TimerAction(period=1.0, actions=[ddc,
            LogInfo(msg='[ANHC] ⚙️  diff_drive_controller spawned')])]
    ))

    # ── Static TF map→odom ────────────────────────────────────
    # FIX: use_sim_time=true — static_transform_publisher must
    #      stamp its TF with sim time, not wall time
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='anhc_static_tf_map_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── Navigation ────────────────────────────────────────────
    map_handler = Node(
        package='anhc_robot_navigation',
        executable='anhc_map_handler_node',
        name='anhc_map_handler_node', output='screen',
        parameters=[nav_config, {'use_sim_time': use_sim_time}],
    )
    planner = Node(
        package='anhc_robot_navigation',
        executable='anhc_planner_node',
        name='anhc_planner_node', output='screen',
        parameters=[nav_config, {'use_sim_time': use_sim_time}],
    )
    path_follower = Node(
        package='anhc_robot_navigation',
        executable='anhc_path_follower_node',
        name='anhc_path_follower_node', output='screen',
        parameters=[nav_config, {'use_sim_time': use_sim_time}],
    )

    # ── RViz ─────────────────────────────────────────────────
    # FIX: use_sim_time=true so RViz interprets TF timestamps
    #      using sim clock, not wall clock → no TF_OLD_DATA
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='anhc_rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(show_rviz),
    )

    # ── Auto goal ─────────────────────────────────────────────
    auto_goal_sender = TimerAction(period=25.0, actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once',
                '/anhc/goal', 'geometry_msgs/msg/PoseStamped',
                ['{header: {frame_id: odom}, pose: {position: {x: ',
                 goal_x, ', y: ', goal_y,
                 ', z: 0.0}, orientation: {w: 1.0}}}'],
            ],
            output='screen', name='anhc_auto_goal',
            condition=IfCondition(auto_goal),
        ),
        LogInfo(msg=['[ANHC] 🎯 Goal → (', goal_x, ', ', goal_y, ')'],
                condition=IfCondition(auto_goal)),
    ])

    # ══════════════════════════════════════════════════════════
    # STARTUP SEQUENCE — carefully timed to fix TF_OLD_DATA
    #
    # Key insight: /clock must be publishing BEFORE any node
    # with use_sim_time=true tries to stamp a TF message.
    #
    # T=0s  Gazebo starts (begins publishing /clock internally)
    # T=2s  clock_bridge → /clock available in ROS 2
    # T=4s  RSP + all bridges start (they now see /clock)
    #        static TF map→odom (stamped with sim time)
    # T=6s  Spawn robot
    # T=9s  joint_state_broadcaster
    # T=10s diff_drive_controller
    # T=14s Navigation + RViz (all data flowing by now)
    # T=25s Auto goal
    # ══════════════════════════════════════════════════════════
    return LaunchDescription([
        *args,
        LogInfo(msg='\n╔══════════════════════════════════════════╗\n'
                    '║     ANHC ROBOT FULL SYSTEM LAUNCH        ║\n'
                    '║  4-Wheel Mobile Robot + A* Navigation    ║\n'
                    '╚══════════════════════════════════════════╝'),

        # T=0s — Gazebo only
        gazebo,

        # T=2s — Clock bridge FIRST (before any use_sim_time node)
        TimerAction(period=2.0, actions=[
            clock_bridge,
            LogInfo(msg='[ANHC] ✅ T=2s  /clock bridge UP'),
        ]),

        # T=4s — Everything that needs sim time
        # RSP, bridges, static TF all start together
        # /clock is guaranteed to be publishing by now
        TimerAction(period=4.0, actions=[
            robot_state_publisher,
            sensor_bridge,
            tf_bridge,
            joint_state_bridge,
            static_tf,
            LogInfo(msg='[ANHC] ✅ T=4s  RSP + bridges + static TF UP'),
        ]),

        # T=6s — Spawn robot (RSP must be up first)
        TimerAction(period=6.0, actions=[
            spawn_robot,
            LogInfo(msg='[ANHC] ✅ T=6s  Spawning robot at (-7, 0)'),
        ]),

        # T=9s, T=10s — Controllers (via event chain)
        spawn_jsb,
        spawn_dd,

        # T=14s — Navigation + RViz
        TimerAction(period=14.0, actions=[
            map_handler,
            planner,
            path_follower,
            rviz,
            LogInfo(msg='[ANHC] ✅ T=14s Navigation + RViz UP'),
        ]),

        # T=25s — Auto goal
        auto_goal_sender,
    ])
