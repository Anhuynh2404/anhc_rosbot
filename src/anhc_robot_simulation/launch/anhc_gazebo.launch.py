import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, RegisterEventHandler, LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, Command,
    PathJoinSubstitution, FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_sim         = get_package_share_directory('anhc_robot_simulation')
    pkg_description = get_package_share_directory('anhc_robot_description')

    # ── Arguments ────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    robot_x_arg  = DeclareLaunchArgument('robot_x',  default_value='-7.0')
    robot_y_arg  = DeclareLaunchArgument('robot_y',  default_value='0.0')
    robot_z_arg  = DeclareLaunchArgument('robot_z',  default_value='0.15')
    robot_yaw_arg = DeclareLaunchArgument('robot_yaw', default_value='0.0')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_x  = LaunchConfiguration('robot_x')
    robot_y  = LaunchConfiguration('robot_y')
    robot_z  = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    # ── Xacro → URDF ─────────────────────────────────────────
    xacro_file = os.path.join(pkg_description, 'urdf', 'main_robot.urdf.xacro')
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file, ' use_sim:=true',
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ── Robot State Publisher ─────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='anhc_robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ── Gazebo ────────────────────────────────────────────────
    world_file = os.path.join(pkg_sim, 'worlds', 'anhc_simple_world.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Clock Bridge ─────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='anhc_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── Spawn Robot ───────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='anhc_robot_spawner',
        arguments=[
            '-name',  'anhc_robot',
            '-topic', '/robot_description',
            '-x', robot_x, '-y', robot_y,
            '-z', robot_z, '-Y', robot_yaw,
        ],
        output='screen',
    )

    # ── Sensor Bridge ─────────────────────────────────────────
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

    # ── TF Bridge ─────────────────────────────────────────────
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

    # ── Joint State Bridge ────────────────────────────────────
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='anhc_joint_state_bridge',
        arguments=[
            '/world/anhc_simple_world/model/anhc_robot/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/anhc_simple_world/model/anhc_robot/joint_state',
             '/joint_states'),
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Controller Spawners ───────────────────────────────────
    # KEY FIX: NO parameters=[{'use_sim_time':...}] on spawner nodes
    # use_sim_time is handled by controller_manager itself via /clock
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='anhc_jsb_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='anhc_diff_drive_spawner',
        arguments=[
            'anhc_diff_drive_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ── Event chain: spawn controllers after robot is up ─────
    spawn_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=3.0, actions=[
                    joint_state_broadcaster_spawner,
                    LogInfo(msg='[ANHC] Spawning joint_state_broadcaster...'),
                ])
            ]
        )
    )

    spawn_diff = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(period=1.0, actions=[
                    diff_drive_spawner,
                    LogInfo(msg='[ANHC] Spawning diff_drive_controller...'),
                ])
            ]
        )
    )

    return LaunchDescription([
        use_sim_time_arg, robot_x_arg, robot_y_arg,
        robot_z_arg, robot_yaw_arg,

        LogInfo(msg='[ANHC] Starting Gazebo...'),
        gazebo,
        robot_state_publisher,

        # Clock bridge first (2s — let Gazebo start)
        TimerAction(period=2.0, actions=[
            clock_bridge,
            LogInfo(msg='[ANHC] Clock bridge UP'),
        ]),

        # All other bridges (3s)
        TimerAction(period=3.0, actions=[
            sensor_bridge,
            tf_bridge,
            joint_state_bridge,
            LogInfo(msg='[ANHC] All bridges UP'),
        ]),

        # Spawn robot (4s)
        TimerAction(period=4.0, actions=[
            spawn_robot,
            LogInfo(msg='[ANHC] Spawning robot...'),
        ]),

        spawn_jsb,
        spawn_diff,
    ])
