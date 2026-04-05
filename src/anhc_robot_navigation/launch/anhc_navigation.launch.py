import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_nav = get_package_share_directory('anhc_robot_navigation')
    config  = os.path.join(pkg_nav, 'config', 'anhc_navigation.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Map Handler ───────────────────────────────────────────
    map_handler = Node(
        package='anhc_robot_navigation',
        executable='anhc_map_handler_node',
        name='anhc_map_handler_node',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    # ── A* Planner ────────────────────────────────────────────
    planner = Node(
        package='anhc_robot_navigation',
        executable='anhc_planner_node',
        name='anhc_planner_node',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    # ── Path Follower (Step 5) ────────────────────────────────
    path_follower = Node(
        package='anhc_robot_navigation',
        executable='anhc_path_follower_node',
        name='anhc_path_follower_node',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        LogInfo(msg='[ANHC] Starting Navigation Stack...'),
        map_handler,
        planner,
        path_follower,
    ])
