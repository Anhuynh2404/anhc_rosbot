# ═══════════════════════════════════════════════════════════════
# ANHC Simulation Launch
# Combines: Gazebo + Robot Spawn + RViz
# (Navigation nodes are added in the full launch — Step 7)
# ═══════════════════════════════════════════════════════════════

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_sim    = get_package_share_directory('anhc_robot_simulation')
    pkg_bringup = get_package_share_directory('anhc_robot_bringup')

    # ── Arguments ────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    robot_x_arg = DeclareLaunchArgument(
        'robot_x', default_value='-7.0'
    )
    robot_y_arg = DeclareLaunchArgument(
        'robot_y', default_value='0.0'
    )

    # ── Include Gazebo Launch ─────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'anhc_gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_x':      LaunchConfiguration('robot_x'),
            'robot_y':      LaunchConfiguration('robot_y'),
        }.items(),
    )

    # ── Include RViz Launch ───────────────────────────────────
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'anhc_rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_x_arg,
        robot_y_arg,
        LogInfo(msg='╔══════════════════════════════════════╗'),
        LogInfo(msg='║   ANHC Robot Simulation Starting     ║'),
        LogInfo(msg='╚══════════════════════════════════════╝'),
        gazebo_launch,
        rviz_launch,
    ])
