# ═══════════════════════════════════════════════════════════════
# ANHC RViz Launch File
# Launches RViz2 with pre-configured display settings
# ═══════════════════════════════════════════════════════════════

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_description = get_package_share_directory('anhc_robot_description')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            pkg_description, 'rviz', 'anhc_robot.rviz'
        ),
        description='Path to RViz config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='anhc_rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
    )

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        rviz_node,
    ])
