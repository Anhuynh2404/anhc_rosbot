"""
Diagnostic launch — prints topic graph summary.
Run this while simulation is active to verify all connections.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'topic', 'list'],
            output='screen',
            name='topic_list',
        ),

        ExecuteProcess(
            cmd=['ros2', 'node', 'list'],
            output='screen',
            name='node_list',
        ),
    ])
