"""
Publishes static transform: map → odom
In a full SLAM system, this would be replaced by a SLAM node.
For our A* demo with a known map, map = odom (identity transform).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='anhc_static_tf_map_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_tf_map_odom,
    ])
