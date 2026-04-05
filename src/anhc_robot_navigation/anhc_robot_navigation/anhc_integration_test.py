"""
Quick integration test — verifies all topics are alive.
Run while simulation + navigation stack is running.

Usage:
  python3 anhc_integration_test.py
"""
import subprocess, sys, time


REQUIRED_TOPICS = [
    ('/anhc/scan',           'sensor_msgs/msg/LaserScan'),
    ('/anhc/odom',           'nav_msgs/msg/Odometry'),
    ('/anhc/lidar_3d/points','sensor_msgs/msg/PointCloud2'),
    ('/anhc/map',            'nav_msgs/msg/OccupancyGrid'),
    ('/anhc/planned_path',   'nav_msgs/msg/Path'),
    ('/anhc/cmd_vel',        'geometry_msgs/msg/Twist'),
    ('/tf',                  'tf2_msgs/msg/TFMessage'),
]

REQUIRED_NODES = [
    'anhc_robot_state_publisher',
    'anhc_map_handler_node',
    'anhc_planner_node',
    'anhc_path_follower_node',
]


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    return r.stdout


def check():
    print('\n═══ ANHC Integration Check ═══\n')
    topics = run(['ros2', 'topic', 'list'])
    nodes  = run(['ros2', 'node',  'list'])

    all_ok = True
    print('Topics:')
    for topic, _ in REQUIRED_TOPICS:
        ok = topic in topics
        all_ok &= ok
        print(f'  {"✅" if ok else "❌"} {topic}')

    print('\nNodes:')
    for node in REQUIRED_NODES:
        ok = node in nodes
        all_ok &= ok
        print(f'  {"✅" if ok else "❌"} {node}')

    print(f'\nResult: {"✅ ALL OK" if all_ok else "❌ ISSUES FOUND"}')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(check())
