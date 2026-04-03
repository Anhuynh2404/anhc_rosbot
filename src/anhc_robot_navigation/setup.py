from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'anhc_robot_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Install map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anhc',
    maintainer_email='dev@anhc.com',
    description='A* path planning and navigation for ANHC robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Nodes will be registered here in Step 4
            'anhc_map_handler_node    = anhc_robot_navigation.anhc_map_handler:main',
            'anhc_planner_node        = anhc_robot_navigation.anhc_planner:main',
            'anhc_path_follower_node  = anhc_robot_navigation.anhc_path_follower:main',
        ],
    },
)
