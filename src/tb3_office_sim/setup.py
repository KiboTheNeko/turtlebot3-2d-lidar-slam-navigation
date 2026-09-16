#!/usr/bin/env python3

import os
from glob import glob

from setuptools import setup

package_name = 'tb3_office_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models'),
         [f for f in glob('models/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'maps'),
         [f for f in glob('maps/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config'),
         [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config', 'nav2'),
         glob('config/nav2/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kibo',
    maintainer_email='kibo@localhost',
    description='TB3 Office SLAM simulation: Gazebo Harmonic world + ROS 2 Jazzy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = tb3_office_sim.odom_to_tf:main',
            'scan_republisher = tb3_office_sim.scan_republisher:main',
            'drive_route = tb3_office_sim.drive_route:main',
            'compare_map = tb3_office_sim.compare_map:main',
            'frontier_explorer = tb3_office_sim.frontier_explorer:main',
        ],
    },
)