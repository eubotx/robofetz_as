from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dario',
    maintainer_email='dario@todo.todo',
    description='Navigation configuration for RoboFetz',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_navigator = robot_navigation.simple_navigator:main',
            'create_map = robot_navigation.create_map:main',
            'map_node = robot_navigation.map_node:main',
            'cmd_vel_relay = robot_navigation.cmd_vel_relay:main',
        ],
    },
)