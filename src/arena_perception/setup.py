from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arena_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Add launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        
        # Add config files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python>=4.11.0.86',
        'numpy>=2.2.3',
        'matplotlib>=3.10.1',
        'scipy>=1.15.2',
        'pyapriltags>=3.3.0.3',
        'bitstring',
        'pyyaml>=6.0',
    ],
    zip_safe=True,
    maintainer='eubotx',
    maintainer_email='eubotx@mailbox.org',
    description='Position camera, detect friendly robot and opponent robot with external arena camera.',
    license='GNU GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_rectification_node = arena_perception.camera_rectification_node:main',
            'arena_calibration_service = arena_perception.arena_calibration_service:main',
            'apriltag_detection_node = arena_perception.apriltag_detection_node:main',
            'opponent_detection_node = arena_perception.opponent_detection:main',
            'filter_transform_node = arena_perception.filter_transform_node:main',
            'static_tag_position_publisher = arena_perception.static_tag_position_publisher:main',
            'odom_drift_correction_node1 = arena_perception.odom_drift_correction_node1:main',
            'odom_drift_correction_node2 = arena_perception.odom_drift_correction_node2:main',
            'robot_detection_node1 = arena_perception.robot_detection_node1:main',
            'robot_detection_node2 = arena_perception.robot_detection_node2:main',

        ],
    },
)