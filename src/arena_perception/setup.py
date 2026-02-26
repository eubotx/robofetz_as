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
            'find_camera_in_world_service = arena_perception.find_camera_in_world_service:main',
            'apriltag_detection_node = arena_perception.apriltag_detection_node:main',
            'robot_detection_node = arena_perception.robot_detection_node:main',
            'opponent_detection_node = arena_perception.opponent_detection:main',
            'filter_transform_node = arena_perception.filter_transform_node:main',
            'odom_drift_correction_node = arena_perception.odom_drift_correction_node:main',
            'tf_frame_relay = arena_perception.tf_frame_relay:main',
            'opponent_det_MOG2single = arena_perception.opponent_det_MOG2single:main',
            'opponent_det_MOG2multiple = arena_perception.opponent_det_MOG2multiple:main',
            'opponent_det_ColorSingle = arena_perception.opponent_det_ColorSingle:main',
            'opponent_det_ContourSingle = arena_perception.opponent_det_ContourSingle:main',
            'multi_sensor_kalman_filter = arena_perception.multi_sensor_kalman_filter:main',
            'detection_transformation_2D_3D = arena_perception.detection_transformation_2D_3D:main',
            'opponent_tracker_node = arena_perception.opponent_tracker_node:main',
            'detection_2d_to_3d_node = arena_perception.detection_2d_to_3d_node:main',
            'frame_diff_detector = arena_perception.frame_diff_detector:main',
        ],
    },
)