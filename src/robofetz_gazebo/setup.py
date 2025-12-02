from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robofetz_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # install ressource folder
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # install package.xml file
        ('share/' + package_name, ['package.xml']),
        # install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # install Gazebo world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Install models robot descriptions
        (os.path.join('share', package_name, 'models/simple_diff_drive_robot'), glob('models/simple_diff_drive_robot/*')),
        (os.path.join('share', package_name, 'models/simple_diff_drive_opponent'), glob('models/simple_diff_drive_opponent/*')),
        # install other Parameter file (e.g., models)
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    # must match package.xml
    maintainer='eubotx',
    maintainer_email='eubotx@mailbox.org',
    description='Robot simulation for differential drive robot with spinning weapon in an arena with top down camera and ground truth position publishing.',
    license='GNU GPL',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'tf_to_pose = robofetz_gazebo.tf_to_pose:main',
            'grayscale_republisher = robofetz_gazebo.grayscale_republisher_node:main',
            'camera_info_publisher = robofetz_gazebo.camera_info_publisher:main',
        ],
    },
)
