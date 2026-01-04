from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),


        # Config files
        ('share/' + package_name + '/config/joy_teleop', ['config/joy_teleop/rhinozeros_teleop.config.yaml']),
        ('share/' + package_name + '/config/slider_publisher', ['config/slider_publisher/Float32.yaml']),
        

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eubotx',
    maintainer_email='eubotx@mailbox.org',
    description='Provides utility functionality like config files, launch files and helper nodes without a home',
    license='GNU GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
