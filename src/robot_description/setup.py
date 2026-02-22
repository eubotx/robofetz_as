from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_description'

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
        # Install models robot descriptions
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf/gazebo'), glob('urdf/gazebo/*')),
        (os.path.join('share', package_name, 'urdf/sensors'), glob('urdf/sensors/*')),
        (os.path.join('share', package_name, 'textures'), glob('textures/*')),


        # install other Parameter and Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,

    # must match package.xml
    maintainer='eubotx',
    maintainer_email='eubotx@mailbox.org',
    description='Provide robot description and robot_state_publisher.',
    license='GNU GPL',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
        ],
    },
)
