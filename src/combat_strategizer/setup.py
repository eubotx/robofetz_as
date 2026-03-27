from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'combat_strategizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eubotx',
    maintainer_email='eubotx@mailbox.org',
    description='Decides combat strategy and sets goal_pose / path',
    license='GNU GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_attack = combat_strategizer.nav2_attack:main',
            'weapon_control = combat_strategizer.weapon_control:main',
            'point_to_pose_remap = combat_strategizer.point_to_pose_remap:main',
            'tf_to_pose = combat_strategizer.tf_to_pose:main',
            'simple_attack = combat_strategizer.simple_attack:main',
            'elf_combat_strategizer = combat_strategizer.elf_combat_strategizer:main',
        ],
    },
)
