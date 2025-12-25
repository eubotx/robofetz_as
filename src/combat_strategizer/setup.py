from setuptools import setup
import os
from glob import glob

package_name = 'combat_strategizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
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
        ],
    },
)
