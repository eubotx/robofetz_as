from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'detection'

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
    ],
    zip_safe=True,
    maintainer='eubotx',
    maintainer_email='eubotx@mailbox.org',
    description='Detect robot and enemy robot',
    license='GNU GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_lean = detection.main_lean:main',
        ],
    },
)