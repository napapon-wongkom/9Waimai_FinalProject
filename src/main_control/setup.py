from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'main_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch','*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nawako05',
    maintainer_email='nawako05@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_filter = main_control.lidar_filter:main',
            'hardware_bridge = main_control.hardware_bridge:main',
            'floor_manager = main_control.multifloor:main',
            'compensator = main_control.deadband_compensator:main',
            'sequence = main_control.Sequence_Move:main',
            'setpose = main_control.reset_position:main',
            'waypoint_manage = main_control.waypoint_manager:main',
            'state_machine = main_control.state_machine:main',
            'command_input = main_control.command:main',
        ],
    },
)
