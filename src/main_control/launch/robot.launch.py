from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main_control',
            executable='lidar_filter',
            name='lidar_filter_node',
            output='screen'
        ),
        Node(
            package='uart',
            executable='uart_node',
            name='uart_protocol',
            output='screen'
        ),
        Node (
            package='uart',
            executable='data_process',
            name='data_transfer',
            output='screen'
        ),
        Node (
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            output='screen'
        ),
        Node(
            package='main_control',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen'
        ),
        Node(
            package='main_control',
            executable='compensator',
            name='compensator_node',
            output='screen'
        )
    ])