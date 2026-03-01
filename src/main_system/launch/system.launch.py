import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    rsp_launch_path = os.path.join(
        get_package_share_directory('main_system'),
        'launch',
        'rsp.launch.py'
    )

    control_launch_path = os.path.join(
        get_package_share_directory('main_control'),
        'launch',
        'robot.launch.py'
    )

    launch_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path)
    )

    delayed_control = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(control_launch_path)
            )
        ]
    )

    return LaunchDescription([
        launch_rsp,
        delayed_control
    ])