import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    t42 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("t42_gripper_bringup"), 'launch', 't42_gripper.launch.py'),),
        launch_arguments=[
            ("use_fake_hardware", use_fake_hardware),
        ],)

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("t42_gripper_moveit_config"), 'launch', 't42_moveit.launch.py'),),
        launch_arguments=[
            ("launch_rviz", "true"),

        ],)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value="true",
            description='Use fake hardware'),

        t42,
        TimerAction(
            period=2.5,
            actions=[
                moveit,
            ]
        )
    ])
