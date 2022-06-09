import os
from shutil import move
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    ur10 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_bringup"), 'launch', 'ur_control.launch.py'),),
            launch_arguments=[
                ("ur_type", robot_type),
                ("robot_ip", "yyy.yyy.yyy.yyy"),
                ("use_fake_hardware", use_fake_hardware),
                ("launch_rviz", "false"),
                ("initial_joint_controller", "joint_trajectory_controller"),
            ],)

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_moveit_config"), 'launch', 'ur_moveit.launch.py'),),            
        launch_arguments=[
                ("ur_type", robot_type),
                ("launch_rviz", "true"),
                ("launch_servo", "false"),
            ],)

    return LaunchDescription([
        DeclareLaunchArgument(
        'robot_type',
        default_value="ur10",
        description='What UR robot to launch'),
        DeclareLaunchArgument(
        'use_fake_hardware',
        default_value="true",
        description='Use fake hardware'),

        ur10,
        TimerAction(
            period=5.,
            actions=[
                moveit,
            ]
        )
    ])
