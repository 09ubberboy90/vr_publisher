import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    ur10 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_t42"), 'launch', 'ur_t42_control.launch.py'),),
        launch_arguments=[
            ("ur_type", robot_type),
            ("robot_ip", "yyy.yyy.yyy.yyy"),
            ("use_fake_hardware", use_fake_hardware),
            ("launch_rviz", "false"),
            ("initial_joint_controller", "joint_trajectory_controller"),
            ("description_file", "ur_with_gripper.xacro"),
            ("description_package", "ur_t42"),
        ],)

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_t42"), 'launch', 'ur_t42_moveit.launch.py'),),
        launch_arguments=[
            ("ur_type", robot_type),
            ("launch_rviz", "true"),
            ("launch_servo", "false"),
            ("description_file", "ur_with_gripper.xacro"),
            ("description_package", "ur_t42"),
            ("moveit_config_file", "ur_with_gripper.srdf.xacro"),
            ("moveit_config_package", "ur_t42"),

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
            period=2.5,
            actions=[
                moveit,
                # moveit_gripper
            ]
        )
    ])
