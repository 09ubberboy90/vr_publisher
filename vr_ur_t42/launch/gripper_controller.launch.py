import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch.conditions import IfCondition

def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("t42_gripper_description"), "urdf", "t42_hand.xacro"]),
            " ",
            "name:=",
            "t42_gripper",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("t42_gripper_moveit_config"), "srdf", "t42.srdf.xacro"]
            ),
            " ",
            "name:=",
            "t42_gripper",
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}


    # MoveGroupInterface demo executable
    moveit_controller = Node(name='gripper_controller',
                               package='vr_controller',
                               executable='gripper_controller',
                               output='screen',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           ],
                            #   prefix=['gdbserver localhost:3000']
                            )
    
    return LaunchDescription([moveit_controller])
