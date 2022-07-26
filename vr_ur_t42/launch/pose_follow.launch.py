import os
from shutil import move
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import OpaqueFunction
from launch import LaunchDescription, LaunchDescriptionSource

try:
    sys.path.append(get_package_share_directory("ur_t42_utils"))
    from ur_t42_utils import generate_descriptions
except Exception:
    print("Failed to generate description. Please make sure you have ur_t42_utils compiled and sourced")


def launch_setup(context):
    pkg_name = "vr_ur_t42"
    pkg_share = get_package_share_directory(pkg_name)

    vr_publish = Node(package='vr_publisher', 
                executable='vr_publish', 
                output='screen')

    visual = Node(package='vr_controller', 
                executable='rviz_vr_visual', 
                output='screen')

    servo_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'pose_tracking.launch.py'),
        ),)

    return [
        vr_publish,
        visual,
        # TimerAction(
        #     period=5.,
        #     actions=[
        #         servo_controller
        #     ]
        # )
    ]

def generate_launch_description():
    declared_arguments_ur = [OpaqueFunction(
    function=generate_descriptions.generate_launch_arguments_ur)]

    declared_arguments_general = [OpaqueFunction(
        function=generate_descriptions.generate_launch_arguments_ur_t42)]
    return LaunchDescription(declared_arguments_ur + declared_arguments_general + [OpaqueFunction(function=launch_setup)])
