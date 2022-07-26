import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import LogInfo
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction

try:
    sys.path.append(get_package_share_directory("ur_t42_utils"))
    from ur_t42_utils import generate_descriptions
except Exception:
    print("Failed to generate description. Please make sure you have ur_t42_utils compiled and sourced")

def launch_setup(context):

    robot_description_content = generate_descriptions.generate_urdf_moveit(locals())
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic_content =generate_descriptions.generate_srdf(locals())
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

    return [moveit_controller]

def generate_launch_description():
    declared_arguments_general = OpaqueFunction(
        function=generate_descriptions.generate_launch_arguments_ur_t42)
    declared_arguments = [OpaqueFunction(
        function=generate_descriptions.generate_launch_arguments_moveit)]
    desc = IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])))
    return LaunchDescription([declared_arguments_general,desc])
