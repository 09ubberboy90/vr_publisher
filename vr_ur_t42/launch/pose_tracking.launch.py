import sys, os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch.actions import OpaqueFunction
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

try:
    sys.path.append(get_package_share_directory("ur_t42_utils"))
    from ur_t42_utils import generate_descriptions
except Exception:
    print("Failed to generate description. Please make sure you have ur_t42_utils compiled and sourced")


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context):
    robot_description_content = generate_descriptions.generate_urdf_moveit(locals())
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic_content =generate_descriptions.generate_srdf(locals())
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    pose_tracking_yaml = load_yaml("moveit_servo", "config/pose_tracking_settings.yaml")
    pose_tracking_params = {"moveit_servo": pose_tracking_yaml}



    kinematics_yaml = load_yaml(
        "ur_t42", "config/ur10/default_kinematics.yaml"
    )
    # servo_yaml = load_yaml("vr_panda", "config/ur_simulated.yaml")
    servo_yaml = load_yaml("vr_ur_t42", "config/ur_simulated.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    pose_tracking_node = Node(
        package="vr_controller",
        executable="pose_tracking",
        # prefix=['xterm -e gdb -ex run --args'],
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            pose_tracking_params,
            servo_params,
        ],
    )

    # ros2_control using FakeSystem as hardware

    return [pose_tracking_node,]


def generate_launch_description():
    declared_arguments_general = OpaqueFunction(
        function=generate_descriptions.generate_launch_arguments_ur_t42)
    declared_arguments = [OpaqueFunction(
        function=generate_descriptions.generate_launch_arguments_moveit)]
    desc = IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])))
    return LaunchDescription([declared_arguments_general,desc])
