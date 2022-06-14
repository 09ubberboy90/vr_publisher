import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


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


def generate_launch_description():
    robot_description_config = load_file("vr_panda", 
        os.path.join(
            "urdf",
            "ur10.urdf",
        )
)
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "vr_panda", "srdf/ur10.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    pose_tracking_yaml = load_yaml("moveit_servo", "config/pose_tracking_settings.yaml")
    pose_tracking_params = {"moveit_servo": pose_tracking_yaml}



    kinematics_yaml = load_yaml(
        "vr_panda", "config/ur10_default_kinematics.yaml"
    )
    # servo_yaml = load_yaml("vr_panda", "config/ur_simulated.yaml")
    servo_yaml = load_yaml("vr_panda", "config/ur_simulated.yaml")
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

    return LaunchDescription(
        [
            pose_tracking_node,
        ]
    )
