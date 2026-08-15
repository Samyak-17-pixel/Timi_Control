"""Launch gate detector (vision) + gate mission (control) together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    det_yaml = os.path.join(
        get_package_share_directory("qualification_gate_detection"),
        "config",
        "gate_detector_params.yaml",
    )
    mission_yaml = os.path.join(
        get_package_share_directory("qualification_gate_control"),
        "config",
        "qualification_mission.yaml",
    )

    det_params = LaunchConfiguration("detection_params_file")
    mission_params = LaunchConfiguration("mission_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detection_params_file",
                default_value=det_yaml,
                description="YAML for qualification_gate_detection",
            ),
            DeclareLaunchArgument(
                "mission_params_file",
                default_value=mission_yaml,
                description="YAML for gate_mission",
            ),
            Node(
                package="qualification_gate_detection",
                executable="gate_detector_node",
                name="gate_detector",
                output="screen",
                parameters=[det_params],
            ),
            Node(
                package="qualification_gate_control",
                executable="gate_mission",
                name="gate_mission",
                output="screen",
                parameters=[mission_params],
            ),
        ]
    )
