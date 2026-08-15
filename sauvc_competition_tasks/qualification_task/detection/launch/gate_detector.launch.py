import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_yaml = os.path.join(
        get_package_share_directory("qualification_gate_detection"),
        "config",
        "gate_detector_params.yaml",
    )
    params = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_yaml,
                description="YAML parameters for gate_detector",
            ),
            Node(
                package="qualification_gate_detection",
                executable="gate_detector_node",
                name="gate_detector",
                output="screen",
                parameters=[params],
            ),
        ]
    )
