import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_yaml = os.path.join(
        get_package_share_directory("ball_dropping_detection"),
        "config",
        "ball_dropping_detection.yaml",
    )
    params = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_yaml,
                description="YAML parameters for ball dropping detection stack",
            ),
            Node(
                package="ball_dropping_detection",
                executable="front_drum_detector",
                name="front_drum_detector",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="ball_dropping_detection",
                executable="bottom_drum_detector",
                name="bottom_drum_detector",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="ball_dropping_detection",
                executable="drum_detection_fusion",
                name="drum_detection_fusion",
                output="screen",
                parameters=[params],
            ),
        ]
    )

