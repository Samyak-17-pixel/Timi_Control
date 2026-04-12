"""Launch mother node with config paths."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("timi_auv_control")
    ctrl = os.path.join(pkg, "config", "control_params.yaml")
    geo = os.path.join(pkg, "config", "geometry.yaml")

    mission_type = LaunchConfiguration("mission_type")
    mission_file = LaunchConfiguration("mission_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_type",
                default_value="station_keeping",
                description="station_keeping | waypoint | path_following",
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value=os.path.join(
                    pkg, "config", "missions", "station_keeping", "mission.yaml"
                ),
                description="Path to mission-specific YAML (e.g. .../station_keeping/mission.yaml)",
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument("pwm_topic", default_value="/auv/thrusters/pwm"),
            DeclareLaunchArgument("control_rate_hz", default_value="50.0"),
            Node(
                package="timi_auv_control",
                executable="mother_node",
                name="timi_auv_mother",
                output="screen",
                parameters=[
                    {"control_config": ctrl},
                    {"geometry_config": geo},
                    {"mission_config": mission_file},
                    {"mission_type": mission_type},
                    {"odom_topic": LaunchConfiguration("odom_topic")},
                    {"pwm_topic": LaunchConfiguration("pwm_topic")},
                    {"control_rate_hz": LaunchConfiguration("control_rate_hz")},
                ],
            ),
        ]
    )
