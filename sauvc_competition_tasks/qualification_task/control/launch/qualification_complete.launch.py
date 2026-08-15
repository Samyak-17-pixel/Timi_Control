"""
Full qualification stack: vision + gate mission + depth/heave + 3D thruster output.

Does not start controller_2d (gate_mission owns /control/cmd_vel).

Example:
  ros2 launch qualification_gate_control qualification_complete.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    depth_yaml = os.path.join(
        get_package_share_directory("auv_depth_control"),
        "config",
        "depth_control_default.yaml",
    )
    allocator_3d_yaml = os.path.join(
        get_package_share_directory("auv_3d_control"),
        "config",
        "control_3d_default.yaml",
    )
    dvl_yaml = os.path.join(
        get_package_share_directory("dvl_to_odom_bridge"),
        "config",
        "dvl_to_odom.yaml",
    )

    det_params = LaunchConfiguration("detection_params_file")
    mission_params = LaunchConfiguration("mission_params_file")
    depth_params = LaunchConfiguration("depth_params_file")
    allocator_3d_params = LaunchConfiguration("allocator_3d_params_file")
    dvl_params = LaunchConfiguration("dvl_params_file")
    launch_dvl_bridge = LaunchConfiguration("launch_dvl_bridge")

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
            DeclareLaunchArgument(
                "depth_params_file",
                default_value=depth_yaml,
                description="YAML for depth_controller and cmd_vel_3d_merge",
            ),
            DeclareLaunchArgument(
                "allocator_3d_params_file",
                default_value=allocator_3d_yaml,
                description="YAML for thruster_allocator_3d",
            ),
            DeclareLaunchArgument(
                "dvl_params_file",
                default_value=dvl_yaml,
                description="YAML for dvl_to_odom_bridge when launch_dvl_bridge is true",
            ),
            DeclareLaunchArgument(
                "launch_dvl_bridge",
                default_value="true",
                description="If true, start dvl_to_odom_bridge (expects /dvl/position from DVL driver).",
            ),
            Node(
                package="qualification_gate_detection",
                executable="gate_detector_node",
                name="gate_detector",
                output="screen",
                parameters=[det_params],
            ),
            Node(
                package="auv_depth_control",
                executable="depth_controller",
                name="depth_controller",
                output="screen",
                parameters=[depth_params],
            ),
            Node(
                package="auv_depth_control",
                executable="cmd_vel_3d_merge",
                name="cmd_vel_3d_merge",
                output="screen",
                parameters=[depth_params],
            ),
            Node(
                package="auv_3d_control",
                executable="thruster_allocator_3d",
                name="thruster_allocator_3d",
                output="screen",
                parameters=[allocator_3d_params],
            ),
            Node(
                package="qualification_gate_control",
                executable="gate_mission",
                name="gate_mission",
                output="screen",
                parameters=[mission_params],
            ),
            Node(
                package="dvl_to_odom_bridge",
                executable="dvl_to_odom_node",
                name="dvl_to_odom_bridge",
                output="screen",
                parameters=[dvl_params],
                condition=IfCondition(launch_dvl_bridge),
            ),
        ]
    )
