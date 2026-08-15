"""
Depth stack (single launch): 2D controller + depth_controller + cmd_vel_3d_merge + thruster_allocator_3d.

For DVL + 2D mission + 2D controller only (no depth), use:
  ros2 launch auv_2d_control dvl_mission_control.launch.py

Do NOT run thruster_allocator_2d together with this stack (both publish /auv/thruster_cmd).

Depth arming: see depth_control_default.yaml (auto_arm_on_start).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    depth_params = os.path.join(
        get_package_share_directory("auv_depth_control"),
        "config",
        "depth_control_default.yaml",
    )
    control_2d_params = os.path.join(
        get_package_share_directory("auv_2d_control"),
        "config",
        "control_2d_default.yaml",
    )
    allocator_3d_params = os.path.join(
        get_package_share_directory("auv_3d_control"),
        "config",
        "control_3d_default.yaml",
    )

    depth_params_file = LaunchConfiguration("depth_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "depth_params_file",
                default_value=depth_params,
                description="YAML for depth_controller and cmd_vel_3d_merge",
            ),
            Node(
                package="auv_2d_control",
                executable="controller_2d",
                name="controller_2d",
                output="screen",
                parameters=[control_2d_params],
            ),
            Node(
                package="auv_depth_control",
                executable="depth_controller",
                name="depth_controller",
                output="screen",
                parameters=[depth_params_file],
            ),
            Node(
                package="auv_depth_control",
                executable="cmd_vel_3d_merge",
                name="cmd_vel_3d_merge",
                output="screen",
                parameters=[depth_params_file],
            ),
            Node(
                package="auv_3d_control",
                executable="thruster_allocator_3d",
                name="thruster_allocator_3d",
                output="screen",
                parameters=[allocator_3d_params],
            ),
        ]
    )
