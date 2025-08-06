"""Launch pylon_detection node together with RealSense node."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for pylon_detection and RealSense nodes."""
    return LaunchDescription([
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            output="screen",
            prefix="sh -c 'exec \"$0\" \"$@\" > /dev/null 2>&1'",
            parameters=[{
                "depth_module.depth_profile": "640x480x15",
                "rgb_camera.color_profile": "640x480x15",
                "align_depth.enable": True,
                "enable_sync": True,
            }],
        ),

        Node(
            package="landmark_based_lane_detection_pkg",
            executable="pylon_detection_node",
            name="pylon_detection_node",
            output="screen",
            parameters=[],
        ),
    ])
