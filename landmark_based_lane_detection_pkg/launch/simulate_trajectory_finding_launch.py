"""Launch pylon_detection node together with RealSense node."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for pylon_detection and RealSense nodes."""
    simulate_map_creation = Node(
            package="landmark_based_lane_detection_pkg",
            executable="simulate_map_creation_node",
            name="simulate_map_creation_node",
            output="screen",
            parameters=[],
        )

    simulate_driving = Node(
            package="landmark_based_lane_detection_pkg",
            executable="simulate_driving_node",
            name="simulate_driving_node",
            output="screen",
            parameters=[],
        )
    
    trajectory_creation = Node(
            package="landmark_based_lane_detection_pkg",
            executable="trajectory_creation_node",
            name="trajectory_creation_node",
            output="screen",
            parameters=[],
        )

    launch_description = LaunchDescription()

    launch_description.add_action(simulate_map_creation)
    launch_description.add_action(simulate_driving)
    launch_description.add_action(trajectory_creation)

    return launch_description
