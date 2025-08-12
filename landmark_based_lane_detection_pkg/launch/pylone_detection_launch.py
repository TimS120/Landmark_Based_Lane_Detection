"""Launch pylon_detection node together with RealSense node."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for pylon_detection and RealSense nodes."""
    realsense_camera = Node(
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
        )

    pylon_detection = Node(
            package="landmark_based_lane_detection_pkg",
            executable="pylon_detection_node",
            name="pylon_detection_node",
            output="screen",
            parameters=[],
        )

    localization = Node(
            package="robus_localization_pkg",
            executable="robus_localization_node",
            name="robus_localization_node",
            output="screen",
            parameters=[],
        )

    car_description_path = os.path.join(
        get_package_share_directory("car_description"),
        "launch",
        "publish_model.launch.py"
    )
    car_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(car_description_path),
            launch_arguments={"car_id": "7"}.items()
        )
    
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_7_camera_link_to_camera_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "1", "7/camera_link", "camera_link"],
    )

    launch_description = LaunchDescription()

    #launch_description.add_action(realsense_camera)  # Makes somehow problems, when the launch file is stopped and started again --> Start manually:
        # ros2 launch realsense2_camera rs_launch.py depath_module.depth_profile:=640x480x15 rgb_camear.color_profile:=640x480x15 align_depth.enable:=true enable_sync:=true
    launch_description.add_action(pylon_detection)
    launch_description.add_action(localization)
    launch_description.add_action(car_description)
    launch_description.add_action(static_tf)

    return launch_description
