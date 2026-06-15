"""
Launch AprilTag-based camera-to-robot calibration tools.

This launch file starts the following:
- launch: apriltag_ros/camera_36h11.launch.py
- Node: vision/visualize_tag_detections
- Node: vision/cal_cam_robot

Usage:
    ros2 launch vision cal_cam_robot.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch.actions import IncludeLaunchDescription


from launch_ros.actions import Node

def generate_launch_description():

    apriltag = IncludeLaunchDescription(  
        PythonLaunchDescriptionSource(  
            os.path.join(  
                get_package_share_directory("apriltag_ros"),  
                "launch", "camera_36h11.launch.py"  
            )  
        )  
    )



    visualize_tag_detections = Node(
        package='vision',
        executable='visualize_tag_detections',
        name='visualize_tag_detections',
        output='screen',
    )

    cal_cam_robot = Node(
        package='vision',
        executable='cal_cam_robot',
        name='cal_cam_robot',
        output='screen',
    )


    return LaunchDescription([
        apriltag,
        visualize_tag_detections,
        cal_cam_robot,
    ])