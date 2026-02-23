"""
Launch file for ball detection and pose estimation.

This launch file:
1. Launches the ball_detector node
2. Launches the appropriate pose estimation node based on the 'pose_estimation_method' parameter:
   - 'pnp': Uses ball_pose_estimation (PnP-based)
   - 'depth': Uses ball_pose_estimation_depth (Depth-based)

Usage:
  ros2 launch vision ball_pose.launch.py pose_estimation_method:=pnp
  ros2 launch vision ball_pose.launch.py pose_estimation_method:=depth
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pose_estimation_method_arg = DeclareLaunchArgument(
        'pose_estimation_method',
        default_value='pnp',  # remove this if you want it required
        description='Method for pose estimation: "pnp" or "depth"'
    )

    method = LaunchConfiguration('pose_estimation_method')

    ball_detector = Node(
        package='vision',
        executable='ball_detector',
        name='ball_detector',
        output='screen',
    )

    ball_pose_estimation_pnp = Node(
        package='vision',
        executable='ball_pose_estimation',
        name='ball_pose_estimation',
        output='screen',
        condition=IfCondition(PythonExpression(["'", method, "' == 'pnp'"])),
    )

    ball_pose_estimation_depth = Node(
        package='vision',
        executable='ball_pose_estimation_depth',
        name='ball_pose_estimation_depth',
        output='screen',
        condition=IfCondition(PythonExpression(["'", method, "' == 'depth'"])),
    )

    return LaunchDescription([
        pose_estimation_method_arg,
        ball_detector,
        ball_pose_estimation_pnp,
        ball_pose_estimation_depth,
    ])