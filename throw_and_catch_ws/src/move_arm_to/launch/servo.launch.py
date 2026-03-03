#!/usr/bin/env python3

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Launch MoveIt2 + Servo server for ball catching.
    """
    
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (set true for Gazebo)",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Package directories
    my_pkg_share = get_package_share_directory("move_arm_to")
    om_desc_share = get_package_share_directory("open_manipulator_description")
    
    # URDF/xacro
    urdf_xacro = os.path.join(om_desc_share, "urdf", "omy_f3m", "omy_f3m.urdf.xacro")

    # MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder(robot_name="omy_f3m", package_name="open_manipulator_moveit_config")
        .robot_description(file_path=urdf_xacro)
        .robot_description_semantic("config/omy_f3m/omy_f3m.srdf")
        .robot_description_kinematics("config/kinematics.yaml")
        .joint_limits("config/omy_f3m/joint_limits.yaml")
        .trajectory_execution("config/omy_f3m/moveit_controllers.yaml")
  
        .moveit_cpp(file_path=os.path.join(my_pkg_share, "config", "my_motion_planning_python_api_tutorial.yaml"))
        .to_moveit_configs()
    )

    params = [
        moveit_config.to_dict(),
        {"use_sim_time": use_sim_time},
    ]

    # Servo parameters (inline instead of from YAML)
    servo_params = {
        "moveit_servo": {
            "move_group_name": "arm",
            "planning_frame": "world", #
            "ee_frame_name": "end_effector_link", #
            "publish_period": 0.02, #
            "use_smoothing": True,
            "smoothing_filter_plugin": "online_signal_smoothing/ButterworthFilterPlugin",
            "command_in_type": "speed_units",
            "robot_link_command_frame": "end_effector_link",
            "command_frame": "world",
            "incoming_command_mode": "topic_commands",
            "pose_tracked_error_threshold": 0.1,
            "linear_speed_limit": 0.5,
            "rotational_speed_limit": 1.0,
            "lower_singularity_threshold": 17.0,
            "upper_singularity_threshold": 18.0,
            "use_collision_avoidance": True,
            "incoming_command_timeout": 2.0,
            "command_out_topic": "/arm_controller/joint_trajectory",
            "scale": {
                "linear": 1.0,
                "rotational": 1.0,
                "joint": 1.0,
            },
        }
    }

    # =========================================================================
    # MoveIt Servo Node
    # =========================================================================
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # =========================================================================
    # Ball Catching Controller Node
    # =========================================================================
    catch_ball_servo_node = Node(
        package="move_arm_to",
        executable="catch_ball_servo",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # =========================================================================
    # Joint State Publisher (if not coming from controllers)
    # =========================================================================
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[moveit_config.joint_limits],
        output="screen",
    )

    # =========================================================================
    # Move Group Node
    # =========================================================================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=params,
    )

    # =========================================================================
    # RViz (optional, for visualization)
    # =========================================================================
    rviz_config_file = os.path.join(
        om_desc_share,
        "rviz",
        "open_manipulator.rviz",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            # move_group_node,
            servo_node,
            catch_ball_servo_node,
            # joint_state_publisher,
            # rviz_node,  # Uncomment for visualization
        ]
    )
