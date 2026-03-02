#!/usr/bin/env python3

"""
Launch file for simple ball catching using MoveIt trajectory execution.
- No servo control, just MoveIt move_group + catch_ball_simple node.
- Subscribes to ball pose and commands arm to move toward it.

how to launch:
ros2 launch move_arm_to catch_ball_simple.launch.py use_sim_time:=true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Simple launch for ball catching using trajectory execution.
    No servo complexity - just MoveIt move_group + catch_ball_simple node.
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
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
            load_all=False,
        )
        .pilz_cartesian_limits("config/pilz_cartesian_limits.yaml")
        .moveit_cpp(file_path=os.path.join(my_pkg_share, "config", "my_motion_planning_python_api_tutorial.yaml"))
        .to_moveit_configs()
    )

    params = [
        moveit_config.to_dict(),
        {"use_sim_time": use_sim_time},
    ]

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
    # Joint State Publisher
    # =========================================================================
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[moveit_config.joint_limits],
        output="screen",
    )

    # =========================================================================
    # Catch Ball Simple Node
    # =========================================================================
    catch_ball_node = Node(
        package="move_arm_to",
        executable="catch_ball_simple",
        output="screen",
        parameters=params,
    )

    return LaunchDescription(
        declared_arguments
        + [
            move_group_node,
            joint_state_publisher,
            catch_ball_node,
        ]
    )
