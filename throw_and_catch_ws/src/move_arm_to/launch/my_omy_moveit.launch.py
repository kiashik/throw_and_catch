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
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time (set true for Gazebo)",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Your package share (for your MoveItCpp yaml)
    my_pkg_share = get_package_share_directory("move_arm_to")

    # URDF/xacro from the description package
    om_desc_share = get_package_share_directory("open_manipulator_description")
    urdf_xacro = os.path.join(om_desc_share, "urdf", "omy_f3m", "omy_f3m.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="omy_f3m", package_name="open_manipulator_moveit_config")
        # Provide URDF explicitly
        .robot_description(file_path=urdf_xacro)
        # SRDF + kinematics + joint limits + controllers
        .robot_description_semantic(Path("config") / "omy_f3m" / "omy_f3m.srdf")
        .robot_description_kinematics(Path("config") / "kinematics.yaml")
        .joint_limits(Path("config") / "omy_f3m" / "joint_limits.yaml")
        .trajectory_execution(Path("config") / "omy_f3m" / "moveit_controllers.yaml")
        # Load OMPL/CHOMP/Pilz pipeline params (named pipelines)
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
            load_all=False,
        )
        .pilz_cartesian_limits(Path("config") / "pilz_cartesian_limits.yaml")
        # Your MoveItCpp yaml
        .moveit_cpp(file_path=os.path.join(my_pkg_share, "config", "my_motion_planning_python_api_tutorial.yaml"))
        .to_moveit_configs()
    )

    # If you are using Gazebo/ros_gz_sim, set use_sim_time to True
    # Otherwise set to False for real hardware
    params = [
        moveit_config.to_dict(),
        {"use_sim_time": use_sim_time},
    ]

    my_moveit_node = Node(
        package="move_arm_to",
        executable="my_omy_moveit",
        name="my_omy_moveit",
        output="screen",
        parameters=params,
    )

    return LaunchDescription(
        declared_arguments
        + [
            my_moveit_node,
        ]
    )
