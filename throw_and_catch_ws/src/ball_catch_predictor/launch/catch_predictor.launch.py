"""Launch catch predictor nodes and RViz for ROS 2 Jazzy.

Usage:
	ros2 launch ball_catch_predictor catch_predictor.launch.py
	ros2 launch ball_catch_predictor catch_predictor.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
	package_share = get_package_share_directory("ball_catch_predictor")
	default_rviz_config = os.path.join(package_share, "config", "catch_predictor.rviz")
	rviz_default_value = default_rviz_config if os.path.exists(default_rviz_config) else ""

	declared_arguments = [
		DeclareLaunchArgument(
			"use_sim_time",
			default_value="false",
			description="Use simulation clock if true.",
		),
		DeclareLaunchArgument(
			"rviz_config",
			default_value=rviz_default_value,
			description="Optional path to an RViz config file.",
		),
	]

	use_sim_time = LaunchConfiguration("use_sim_time")
	rviz_config = LaunchConfiguration("rviz_config")

	catch_predictor_node = Node(
		package="ball_catch_predictor",
		executable="catch_predictor",
		name="catch_predictor",
		output="screen",
		parameters=[{"use_sim_time": use_sim_time}],
	)

	catch_predictor_v3_node = Node(
		package="ball_catch_predictor",
		executable="catch_predictor_v3",
		name="catch_predictor_v3",
		output="screen",
		parameters=[{"use_sim_time": use_sim_time}],
	)

	rviz_without_config = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="screen",
		parameters=[{"use_sim_time": use_sim_time}],
		condition=IfCondition(
			PythonExpression(["'", use_sim_time, "' == 'true' and '", rviz_config, "' == ''"])
		),
	)

	rviz_with_config = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="screen",
		arguments=["-d", rviz_config],
		parameters=[{"use_sim_time": use_sim_time}],
		condition=IfCondition(
			PythonExpression(["'", use_sim_time, "' == 'true' and '", rviz_config, "' != ''"])
		),
	)

	actions = declared_arguments + [
		# catch_predictor_node,
		catch_predictor_v3_node,
		# rviz_with_config,
		# rviz_without_config,
	]

	return LaunchDescription(actions)
