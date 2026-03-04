"""
see docstring in track_ball.py
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Whether to use simulation time',
        ),
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored',
        ),
        DeclareLaunchArgument(
            'publish_robot_description_semantic',
            default_value='true',
            description='Whether to publish robot description semantic',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')

    # Get URDF path
    om_desc_share = get_package_share_directory('open_manipulator_description')
    urdf_xacro = os.path.join(om_desc_share, 'urdf', 'omy_f3m', 'omy_f3m.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='omy_f3m', package_name='open_manipulator_moveit_config')
        .robot_description(file_path=urdf_xacro)
        .robot_description_semantic(str(Path('config') / 'omy_f3m' / 'omy_f3m.srdf'))
        .robot_description_kinematics(str(Path('config') / 'kinematics.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .joint_limits(str(Path('config') / 'omy_f3m' / 'joint_limits.yaml'))
        .trajectory_execution(str(Path('config') / 'omy_f3m' / 'moveit_controllers.yaml'))
        .to_moveit_configs()
    )


    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('open_manipulator_moveit_config'), 'config', 'moveit.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(start_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                'use_sim_time': use_sim,
            },
        ],
    )

    track_ball_node = Node(
        package="my_motion_planner",
        executable="track_ball",
        parameters=[{"use_sim_time": use_sim}],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            move_group_node,
            rviz_node,
            track_ball_node,
        ]
    )