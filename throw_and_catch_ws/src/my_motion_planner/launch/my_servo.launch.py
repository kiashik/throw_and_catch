
'''
https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/launch/demo_ros_api.launch.py
https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html 

# Test mock hardware
ros2 launch my_motion_planner my_servo.launch.py use_mock_hardware:=true

# Test real hardware (when ready)
ros2 launch my_motion_planner my_servo.launch.py

currently does not set the command type for moveit_servo. 
for now set with this cli command (1 for twist):
    ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
    see: https://github.com/moveit/moveit_msgs/blob/ros2/srv/ServoCommandType.srv 



NOT FULLY FUNCTIONAL.

'''

import os
from pathlib import Path

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware for simulation',
        ),
    ]

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    om_desc_share = get_package_share_directory('open_manipulator_description')
    om_moveit_config_share = get_package_share_directory('open_manipulator_moveit_config')
    
    # Generate URDF with xacro to support use_mock_hardware parameter TODO: i don't understand this well. For real hardware, may need to modify this.
    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            om_desc_share,
            'urdf',
            'omy_f3m',
            'omy_f3m.urdf.xacro',
        ]),
        ' ',
        'use_mock_hardware:=',
        use_mock_hardware,
        ' ',
        'ros2_control_type:=omy_f3m_position',
    ])

    moveit_config = (
        MoveItConfigsBuilder(robot_name='omy_f3m', package_name='open_manipulator_moveit_config')
        .robot_description_semantic(file_path=os.path.join(om_moveit_config_share, 'config', 'omy_f3m', 'omy_f3m.srdf'))
        .joint_limits(file_path=os.path.join(om_moveit_config_share, 'config', 'omy_f3m', 'joint_limits.yaml')) 
        .to_moveit_configs()                               
    )
    
    # Add the generated URDF to the moveit_config dict
    moveit_config.robot_description = {'robot_description': urdf_file}

    # Launch Servo as a standalone node or as a "node component" for better latency/efficiency
    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="false"
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("my_motion_planner")
        .yaml("config/my_servo_params.yaml")
        .to_dict()
    }

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "arm"}    # this name "arm" came from srdf file.

    # RViz
    rviz_config_file = (
        os.path.join(om_desc_share, 'rviz', 'open_manipulator.rviz')
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )


    ros2_controllers_path = os.path.join(
        get_package_share_directory("open_manipulator_bringup"),
        "config",
        "omy_f3m",
        "hardware_controller_manager.yaml",
    )      
    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': urdf_file}, ros2_controllers_path],
        output="screen",
    )

    robot_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "gripper_controller",
            "joint_state_broadcaster",
        ],
        output="screen",
        parameters=[{'robot_description': urdf_file}],
    )

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="my_moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    planning_group_name,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/link0", "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    # Delay controller spawning after ros2_control_node starts
    delay_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[robot_controller_spawner],
                )
            ],
        )
    )

    return launch.LaunchDescription(
        declared_arguments
        + [
            rviz_node,
            ros2_control_node,
            delay_controller_spawner,
            servo_node,
            container,
        ]
    )