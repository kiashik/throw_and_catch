"""
ASHIK: adapted from https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/launch/demo_ros_api.launch.py

--- usage
    ros2 launch my_motion_planner2 my_servo2.launch.py use_sim_time:=true 
"""

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder, Path
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart



def generate_launch_description():

    # Declare launch arguments
    declared_arguments = [     
        DeclareLaunchArgument(      # TODO: this does not work rn
            'command_type',
            default_value='1',
            description='Servo command type: 0=joint, 1=twist, 2=pose',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation mode/time (true: Gazebo bringup, false: real bringup)',
        ),
    ]



    bringup_gazebo = IncludeLaunchDescription(  
        PythonLaunchDescriptionSource(  
            os.path.join(  
                get_package_share_directory("open_manipulator_bringup"),  
                "launch", "omy_f3m_gazebo.launch.py"  
            )  
        ),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
    )
    bringup_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("open_manipulator_moveit_config"),
                "launch", "omy_f3m_moveit.launch.py"
            )
        ),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
    )  

    # Get the absolute path to the URDF file from the description package
    urdf_path = os.path.join(
        get_package_share_directory("open_manipulator_description"),
        "urdf",
        "omy_f3m",
        "omy_f3m.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("omy_f3m", package_name="open_manipulator_moveit_config")  
        .robot_description(file_path=urdf_path)   
        .robot_description_semantic(file_path="config/omy_f3m/omy_f3m.srdf")  
        .joint_limits(file_path="config/omy_f3m/joint_limits.yaml")  
        .robot_description_kinematics(file_path="config/kinematics.yaml")  
        .trajectory_execution(file_path="config/omy_f3m/moveit_controllers.yaml") 
        .planning_pipelines("ompl")
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()  
    )

    # Launch Servo as a standalone node or as a "node component" for better latency/efficiency
    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="true"
    )
    
    command_type = LaunchConfiguration("command_type")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("my_motion_planner2")
        .yaml("config/my_servo_params2.yaml")
        .to_dict()
    }

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.005}
    planning_group_name = {"planning_group_name": "arm"}    # this name "arm" came from srdf file.

    # TODO: ignorning rviz for now. may need to provide math to rviz config file
    # rviz_node = launch_ros.actions.Node(  
    # package="rviz2",  
    # executable="rviz2",  
    # name="rviz2",  
    # output="log",  
    # parameters=[  
    #     moveit_config.robot_description,  
    #     moveit_config.robot_description_semantic,  
    # ],  
    # )

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
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
                    moveit_config.trajectory_execution,
                    moveit_config.planning_pipelines,
                    moveit_config.planning_scene_monitor,
                    moveit_config.sensors_3d,
                    {"use_sim_time": use_sim_time},
                ],
                condition=UnlessCondition(launch_as_standalone_node),
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
            moveit_config.trajectory_execution,
            moveit_config.planning_pipelines,
            moveit_config.planning_scene_monitor,
            moveit_config.sensors_3d,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    def set_command_type_action():
        return ExecuteProcess(
            cmd=[
                "ros2",
                "service",
                "call",
                "/servo_node/switch_command_type",
                "moveit_msgs/srv/ServoCommandType",
                "{command_type: 1}",  # default to twist command
            ],
            output="screen",
    )

        # Set servo command type to TwistStamped after servo process starts.
    set_command_type_for_component = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=container,
            on_start=[
                TimerAction(
                    period=8.0,
                    actions=[set_command_type_action()],
                )
            ],
        ),
        condition=UnlessCondition(launch_as_standalone_node),
    )

    set_command_type_for_standalone = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=servo_node,
            on_start=[
                TimerAction(
                    period=8.0,
                    actions=[set_command_type_action()],
                )
            ],
        ),
        condition=IfCondition(launch_as_standalone_node),
    )


    return launch.LaunchDescription(
        declared_arguments + [
            bringup_gazebo,
            # bringup_real,  
            # Delay servo start to let Gazebo and controllers initialize  
            TimerAction(period=12.0, actions=[servo_node, container]),  
            set_command_type_for_standalone,
            set_command_type_for_component,

        ]
    )   