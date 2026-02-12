from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Your defaults (change these!)
    declared_args = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),

        DeclareLaunchArgument('enable_depth', default_value='false'),
        DeclareLaunchArgument('enable_sync', default_value='false'),
        DeclareLaunchArgument('enable_rgbd', default_value='false'),

        # Useful for perception pipelines
        DeclareLaunchArgument('align_depth.enable', default_value='false'),
        
        DeclareLaunchArgument('spatial_filter.enable', default_value='false'),
        DeclareLaunchArgument('temporal_filter.enable', default_value='false'),
        DeclareLaunchArgument('disparity_filter.enable', default_value='false'),
        DeclareLaunchArgument('hole_filling_filter.enable', default_value='false'),

        # Profiles (format is width,height,fps)
        DeclareLaunchArgument('rgb_camera.color_profile', default_value='640,480,60'),
        DeclareLaunchArgument('depth_module.depth_profile', default_value='640,480,60'),

        DeclareLaunchArgument('output', default_value='screen'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            # Forward your args into rs_launch.py
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_namespace': LaunchConfiguration('camera_namespace'),

            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_sync': LaunchConfiguration('enable_sync'),
            'enable_rgbd': LaunchConfiguration('enable_rgbd'),

            'align_depth.enable': LaunchConfiguration('align_depth.enable'),
            
            'spatial_filter.enable': LaunchConfiguration('spatial_filter.enable'),
            'temporal_filter.enable': LaunchConfiguration('temporal_filter.enable'),
            'disparity_filter.enable': LaunchConfiguration('disparity_filter.enable'),
            'hole_filling_filter.enable': LaunchConfiguration('hole_filling_filter.enable'),

            'rgb_camera.color_profile': LaunchConfiguration('rgb_camera.color_profile'),
            'depth_module.depth_profile': LaunchConfiguration('depth_module.depth_profile'),

            'output': LaunchConfiguration('output'),
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )

    return LaunchDescription(declared_args + [realsense_launch])
