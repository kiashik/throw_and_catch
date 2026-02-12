"""
I'm following the example from the ROS 2 launch documentation to create a launch 
file that loads custom parameters from a YAML file for the RealSense camera. see
the file "realsense-ros/realsense2_camera/examples/launch_params_from_file"


Purpose: launch the camera bringup with a set of params that work for our project.
This way we can "disable" all unnecessary data from the camera to save bandwidth.

## Syntax for defining params in YAML file
```
param1: value
param2: value
```
"""

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

# these are the custom parameters that we are using for our throw and catch project.
# note that the last parameter is the config file that lists most of the parameters
local_parameters = [{'name': 'camera_name',         'default': 'camera', 'description': 'camera unique name'},
                    {'name': 'camera_namespace',    'default': 'camera', 'description': 'camera namespace'},
                    {'name': 'config_file',         'default': os.path.join(get_package_share_directory('vision'), 'config', 'rs_launch_config.yaml'), 'description': 'yaml config file for realsense camera'},

                    ]

def set_configurable_parameters(local_params):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])


def generate_launch_description():
    params = rs_launch.configurable_parameters
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) + 
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                kwargs = {'params' : set_configurable_parameters(params)}
        )
    ])
