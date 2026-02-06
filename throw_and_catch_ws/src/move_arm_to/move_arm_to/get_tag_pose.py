#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import TransformListener, Buffer
# import time
import os
from ament_index_python.packages import get_package_share_directory

import matplotlib.pyplot as plt

class GetTagPose(Node):
    def __init__(self):
        super().__init__('get_tag_pose')
        
        self.camera_robot_tf = self.load_camera_robot_tf()
        print(self.camera_robot_tf)

        # TF listener to get camera -> tag transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(2.0, self.timer_callback)
        
    def timer_callback(self):
        self.tag_in_cam = self.get_tag_in_cam()
        print(f'tag_in_cam:{self.tag_in_cam}')

        self.tag_pose_robot = self.get_tag_in_robot(self.camera_robot_tf, self.tag_in_cam)
        print(f'tag_pose_robot:{self.tag_pose_robot}')
        
        
    def load_camera_robot_tf(self): # returns a 4x4 numpy tf matrix of the camera robot transform
        # load camera to robot transformation ()
        # from cal_cam_robot.py in vision package
        pkg_share = get_package_share_directory('vision')
        config_dir = os.path.join(pkg_share, 'config')
        os.makedirs(config_dir, exist_ok=True)

        filename = os.path.join(config_dir, 'camera_robot_calibration.npy')
        camera_robot_tf = np.load(filename)
        return camera_robot_tf
        
    def get_tag_in_cam(self):
        # returns a 1x4 vector of an x,y,z,1 pose in camera frame
        # has to be tag_ID = 0

        tag_id = 0
        frame_name = f'tag36h11:{tag_id}'
        
        try:
            transform = self.tf_buffer.lookup_transform(
                        'camera_color_optical_frame',
                        frame_name,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                
            # Extract position
            pos = transform.transform.translation
            tag_in_cam = np.array([pos.x, pos.y, pos.z, 1]) # gives x,y,z in camera base frame
            
            return tag_in_cam
        
        except Exception as e:
                self.get_logger().debug(f"Could not get transform for tag {tag_id}: {e}")
                pass
        
    def get_tag_in_robot(self, camera_robot_tf, tag_in_cam):
        # returns a 1x3 vector of an x,y,z pose in robot frame
        # matrix multiplies the camera to robot transfromation (4x4) with the tag pose in camera frame (1x4)
        tag_in_robot = camera_robot_tf @ tag_in_cam.T
        tag_pose_robot = tag_in_robot.T
        return tag_pose_robot
        

def main():
    rclpy.init()
    node = GetTagPose()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
