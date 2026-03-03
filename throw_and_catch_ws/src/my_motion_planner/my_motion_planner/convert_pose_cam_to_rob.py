'''
from Raegan
Purpose: 
    Subscribes to a ball pose in camera frame and converts it to robot frame using a precomputed camera --> robot transformation matrix 

    -- subscribes to: '/ball_pose_estimation/pose' (geometry_msgs/PoseStamped)

    -- publishes to:  '/ball_pose_estimation/rob_pose' (geometry_msgs/PoseStamped)

    -- requires: camera_robot_calibration.npy in vision/config --> to get transf matrix

'''


import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped 
import numpy as np 
import os 
from ament_index_python.packages import get_package_share_directory


class BallPoseCamToRobot(Node): 

    def __init__(self): 
        super().__init__('ball_pose_cam_to_robot')

        #load camera to robot transform 
        self.camera_robot_tf = self.load_camera_robot_tf()
        self.get_logger().info(f"Loaded camera -> robot transform:\n{self.camera_robot_tf}")


        #subscription (camera frame pose)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ball_pose_estimation/pose', 
            self.pose_callback, 
            10
        )

        #publisher (robot frame pose)
        self.rob_pose_pub = self.create_publisher(
            PoseStamped, 
            '/ball_pose_estimation/rob_pose', 
            10
        )

        self.get_logger().info("Ball Pose camera -> robot transform node started")

    def load_camera_robot_tf(self): 
        '''
        load 4x4 homogenous transformation matrix from cam to rob frame 
        '''

        pkg_share = get_package_share_directory('vision')
        config_dir = os.path.join(pkg_share, 'config')
        filename = os.path.join(config_dir, 'camera_robot_calibration.npy')

        if not os.path.exists(filename):
            self.get_logger().error(f"Calibration file not found: {filename}")
            raise FileNotFoundError(filename)

        return np.load(filename)
    

    def pose_callback(self, msg: PoseStamped):
        """
        Convert pose from camera frame to robot frame
        """

        # Extract camera-frame position
        ball_in_cam = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            1.0
        ])

        # Transform to robot frame
        ball_in_robot = self.camera_robot_tf @ ball_in_cam

        # Create new PoseStamped message
        rob_pose_msg = PoseStamped()
        rob_pose_msg.header.stamp = self.get_clock().now().to_msg()
        rob_pose_msg.header.frame_id = "robot"   # change if needed

        rob_pose_msg.pose.position.x = float(ball_in_robot[0])
        rob_pose_msg.pose.position.y = float(ball_in_robot[1])
        rob_pose_msg.pose.position.z = float(ball_in_robot[2])

        # Orientation is unchanged (sphere, so irrelevant anyway)
        rob_pose_msg.pose.orientation = msg.pose.orientation

        self.rob_pose_pub.publish(rob_pose_msg)

        self.get_logger().info(
            f"Robot frame ball position: "
            f"x={ball_in_robot[0]:.3f}, "
            f"y={ball_in_robot[1]:.3f}, "
            f"z={ball_in_robot[2]:.3f}"
        )


def main():
    rclpy.init()
    node = BallPoseCamToRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

