#!/usr/bin/env python3
"""
Average Pose Publisher Node

Used for pose estimation accuracy test.

The node maintains a rolling buffer of the 5 most recent pose messages.
For each incoming pose, it calculates the average position (x, y, z) across
all poses in the buffer. The orientation is taken from the most recent pose for 
simplicity, as averaging quaternions is hard??

Subscribed Topics:
    - ball_pose_estimation/rob_pose (geometry_msgs/PoseStamped): 
      Raw pose estimates of the ball in the robot frame

Published Topics:
    - /ball_pose_estimation/avg_rob_pose (geometry_msgs/PoseStamped): 
      Smoothed pose computed from the average of the last 5 samples

"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from collections import deque


class AveragePosePublisher(Node):
    def __init__(self):
        super().__init__('average_pose_publisher')
        
        # Buffer to store the 5 most recent poses
        self.pose_buffer = deque(maxlen=5)
        
        # Subscriber to rob_pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            'ball_pose_estimation/rob_pose',
            self.pose_callback,
            10
        )
        
        # Publisher for averaged pose
        self.publisher = self.create_publisher(
            PoseStamped,
            '/ball_pose_estimation/avg_rob_pose',
            10
        )
        
        self.get_logger().info('Average pose publisher node started')
    
    def pose_callback(self, msg):
        # Add the new pose to the buffer
        self.pose_buffer.append(msg)
        
        # Only publish if we have at least one message
        if len(self.pose_buffer) > 0:
            avg_pose = self.calculate_average_pose()
            self.publisher.publish(avg_pose)
            
            self.get_logger().debug(
                f'Published average pose from {len(self.pose_buffer)} samples: '
                f'x={avg_pose.pose.position.x:.4f}, '
                f'y={avg_pose.pose.position.y:.4f}, '
                f'z={avg_pose.pose.position.z:.4f}'
            )
    
    def calculate_average_pose(self):
        # Calculate average position
        avg_x = sum(pose.pose.position.x for pose in self.pose_buffer) / len(self.pose_buffer)
        avg_y = sum(pose.pose.position.y for pose in self.pose_buffer) / len(self.pose_buffer)
        avg_z = sum(pose.pose.position.z for pose in self.pose_buffer) / len(self.pose_buffer)
        
        # Create averaged PoseStamped message
        avg_pose_msg = PoseStamped()
        
        # Use the header from the most recent message
        avg_pose_msg.header = self.pose_buffer[-1].header
        
        # Set averaged position
        avg_pose_msg.pose.position.x = avg_x
        avg_pose_msg.pose.position.y = avg_y
        avg_pose_msg.pose.position.z = avg_z
        
        # Copy orientation from the most recent message
        # (averaging quaternions is more complex, so using the latest)
        avg_pose_msg.pose.orientation = self.pose_buffer[-1].pose.orientation
        
        return avg_pose_msg


def main(args=None):
    rclpy.init(args=args)
    node = AveragePosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
