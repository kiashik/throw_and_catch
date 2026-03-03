#!/usr/bin/env python3

"""
Fake Ball Pose Publisher : used for testing without actual camera.
Publishes random ball poses for testing the catch_ball_servo node.
Publishes to: /ball_pose_estimation/rob_pose (geometry_msgs/PoseStamped)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakeBallPosePublisher(Node):
    def __init__(self):
        super().__init__('fake_ball_pose_publisher')
        
        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/ball_pose_estimation/rob_pose',
            10
        )
        
        # Realistic reachable poses for OMY-F3M robot (x, y, z in meters)
        # draw a rectangle in front of the robot within its workspace
        self.poses = [
            (-0.3, -0.4, 0.2),
            (-0.3, -0.4, 0.6),
            ( 0.3, -0.4, 0.2),
            ( 0.3,  0.4, 0.6),
        ]
        self.current_index = 0
        
        # Timer: publish every 3 seconds (gives planner time to execute)
        self.timer = self.create_timer(3.0, self.timer_callback)
        
        self.get_logger().info("Fake Ball Pose Publisher started")
        self.get_logger().info(f"Publishing {len(self.poses)} test poses in robot workspace")
        for i, pose in enumerate(self.poses):
            self.get_logger().info(
                f"  Pose {i+1}: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}"
            )

    def timer_callback(self):
        """Publish the current pose and increment index"""
        # Get current pose
        x, y, z = self.poses[self.current_index]
        
        # Create PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Orientation (doesn't matter for sphere)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        
        # Publish
        self.pose_pub.publish(msg)
        
        self.get_logger().info(
            f"Published pose {self.current_index + 1}/{len(self.poses)}: "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )
        
        # Increment index (loop back to 0 after last pose)
        self.current_index = (self.current_index + 1) % len(self.poses)


def main():
    rclpy.init()
    node = FakeBallPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
