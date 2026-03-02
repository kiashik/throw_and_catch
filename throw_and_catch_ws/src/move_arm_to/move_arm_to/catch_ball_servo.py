#!/usr/bin/env python3

"""
Ball Catching with MoveIt Servo
Subscribes to ball pose and drives end-effector to track it in real-time.

Subscribes to: /ball_pose_estimation/rob_pose (PoseStamped)
Publishes to: /servo_server/delta_twist_cmds (geometry_msgs/TwistStamped)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
from tf2_ros import TransformListener, Buffer
import threading


class CatchBallServo(Node):
    def __init__(self):
        super().__init__('catch_ball_servo')
        
        # =====================================================================
        # CONTROL PARAMETERS - Adjust these for tuning
        # =====================================================================
        self.LINEAR_VELOCITY_SCALE = 0.3      # m/s per meter of error
        self.ANGULAR_VELOCITY_SCALE = 0.5     # rad/s per radian of error
        self.POSITION_TOLERANCE = 0.02        # m (stop when within 2cm)
        self.SERVO_PUBLISH_RATE = 50          # Hz (20ms update)
        # =====================================================================
        
        # TF2 setup for getting end-effector position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ee_frame = "end_effector_link"
        self.base_frame = "world"
        
        # Ball pose subscriber
        self.ball_pose_sub = self.create_subscription(
            PoseStamped,
            '/ball_pose_estimation/rob_pose',
            self.ball_pose_callback,
            10
        )
        
        # Servo velocity command publisher
        self.servo_vel_pub = self.create_publisher(
            TwistStamped,
            '/servo_server/delta_twist_cmds',
            10
        )
        
        # State tracking
        self.current_ball_pose = None
        self.is_catching = False
        
        # Control loop timer
        self.create_timer(1.0 / self.SERVO_PUBLISH_RATE, self.control_loop_callback)
        
        self.get_logger().info("Catch Ball Servo node started")
        self.get_logger().info(f"  EE Frame: {self.ee_frame}")
        self.get_logger().info(f"  Base Frame: {self.base_frame}")
        self.get_logger().info(f"  Position Tolerance: {self.POSITION_TOLERANCE}m")
        self.get_logger().info(f"  Servo Rate: {self.SERVO_PUBLISH_RATE}Hz")

    def ball_pose_callback(self, msg: PoseStamped):
        """Store latest ball pose"""
        self.current_ball_pose = msg
        if not self.is_catching:
            self.is_catching = True
            self.get_logger().info("Ball detected! Starting to track...")

    def get_ee_position(self):
        """Get current end-effector position in world frame"""
        try:
            # Get transform from base_frame to ee_frame
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time()
            )
            
            pos = transform.transform.translation
            return np.array([pos.x, pos.y, pos.z])
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def pose_to_array(self, pose_stamped: PoseStamped):
        """Extract 3D position from PoseStamped"""
        return np.array([
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z
        ])

    def control_loop_callback(self):
        """Main control loop - runs at servo_publish_rate"""
        if not self.is_catching or self.current_ball_pose is None:
            return

        # Get current end-effector position
        ee_pos = self.get_ee_position()
        if ee_pos is None:
            return

        # Get target ball position
        ball_pos = self.pose_to_array(self.current_ball_pose)

        # Compute position error
        error_pos = ball_pos - ee_pos
        error_magnitude = np.linalg.norm(error_pos)

        # Check if we've reached the ball
        if error_magnitude < self.POSITION_TOLERANCE:
            self.get_logger().info(f"✓ Close to ball! Distance: {error_magnitude:.3f}m")
            # Publish zero velocity
            self.publish_zero_velocity()
            self.is_catching = False
            return

        # Compute proportional velocity command (simple P-controller)
        # Scale error to velocity: larger error → higher velocity
        linear_velocity = error_pos * self.LINEAR_VELOCITY_SCALE

        # Clamp velocity magnitude (safety)
        max_velocity = 0.5  # m/s max
        vel_magnitude = np.linalg.norm(linear_velocity)
        if vel_magnitude > max_velocity:
            linear_velocity = (linear_velocity / vel_magnitude) * max_velocity

        # Create TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.ee_frame

        # Linear velocity
        twist_msg.twist.linear.x = float(linear_velocity[0])
        twist_msg.twist.linear.y = float(linear_velocity[1])
        twist_msg.twist.linear.z = float(linear_velocity[2])

        # No angular velocity for now (orientation doesn't matter for sphere)
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        # Publish velocity command
        self.servo_vel_pub.publish(twist_msg)

        # Log occasional status
        if int(error_magnitude * 100) % 5 == 0:  # Log every ~10 updates
            self.get_logger().info(
                f"Tracking ball | Distance: {error_magnitude:.3f}m | "
                f"Vel: {vel_magnitude:.3f}m/s"
            )

    def publish_zero_velocity(self):
        """Publish zero velocity command to stop motion"""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.ee_frame
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.servo_vel_pub.publish(twist_msg)


def main():
    rclpy.init()
    node = CatchBallServo()
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
