#!/usr/bin/env python3
"""
Simple test node for MoveIt Servo
Sends continuous circular Cartesian twist commands to test the servo system
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import ServoStatus
import math
import time


class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test_node')
        
        # Parameters
        self.declare_parameter('frame_id', 'link0')  # End-effector frame for twist commands
        self.declare_parameter('frequency', 0.5)  # Hz - how fast the wave oscillates
        self.declare_parameter('amplitude', 0.05)  # m/s - max linear velocity
        self.declare_parameter('publish_rate', 10)  # Hz - how often to publish commands
        
        self.frame_id = self.get_parameter('frame_id').value
        self.frequency = self.get_parameter('frequency').value
        self.amplitude = self.get_parameter('amplitude').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publisher for Cartesian twist commands
        self.twist_cmd_pub = self.create_publisher(
            TwistStamped,
            'servo_node/delta_twist_cmds',
            10
        )
        
        # Subscriber to servo status
        self.servo_status_sub = self.create_subscription(
            ServoStatus,
            'servo_node/status' ,
            self.servo_status_callback,
            10
        )
        
        # State
        self.servo_active = False
        self.start_time = None
        self.command_count = 0
        
        # Timer for publishing commands
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info(
            f'ServoTestNode initialized: '
            f'frame_id={self.frame_id}, '
            f'frequency={self.frequency}Hz, '
            f'amplitude={self.amplitude}m/s, '
            f'publish_rate={self.publish_rate}Hz'
        )
    
    def servo_status_callback(self, msg):
        """Monitor servo status"""
        if msg.code == 0:  # 0 = success/active
            if not self.servo_active:
                self.get_logger().info('✓ Servo is ACTIVE')
                self.servo_active = True
                self.start_time = time.time()
        else:
            if self.servo_active:
                self.get_logger().warn(f'✗ Servo ERROR: code={msg.code}, message={msg.message}')
                self.servo_active = False
    
    def timer_callback(self):
        """Publish circular Cartesian twist commands"""
        if not self.servo_active:
            return
        
        if self.start_time is None:
            self.start_time = time.time()
        
        # Elapsed time since start
        elapsed = time.time() - self.start_time
        
        # Generate circular motion in XY plane
        angle = 2 * math.pi * self.frequency * elapsed
        
        # Linear velocities for circular motion
        linear_x = self.amplitude * math.cos(angle)  # X direction
        linear_y = self.amplitude * math.sin(angle)  # Y direction
        linear_z = 0.0  # No Z motion
        
        # Angular velocities (optional rotation around Z)
        angular_x = 0.0
        angular_y = 0.0
        angular_z = 0.0  # Could add rotation: 0.1 * math.sin(angle)
        
        # Create and publish TwistStamped message
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.linear.z = linear_z
        msg.twist.angular.x = angular_x
        msg.twist.angular.y = angular_y
        msg.twist.angular.z = angular_z
        
        self.twist_cmd_pub.publish(msg)
        
        self.command_count += 1
        
        # Log every 20 commands (at 10 Hz = every 2 seconds)
        if self.command_count % 20 == 0:
            self.get_logger().info(
                f'[{self.command_count}] Twist: '
                f'x={linear_x:.3f}, y={linear_y:.3f}, z={linear_z:.3f} m/s '
                f'(frame: {self.frame_id})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ServoTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
