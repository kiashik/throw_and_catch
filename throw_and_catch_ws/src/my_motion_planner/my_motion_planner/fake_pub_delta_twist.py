#!/usr/bin/env python3
"""
Fake Delta Twist Publisher for testing MoveIt Servo.
Publishes continuous twist commands at high frequency.

Tested with Panda arm. It successfully moved Panda arm.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class FakeTwistPublisher(Node):
    def __init__(self):
        super().__init__('fake_twist_publisher')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)
        # self.declare_parameter('frame_id', 'panda_link0') # for PANDA robot, adjust as needed
        self.declare_parameter('frame_id', 'link0') # for omy_f3m robot
        
        # Keep defaults conservative to reduce singularity/emergency-stop hits.
        self.declare_parameter('linear_x', 0.02)
        self.declare_parameter('linear_y', 0.0)
        self.declare_parameter('linear_z', 0.0)
        self.declare_parameter('angular_x', 0.0)
        self.declare_parameter('angular_y', 0.0)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('auto_reverse_seconds', 2.0)
        
        # Get parameters
        rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.linear_x = self.get_parameter('linear_x').value
        self.linear_y = self.get_parameter('linear_y').value
        self.linear_z = self.get_parameter('linear_z').value
        self.angular_x = self.get_parameter('angular_x').value
        self.angular_y = self.get_parameter('angular_y').value
        self.angular_z = self.get_parameter('angular_z').value
        self.auto_reverse_seconds = self.get_parameter('auto_reverse_seconds').value
        self.direction = 1.0
        self.last_direction_switch = self.get_clock().now()
        
        # Create publisher
        self.publisher = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        # Create timer
        period = 1.0 / rate
        self.timer = self.create_timer(period, self.publish_twist)
        
        self.get_logger().info(f'Publishing twist commands at {rate} Hz')
        self.get_logger().info(f'Linear: [{self.linear_x}, {self.linear_y}, {self.linear_z}]')
        self.get_logger().info(f'Angular: [{self.angular_x}, {self.angular_y}, {self.angular_z}]')
        self.get_logger().info(f'Auto reverse every {self.auto_reverse_seconds}s')
    
    def publish_twist(self):
        if self.auto_reverse_seconds > 0.0:
            elapsed = (self.get_clock().now() - self.last_direction_switch).nanoseconds / 1e9
            if elapsed >= self.auto_reverse_seconds:
                self.direction *= -1.0
                self.last_direction_switch = self.get_clock().now()

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = self.direction * self.linear_x
        msg.twist.linear.y = self.direction * self.linear_y
        msg.twist.linear.z = self.direction * self.linear_z
        msg.twist.angular.x = self.direction * self.angular_x
        msg.twist.angular.y = self.direction * self.angular_y
        msg.twist.angular.z = self.direction * self.angular_z
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeTwistPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
