#!/usr/bin/env python3
"""
Fake Delta Twist Publisher for testing MoveIt Servo.
Publishes continuous twist commands at high frequency.

tested with panda arm. it successfully moved panada arm.
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
        
        self.declare_parameter('linear_x', 0.1)
        self.declare_parameter('linear_y', 0.0)
        self.declare_parameter('linear_z', 0.0)
        self.declare_parameter('angular_x', 0.0)
        self.declare_parameter('angular_y', 0.0)
        self.declare_parameter('angular_z', 0.0)
        
        # Get parameters
        rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.linear_x = self.get_parameter('linear_x').value
        self.linear_y = self.get_parameter('linear_y').value
        self.linear_z = self.get_parameter('linear_z').value
        self.angular_x = self.get_parameter('angular_x').value
        self.angular_y = self.get_parameter('angular_y').value
        self.angular_z = self.get_parameter('angular_z').value
        
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
    
    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = self.linear_x
        msg.twist.linear.y = self.linear_y
        msg.twist.linear.z = self.linear_z
        msg.twist.angular.x = self.angular_x
        msg.twist.angular.y = self.angular_y
        msg.twist.angular.z = self.angular_z
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