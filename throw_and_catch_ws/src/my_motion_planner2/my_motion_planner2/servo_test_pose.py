#!/usr/bin/env python3
"""
Simple test node for MoveIt Servo
Sends target pose commands to test the servo system
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ServoStatus


class ServoTestPoseNode(Node):
    def __init__(self):
        super().__init__('servo_test_pose_node')
        
        # Publisher for pose target commands
        self.pose_cmd_pub = self.create_publisher(
            PoseStamped,
            'servo_node/pose_target_cmds',
            10
        )
        
        # Subscriber to ball pose in robot link0 frame
        self.ball_pose_sub = self.create_subscription(
            PoseStamped,
            '/vision/ball_pose_robot',
            self.ball_pose_callback,
            10
        )
        
        # Subscriber to servo status
        self.servo_status_sub = self.create_subscription(
            ServoStatus,
            'servo_node/status',
            self.servo_status_callback,
            10
        )
        
        # State
        self.servo_active = False
        self.last_ball_pose = None
        self.command_count = 0
        self.get_logger().info(
            f'ServoTestPoseNode initialized: '
            f'listening to /vision/ball_pose_robot (link0 frame) and publishing to servo_node/pose_target_cmds'
        )
    
    def ball_pose_callback(self, msg):
        """Receive ball pose and send to servo"""
        if not self.servo_active:
            self.get_logger().info('Received ball pose, but servo not active yet')
            return
        
        # Assume incoming pose is already in robot link0 frame
        target_pose = PoseStamped()
        target_pose.header = msg.header
        target_pose.pose = msg.pose
        
        if msg.header.frame_id != 'link0':
            self.get_logger().warn(
                f'Expected incoming ball pose in frame link0, got {msg.header.frame_id}. '
                'Publishing pose as-is with frame_id=link0.'
            )
            target_pose.header.frame_id = 'link0'
        
        self.pose_cmd_pub.publish(target_pose)
        self.command_count += 1
        
        if self.command_count % 100 == 0:
            self.get_logger().info(
                f'Published pose command #{self.command_count}: '
                f'x={target_pose.pose.position.x:.3f}, y={target_pose.pose.position.y:.3f}, z={target_pose.pose.position.z:.3f} '
                f'in frame {target_pose.header.frame_id}'
            )
    
    def servo_status_callback(self, msg):
        """Monitor servo status"""
        self.get_logger().info(f'Servo status: code={msg.code}, message={msg.message}')
        if msg.code == 0:  # 0 = success/active
            if not self.servo_active:
                self.get_logger().info('Servo is ACTIVE')
                self.servo_active = True
        else:
            if self.servo_active:
                self.get_logger().warn(f'✗ Servo ERROR: code={msg.code}, message={msg.message}')
                self.servo_active = False


def main(args=None):
    rclpy.init(args=args)
    node = ServoTestPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
