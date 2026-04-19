#!/usr/bin/env python3
"""
Simple test node for MoveIt Servo
Sends target pose commands to test the servo system
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ServoStatus
from tf2_ros import Buffer, TransformListener
import numpy as np


class ServoTestPoseNode(Node):
    def __init__(self):
        super().__init__('servo_test_pose_node')

        # Tuning parameters to avoid overshoot and jitter near target.
        self.declare_parameter('base_frame', 'link0')
        self.declare_parameter('ee_frame', 'end_effector_link')
        self.declare_parameter('stop_tolerance_m', 0.015)
        self.declare_parameter('ball_stationary_epsilon_m', 0.003)

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        self.stop_tolerance_m = (
            self.get_parameter('stop_tolerance_m').get_parameter_value().double_value
        )
        self.ball_stationary_epsilon_m = (
            self.get_parameter('ball_stationary_epsilon_m').get_parameter_value().double_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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
        self.ball_is_stationary = False
        self.goal_reached = False
        self.command_count = 0
        self.get_logger().info(
            f'ServoTestPoseNode initialized: '
            f'listening to /vision/ball_pose_robot (link0 frame) and publishing to servo_node/pose_target_cmds'
        )

    def get_ee_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time(),
            )
            pos = transform.transform.translation
            return np.array([pos.x, pos.y, pos.z], dtype=float)
        except Exception:
            return None

    @staticmethod
    def pose_to_array(msg: PoseStamped):
        return np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
    
    def ball_pose_callback(self, msg):
        """Receive ball pose and send to servo"""
        if not self.servo_active:
            self.get_logger().info('Received ball pose, but servo not active yet')
            return

        current_ball = self.pose_to_array(msg)
        if self.last_ball_pose is None:
            self.last_ball_pose = current_ball
            self.ball_is_stationary = False
        else:
            ball_delta = np.linalg.norm(current_ball - self.last_ball_pose)
            self.ball_is_stationary = ball_delta < self.ball_stationary_epsilon_m
            self.last_ball_pose = current_ball

        ee_pos = self.get_ee_position()
        if ee_pos is not None:
            distance_to_ball = np.linalg.norm(current_ball - ee_pos)
            if distance_to_ball <= self.stop_tolerance_m and self.ball_is_stationary:
                if not self.goal_reached:
                    self.get_logger().info(
                        f'Reached stationary ball within tolerance '
                        f'({distance_to_ball:.3f} m). Holding position.'
                    )
                self.goal_reached = True
                return
            self.goal_reached = False
        
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
                self.goal_reached = False


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
