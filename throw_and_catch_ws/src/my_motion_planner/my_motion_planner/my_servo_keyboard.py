#!/usr/bin/env python3
"""
Keyboard teleop for MoveIt Servo Twist commands.

Publishes geometry_msgs/TwistStamped on /servo_node/delta_twist_cmds.
Default command frame is link0 (base frame).

works in sim.
"""

import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class MyServoKeyboard(Node):
    def __init__(self):
        super().__init__("my_servo_keyboard")

        self.declare_parameter("topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("frame_id", "link0")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("linear_step", 0.02)
        self.declare_parameter("angular_step", 0.3)

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.linear_step = float(self.get_parameter("linear_step").value)
        self.angular_step = float(self.get_parameter("angular_step").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.publisher = self.create_publisher(TwistStamped, topic, 10)

        self.lin_x = 0.0
        self.lin_y = 0.0
        self.lin_z = 0.0
        self.ang_x = 0.0
        self.ang_y = 0.0
        self.ang_z = 0.0
        self.running = True
        self.lock = threading.Lock()

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_twist)

        self.get_logger().info(f"Publishing to {topic} in frame '{self.frame_id}'")
        self.get_logger().info(
            "Keys: w/s x, a/d y, r/f z, j/l yaw-z, y/h pitch-y, u/o roll-x, space stop, q quit"
        )

    def publish_twist(self):
        with self.lock:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.twist.linear.x = self.lin_x
            msg.twist.linear.y = self.lin_y
            msg.twist.linear.z = self.lin_z
            msg.twist.angular.x = self.ang_x
            msg.twist.angular.y = self.ang_y
            msg.twist.angular.z = self.ang_z
        self.publisher.publish(msg)

    def set_twist(self, key):
        with self.lock:
            if key == "w":
                self.lin_x = self.linear_step
            elif key == "s":
                self.lin_x = -self.linear_step
            elif key == "a":
                self.lin_y = self.linear_step
            elif key == "d":
                self.lin_y = -self.linear_step
            elif key == "r":
                self.lin_z = self.linear_step
            elif key == "f":
                self.lin_z = -self.linear_step
            elif key == "u":
                self.ang_x = self.angular_step
            elif key == "o":
                self.ang_x = -self.angular_step
            elif key == "y":
                self.ang_y = self.angular_step
            elif key == "h":
                self.ang_y = -self.angular_step
            elif key == "j":
                self.ang_z = self.angular_step
            elif key == "l":
                self.ang_z = -self.angular_step
            elif key == " ":
                self.lin_x = 0.0
                self.lin_y = 0.0
                self.lin_z = 0.0
                self.ang_x = 0.0
                self.ang_y = 0.0
                self.ang_z = 0.0


def _keyboard_loop(node: MyServoKeyboard):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok() and node.running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key == "q":
                node.running = False
                break
            node.set_twist(key)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = MyServoKeyboard()

    key_thread = threading.Thread(target=_keyboard_loop, args=(node,), daemon=True)
    key_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
