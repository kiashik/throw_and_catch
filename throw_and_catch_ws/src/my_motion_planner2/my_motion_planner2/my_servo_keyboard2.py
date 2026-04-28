#!/usr/bin/env python3
"""
Keyboard teleop for MoveIt Servo Twist commands.

Publishes geometry_msgs/TwistStamped on /servo_node/delta_twist_cmds.
Default command frame is link0 (base frame).

How to run:
    ros2 run my_motion_planner my_servo_keyboard
"""


import select
import sys
import termios
import threading
import tty

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node


class MyServoKeyboard(Node):
    def __init__(self):
        super().__init__("my_servo_keyboard")

        self.declare_parameter("topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("frame_id", "link0")
        self.declare_parameter("publish_rate", 200.0)
        self.declare_parameter("linear_step", 0.40)
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

        self._gripper_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )

        self.get_logger().info(f"Publishing to {topic} in frame '{self.frame_id}'")
        self.get_logger().info(
            "Keys: w/s x, a/d y, r/f z, j/l yaw-z, y/h pitch-y, u/o roll-x, space stop, g open gripper, b close gripper, q quit"
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
            elif key == "g":
                self._send_gripper_goal(position=0.0)   # open
            elif key == "b":
                self._send_gripper_goal(position=0.55)  # close (max position is 1.14)

    def _send_gripper_goal(self, position: float, max_effort: float = 10.0):
        """Non-blocking gripper command. Silently skips if action server not available."""
        if not self._gripper_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Gripper action server not available")
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self._gripper_client.send_goal_async(goal)

    def _zero_all(self):
        with self.lock:
            self.lin_x = 0.0
            self.lin_y = 0.0
            self.lin_z = 0.0
            self.ang_x = 0.0
            self.ang_y = 0.0
            self.ang_z = 0.0

    def run_keyboard(self):
        """Read keypresses in raw terminal mode. Velocities reset to zero if no key received for 0.3 s."""
        old_settings = termios.tcgetattr(sys.stdin)
        last_key_time = self.get_clock().now()
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1)
                    if key == "q":
                        self.running = False
                        break
                    self.set_twist(key)
                    last_key_time = self.get_clock().now()
                else:
                    elapsed = (self.get_clock().now() - last_key_time).nanoseconds / 1e9
                    if elapsed > 0.3:
                        self._zero_all()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def destroy_node(self):
        """Publish a zero twist before destroying the node."""
        self._zero_all()
        for _ in range(10):
            self.publish_twist()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MyServoKeyboard()

    # Spin in a background thread so the main thread can run the blocking keyboard loop.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_keyboard()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
