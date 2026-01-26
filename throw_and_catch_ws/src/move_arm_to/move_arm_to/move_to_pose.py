#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit import MoveItPy
from moveit.planning import PlanningComponent

import time

class MoveToPose(Node):
    def __init__(self):
        # use keyword argument for compatibility
        super().__init__(node_name="move_to_pose")

        # Connect to MoveIt (expects parameters injected via launch)
        self.moveit = MoveItPy(node_name="move_to_pose")

        # planning group from your SRDF
        self.arm = PlanningComponent("arm", self.moveit)

        # small delay to let current state / TF come up (optional)
        rclpy.spin_once(self, timeout_sec=0.1)

        # perform the motion
        self.move_robot()

    def move_robot(self):
        # make sure current state is available
        self.arm.set_start_state_to_current_state()

        target = PoseStamped()
        # use your robot's base frame (common names: 'base_link', 'link0', etc.)
        target.header.frame_id = "base_link"
        target.pose.position.x = 0.3
        target.pose.position.y = 0.0
        target.pose.position.z = 0.2
        target.pose.orientation.w = 1.0

        self.arm.set_goal_state(pose_stamped_msg=target)

        # Plan (this returns a PlanResult object or similar)
        plan_result = self.arm.plan()

        if not plan_result:
            self.get_logger().error("Planning failed")
            return

        # Execute; MoveItPy expects the planned trajectory object
        self.moveit.execute(plan_result.trajectory)
        self.get_logger().info("Motion executed")

def main():
    rclpy.init()
    node = MoveToPose()

    # Keep the node alive so MoveIt internals and logs can finish.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

