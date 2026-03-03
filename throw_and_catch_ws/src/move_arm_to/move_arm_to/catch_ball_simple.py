#!/usr/bin/env python3

"""
Simple Ball Catching - Uses MoveIt trajectory execution
Subscribes to ball pose and commands arm to move toward it.

TODO:
This tracks the ball but is very fast and properly buggy. 
the robto got stuck with this code. I should probably delete this file.


TO run: use the launch file:
ros2 launch move_arm_to catch_ball_simple.launch.py


"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest, CollisionObject, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Vector3
import numpy as np
from tf2_ros import TransformListener, Buffer
import time
from moveit.planning import MoveItPy


def add_collision_objects(planning_scene_monitor):
    """Add table collision object to planning scene"""
    object_positions = (-0.555, 0.0, 0.0)
    object_dimensions = (1.5, 0.768, 0.0254)

    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "table"

        table_pose = Pose()
        table_pose.position.x = float(object_positions[0])
        table_pose.position.y = float(object_positions[1])
        table_pose.position.z = float(object_positions[2])

        table = SolidPrimitive()
        table.type = SolidPrimitive.BOX
        table.dimensions = object_dimensions

        collision_object.primitives.append(table)
        collision_object.primitive_poses.append(table_pose)
        collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()


class CatchBallSimple(Node):
    def __init__(self):
        super().__init__('catch_ball_simple')
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ee_frame = "end_effector_link"
        self.base_frame = "world"
        
        # MoveItPy setup for planning scene
        self.moveit_py = MoveItPy(node_name="catch_ball_simple")
        self.planning_scene_monitor = self.moveit_py.get_planning_scene_monitor()
        
        # Add collision objects
        add_collision_objects(self.planning_scene_monitor)
        self.get_logger().info("Table collision object added")
        
        # IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for IK service...')
        
        # Move group action client - for trajectory execution
        self.move_action_client = None
        self._setup_move_action()
        
        # Ball pose subscriber
        self.ball_pose_sub = self.create_subscription(
            PoseStamped,
            '/ball_pose_estimation/rob_pose',
            self.ball_pose_callback,
            10
        )
        
        # State
        self.current_ball_pose = None
        self.is_catching = False
        self.last_goal_target = None  # Track last goal to avoid sending duplicate goals
        self.goal_position_threshold = 0.02  # Only resend goal if target moved >2cm
        self.ball_lost_timeout = 2.0  # Stop tracking if ball not seen for 2 seconds
        self.last_ball_update_time = None
        
        # Control loop timer (20Hz = 50ms)
        self.create_timer(0.05, self.control_loop_callback)
        
        self.get_logger().info("Catch Ball Simple node started")

    def _setup_move_action(self):
        """Setup move_group action client"""
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup
        
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for move_group action...')
        self.move_action_client.wait_for_server(timeout_sec=10.0)

    def ball_pose_callback(self, msg: PoseStamped):
        """Store latest ball pose"""
        self.current_ball_pose = msg
        if not self.is_catching:
            self.is_catching = True
            self.get_logger().info("Ball detected! Starting to track...")

    def get_ee_position(self):
        """Get current end-effector position in world frame"""
        try:
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

    def control_loop_callback(self):
        """Main control loop - continuously tracks ball position"""
        # Track when we last saw the ball
        if self.current_ball_pose is not None:
            self.last_ball_update_time = self.get_clock().now()
        elif self.last_ball_update_time is not None:
            # Check if ball has been lost for too long
            time_since_update = (self.get_clock().now() - self.last_ball_update_time).nanoseconds / 1e9
            if time_since_update > self.ball_lost_timeout:
                if self.is_catching:
                    self.get_logger().info("Ball lost! Stopping track.")
                    self.is_catching = False
                return
        
        if not self.is_catching or self.current_ball_pose is None:
            return

        # Get current EE position
        ee_pos = self.get_ee_position()
        if ee_pos is None:
            return

        # Get target ball position
        target_pos = np.array([
            self.current_ball_pose.pose.position.x,
            self.current_ball_pose.pose.position.y,
            self.current_ball_pose.pose.position.z
        ])

        # Check distance to ball
        distance = np.linalg.norm(target_pos - ee_pos)
        self.get_logger().info(f"Distance to ball: {distance:.3f}m")

        # Only send new goal if target has moved significantly (debounce to avoid spamming goals)
        should_send_goal = False
        if self.last_goal_target is None:
            should_send_goal = True
        else:
            target_movement = np.linalg.norm(target_pos - self.last_goal_target)
            if target_movement > self.goal_position_threshold:
                should_send_goal = True
        
        if should_send_goal:
            self.send_motion_goal(target_pos)
            self.last_goal_target = target_pos.copy()

    def send_motion_goal(self, target_pos):
        """Send motion goal to move_group action"""
        try:
            from moveit_msgs.action import MoveGroup
            
            goal = MoveGroup.Goal()
            goal.request.group_name = "arm"
            goal.request.num_planning_attempts = 1
            goal.request.allowed_planning_time = 4.0
            goal.request.max_velocity_scaling_factor = 0.49
            goal.request.max_acceleration_scaling_factor = 0.49
            
            # Set target pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.base_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = float(target_pos[0])
            pose_stamped.pose.position.y = float(target_pos[1])
            pose_stamped.pose.position.z = float(target_pos[2])
            pose_stamped.pose.orientation.w = 1.0  # Identity quaternion
            
            goal.request.goal_constraints.append(
                self._pose_to_constraint(pose_stamped, self.ee_frame)
            )
            
            # Send goal (non-blocking)
            self.move_action_client.send_goal_async(goal)
            self.get_logger().info(f"Moving to ball at ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        except Exception as e:
            self.get_logger().warn(f"Control error: {e}")

    def _pose_to_constraint(self, pose: PoseStamped, link_name: str):
        """Convert PoseStamped to PositionConstraint"""
        constraint = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = link_name
        pos_constraint.target_point_offset = Vector3()
        
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm radius
        
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0
        
        constraint.position_constraints.append(pos_constraint)
        
        return constraint


def main():
    rclpy.init()
    node = CatchBallSimple()
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

