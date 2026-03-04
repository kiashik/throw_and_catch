#!/usr/bin/env python3

"""
Simple Ball tracking - Uses MoveIt trajectory execution
Subscribes to ball pose and commands arm to move toward it.


how to run:
1) Start robot bringup or Gazebo world with OMY-F3M and table:
   ros2 launch open_manipulator_gazebo open_manipulator_gazebo.launch.py

3) Start this node to track the ball (this runs moveit and RViz too): 
    ros2 launch my_motion_planner track_ball.launch.py use_sim:=true

    
WORKS IN SIM:
IN SIM, If the ball doesn’t move, the robot still moves tothe same ball pose. 
I think since ompl is nondeterministic, it’s computing different traj to the 
same pose every time. We can probably figure out how to only command robot is 
ball has moved a certain distance so the robot doesn’t take a long traj to 
where it already was.

TOOD:maybe we shoudl only plan and excute if the ball has moved a certain distance 
from the last goal, otherwise we might be spamming moveit with new goals every 
time we get a new ball pose, even if it’s just a small update. This would also 
help with the issue of the robot moving to the same pose repeatedly if the ball 
isn’t moving much. We can add a simple distance check before sending a new goal 
to moveit, and only send it if the ball has moved more than a certain threshold (
e.g. 5cm) from the last goal position.
"""

import numpy as np


# rosdeps
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.msg import CollisionObject, Constraints, PositionConstraint, PlanningSceneWorld
from moveit_msgs.msg import PlanningScene
from moveit_msgs.action import MoveGroup


from tf2_ros import TransformListener, Buffer

from rclpy.action import ActionClient


class TrackBall(Node):
    def __init__(self):
        super().__init__('track_ball')
        
        # Ball pose subscriber
        self.ball_pose_sub = self.create_subscription(
            PoseStamped,
            '/ball_pose_estimation/rob_pose',
            self.ball_pose_callback,
            10
        )

        # Planning scene publisher for collision objects
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Add collision objects after a short delay for move_group to be ready
        self._collision_timer = self.create_timer(10.0, self._publish_collision_objects_once)

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ee_frame = "end_effector_link"
        self.base_frame = "world"

        # ---- Move group action client - for trajectory execution -------------
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for move_group action...')
        self.move_action_client.wait_for_server(timeout_sec=15.0)
        # ---------------------------------------------------------------------- 
        
        # Goal state tracking to prevent concurrent execution
        self.goal_in_progress = False
        
        self.current_ball_pose = None
        self.last_goal_target = None
        self.last_ball_update_time = None
        self.goal_position_threshold = 0.06 # Only resend goal if target moved >6cm
        self.ball_lost_timeout = 1.0  # Hold position if no ball updates for this long


        # Control loop timer (20Hz = 50ms)
        self.create_timer(0.05, self.control_loop_callback)
        
        self.get_logger().info("Track Ball node started")

    def _publish_collision_objects_once(self):
        """One-time callback to publish collision objects"""
        self.publish_collision_objects()
        self._collision_timer.cancel()

    def publish_collision_objects(self):
        """Publish table collision object to planning scene"""
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "table"

        table_pose = Pose()
        table_pose.position.x = -0.555
        table_pose.position.y = 0.0
        table_pose.position.z = 0.0
        table_pose.orientation.w = 1.0

        table = SolidPrimitive()
        table.type = SolidPrimitive.BOX
        table.dimensions = [1.53, 0.798, 0.0284]

        collision_object.primitives.append(table)
        collision_object.primitive_poses.append(table_pose)
        collision_object.operation = CollisionObject.ADD

        planning_scene.world.collision_objects.append(collision_object)
        self.planning_scene_pub.publish(planning_scene)
        self.get_logger().info("Table collision object published to planning scene")

    def ball_pose_callback(self, msg: PoseStamped):
        """Store latest ball pose"""
        self.current_ball_pose = msg
        self.last_ball_update_time = self.get_clock().now()
        
    def get_ee_position(self):
        """Get current end-effector position in world frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time()
            )
            pos = transform.transform.translation
            return np.array([pos.x, pos.y, pos.z])
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None
    
    def control_loop_callback(self):
        """Main control loop - continuously tracks ball position"""
        # Don't send new goals while motion is executing
        if self.goal_in_progress:
            return
            
        if self.current_ball_pose is None or self.last_ball_update_time is None:
            return


        time_since_ball = (self.get_clock().now() - self.last_ball_update_time).nanoseconds / 1e9
        if time_since_ball > self.ball_lost_timeout:
            # Hold current position by not issuing new motion goals.
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
        self.get_logger().debug(f"Distance to ball: {distance:.3f}m")

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
            goal = MoveGroup.Goal()
            goal.request.group_name = "arm"
            goal.request.num_planning_attempts = 1
            goal.request.allowed_planning_time = 0.3        # time time might need to match how often a new ball pose is recived
            goal.request.max_acceleration_scaling_factor = 0.5
            goal.request.max_velocity_scaling_factor = 0.5

            # Set target pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.base_frame
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = target_pos[0] * 1.0
            target_pose.pose.position.y = target_pos[1] * 1.0
            target_pose.pose.position.z = target_pos[2] * 1.0
            target_pose.pose.orientation.x = 0.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 1.0

            goal.request.goal_constraints.append(
                self.pose_to_constraint(target_pose, self.ee_frame)
            )

            # Send goal with callbacks to track completion
            self.goal_in_progress = True
            send_goal_future = self.move_action_client.send_goal_async(goal)
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(f"Moving to ball at ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        except Exception as e:
            self.get_logger().warn(f"Control error: {e}")
            self.goal_in_progress = False  # Reset flag on error


    def pose_to_constraint(self, pose: PoseStamped, link_name: str):
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

    def goal_response_callback(self, future):
        """Handle goal acceptance by move_group"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by move_group')
            self.goal_in_progress = False
            return
        
        # Goal accepted, wait for result
        self.get_logger().debug('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle motion completion result"""
        result = future.result().result
        self.goal_in_progress = False  # Reset flag to allow next goal
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().debug('Motion completed successfully')
        else:
            self.get_logger().warn(f'Motion failed with error code: {result.error_code.val}')

def main():
    rclpy.init()
    node = TrackBall()
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

