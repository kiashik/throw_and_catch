#!/usr/bin/env python3


from platform import node
import time


# generic ros libraries
import rclpy
from rclpy.logging import get_logger


# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
   MoveItPy,
   MultiPipelinePlanRequestParameters,
)
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive




def plan_and_execute(
   robot,
   planning_component,
   logger,
   single_plan_parameters=None,
   multi_plan_parameters=None,
   sleep_time=0.0,
):
   """Helper function to plan and execute a motion."""
   # plan to goal
   logger.info("Planning trajectory")
   if multi_plan_parameters is not None:
       plan_result = planning_component.plan(
           multi_plan_parameters=multi_plan_parameters
       )
   elif single_plan_parameters is not None:
       plan_result = planning_component.plan(
           single_plan_parameters=single_plan_parameters
       )
   else:
       plan_result = planning_component.plan()


   # execute the plan
   if plan_result:
       logger.info("Executing plan")
       robot_trajectory = plan_result.trajectory
       robot.execute(robot_trajectory, controllers=[])
   else:
       logger.error("Planning failed")


   time.sleep(sleep_time)


def add_collision_objects(planning_scene_monitor):
   object_positions = (-0.555, 0.0, 0.0) # meters
   object_dimensions = (1.5,0.768,0.0254) # meters


   # acquire a read-write lock on the planning scene (locks the scene until I make all changes)
   with planning_scene_monitor.read_write() as scene:
       # create a collsion onject message
       collision_object = CollisionObject()


       # all object poses are expressed relative to this frame
       collision_object.header.frame_id = "world"


       # Unique ID for this collision object
       collision_object.id = "table"


       # Add table primitive and its pose
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
       scene.current_state.update()    # important to ensure scene is updated




def main():
   ###################################################################
   # MoveItPy Setup
   ###################################################################
   rclpy.init()
   logger = get_logger("moveit_py.pose_goal")


   # instantiate MoveItPy instance and get planning component
   omy = MoveItPy(node_name="my_omy_moveit")
   omy_arm = omy.get_planning_component("arm") # planning group name is "arm" or "gripper"


   # get scene monitor
   planning_scene_monitor = omy.get_planning_scene_monitor()


   logger.info("MoveItPy instance created")


   # brief wait so MoveIt can populate current state/planning scene
   time.sleep(1.0)


   ###########################################################################
   # Plan with collision objects
   ###########################################################################
   add_collision_objects(planning_scene_monitor)
   logger.info("table added")
   try:
      while rclpy.ok():
         time.sleep(0.5)
   except KeyboardInterrupt:
        pass
   rclpy.shutdown()


   # ###########################################################################
   # # Plan 1 - set states with predefined string
   # ###########################################################################


   # # set plan start state to current state
   # omy_arm.set_start_state_to_current_state()




   # # set pose goal using predefined state
   # omy_arm.set_goal_state(configuration_name="init")


   # # plan to goal
   # plan_and_execute(omy, omy_arm, logger, sleep_time=3.0)


   # ###################################################################
   # # Check collisions
   # ###################################################################
   # with planning_scene_monitor.read_only() as scene:
   #     robot_state = scene.current_state
   #     original_joint_positions = robot_state.get_joint_group_positions("omy_arm")


   #     # Set the pose goal
   #     pose_goal = Pose()
   #     pose_goal.position.x = -0.0
   #     pose_goal.position.y = -0.403
   #     pose_goal.position.z = 0.335
   #     pose_goal.orientation.w = 1.0


   #     # Set the robot state and check collisions
   #     robot_state.set_from_ik("omy_arm", pose_goal, "end_effector_link")
   #     robot_state.update()  # required to update transforms
   #     robot_collision_status = scene.is_state_colliding(
   #         robot_state=robot_state, joint_model_group_name="omy_arm", verbose=True
   #     )
   #     logger.info(f"\nRobot is in collision: {robot_collision_status}\n")


   #     # Restore the original state
   #     robot_state.set_joint_group_positions(
   #         "omy_arm",
   #         original_joint_positions,
   #     )
   #     robot_state.update()  # required to update transforms


   # time.sleep(3.0)




   ###########################################################################
   # Plan - set goal state with PoseStamped message
   ###########################################################################


   # # set 4 pose goals that trace a rectangle in task space
   # rectangle_poses = [
   #     (-0.0, -0.403, 0.335),
   #     (0.0, 0.0, 0.25),
   # ]


   # for idx, (x, y, z) in enumerate(rectangle_poses, start=1):
   #     # update the start state to the current robot state for each waypoint
   #     omy_arm.set_start_state_to_current_state()


   #     pose_goal = PoseStamped()
   #     pose_goal.header.frame_id = "world"
   #     pose_goal.pose.orientation.x = 0.479
   #     pose_goal.pose.orientation.y = 1.0
   #     pose_goal.pose.orientation.z = 0.0
   #     pose_goal.pose.orientation.w = 0.878


   #     pose_goal.pose.position.x = x
   #     pose_goal.pose.position.y = y
   #     pose_goal.pose.position.z = z
   #     omy_arm.set_goal_state(
   #         pose_stamped_msg=pose_goal,
   #         pose_link="end_effector_link",
   #     )


   #     logger.info(f"Planning to rectangle waypoint {idx}: x={x}, y={y}, z={z}")
   #     plan_and_execute(omy, omy_arm, logger, sleep_time=1.0)
  
   # logger.info("Motion completed")


   # time.sleep(2.0)  # keep node alive for a bit to inspect in RViz2
   # rclpy.shutdown()




if __name__ == "__main__":
   main()
