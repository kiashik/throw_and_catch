#!/usr/bin/env python3

# adapted from open_manipulator_playground/omy_f3m_hello_moveit.py

import time
import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import Pose, PoseStamped


def main():
    # =====================================================================
    # PLANNER CONFIGURATION - Change this to switch planners
    # =====================================================================
    # Available config namespaces (defined in my_motion_planning_python_api_tutorial.yaml):
    # ARM PLANNERS (for pose/cartesian goals):
    # - "pilz_ptp"       : Pilz Point-to-Point motion
    # - "pilz_lin"       : Pilz Linear (straight-line) motion
    # - "ompl_rrtc"      : OMPL RRTConnect planner
    # - "chomp_planner"  : CHOMP optimization planner
    # GRIPPER PLANNER (for joint goals - Pilz doesn't support grippers):
    # - "ompl_gripper"   : OMPL planner for gripper
    
    ARM_PLAN_CONFIG = "ompl_rrtc"      # Change this to switch arm planners
    GRIPPER_PLAN_CONFIG = "ompl_gripper"  # Gripper uses OMPL (Pilz requires IK)
    # =====================================================================

    # Initialize the ROS2 node
    rclpy.init()

    # Create a logger for logging messages
    logger = get_logger("hello_moveit")

    # Create the MoveIt MoveItPy instance and get the "arm" planning group
    omy = MoveItPy(node_name="hello_moveit")
    move_group_interface = omy.get_planning_component("arm")

    # Define the target pose for the robot arm
    target_pose = PoseStamped()
    target_pose.header.frame_id = "world"
    target_pose.pose.orientation.x = 0.0  # Orientation (quaternion x)
    target_pose.pose.orientation.y = 0.0  # Orientation (quaternion y)
    target_pose.pose.orientation.z = 0.0  # Orientation (quaternion z)
    target_pose.pose.orientation.w = 1.0  # Orientation (quaternion w)
    target_pose.pose.position.x = -0.062   # Position in x
    target_pose.pose.position.y = -0.468  # Position in y
    target_pose.pose.position.z = 0.599   # Position in z

    # Set the target pose for the arm
    move_group_interface.set_goal_state(pose_stamped_msg=target_pose, pose_link="end_effector_link")

    # Load planner parameters from config namespace
    arm_plan_params = PlanRequestParameters(omy, ARM_PLAN_CONFIG)

    # Plan the motion for the arm to reach the target pose
    logger.info(f"Planning ARM motion using config: {ARM_PLAN_CONFIG}...")
    plan_result = move_group_interface.plan(single_plan_parameters=arm_plan_params)

    # If planning succeeds, execute the planned motion
    if plan_result:
        logger.info("Executing planned motion...")
        omy.execute(plan_result.trajectory, controllers=[])
        time.sleep(2)
    else:
        logger.error("Planning failed for the arm!")

    # Get the gripper planning group
    logger.info("Moving gripper to close position...")
    gripper_interface = omy.get_planning_component("gripper")
    gripper_plan_params = PlanRequestParameters(omy, GRIPPER_PLAN_CONFIG)

    # Set the "close" position for the gripper and move it
    gripper_interface.set_goal_state(configuration_name="close")
    gripper_result = gripper_interface.plan(single_plan_parameters=gripper_plan_params)
    if gripper_result:
        omy.execute(gripper_result.trajectory, controllers=[])
        logger.info("Gripper closed successfully")
        time.sleep(2)
    else:
        logger.error("Failed to close the gripper")

    # Move the arm back to the "home" position
    logger.info("Moving arm back to home position...")
    move_group_interface.set_goal_state(configuration_name="home")
    home_result = move_group_interface.plan(single_plan_parameters=arm_plan_params)
    if home_result:
        omy.execute(home_result.trajectory, controllers=[])
        logger.info("Arm moved back to home position")
        time.sleep(2)

        # Open the gripper
        logger.info("Opening gripper...")
        gripper_interface.set_goal_state(configuration_name="open")
        open_result = gripper_interface.plan(single_plan_parameters=gripper_plan_params)
        if open_result:
            omy.execute(open_result.trajectory, controllers=[])
            logger.info("Gripper opened successfully")
        else:
            logger.error("Failed to open the gripper")

    else:
        logger.error("Failed to move the arm back to home position")

    # Shut down the ROS2 node
    rclpy.shutdown()


if __name__ == "__main__":
    main()
