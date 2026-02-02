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


def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    omy = MoveItPy(node_name="my_omy_moveit")
    omy_arm = omy.get_planning_component("arm") # planning group name is "arm" or "gripper"
    logger.info("MoveItPy instance created")

    # brief wait so MoveIt can populate current state/planning scene
    time.sleep(1.0)


    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state to current state
    omy_arm.set_start_state_to_current_state()


    # set pose goal using predefined state
    omy_arm.set_goal_state(configuration_name="init")

    # plan to goal
    plan_and_execute(omy, omy_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan - set goal state with PoseStamped message
    ###########################################################################

    # set 4 pose goals that trace a rectangle in task space
    rectangle_poses = [
        (-0.0, -0.403, 0.335),
    ]

    for idx, (x, y, z) in enumerate(rectangle_poses, start=1):
        # update the start state to the current robot state for each waypoint
        omy_arm.set_start_state_to_current_state()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.orientation.x = 0.479
        pose_goal.pose.orientation.y = 1.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.878

        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        omy_arm.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link="end_effector_link",
        )

        logger.info(f"Planning to rectangle waypoint {idx}: x={x}, y={y}, z={z}")
        plan_and_execute(omy, omy_arm, logger, sleep_time=5.0)
    
    logger.info("Motion completed")

    time.sleep(2.0)  # keep node alive for a bit to inspect in RViz2
    rclpy.shutdown()


if __name__ == "__main__":
    main()