import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from moveit.planning import MoveItPy

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from apriltag_msgs.msg import AprilTagDetectionArray


def plan_and_execute(robot, planning_component, logger):
    logger.info("Planning trajectory")

    plan_result = planning_component.plan()

    if plan_result and plan_result.trajectory:
        logger.info("Executing plan")
        robot.execute(plan_result.trajectory, controllers=[])
    else:
        logger.warn("Planning failed — no execution")


def add_collision_objects(planning_scene_monitor):
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


class TrackAprilTag(Node):
    def __init__(self):
        super().__init__("track_april_tag")

        self.logger = get_logger("moveit_py.track_april_tag")

        self.omy = MoveItPy(node_name="track_april_tag")
        self.omy_arm = self.omy.get_planning_component("arm")
        self.planning_scene_monitor = self.omy.get_planning_scene_monitor()

        self.logger.info("MoveItPy Instance created")

        time.sleep(1.0)

        # self.tag_detected = False
        self._exec_lock = threading.Lock()

        add_collision_objects(self.planning_scene_monitor)
        self.logger.info("Collision object added")

        # self.subscription = self.create_subscription(
        #     PoseStamped,
        #     "/target_pose",
        #     self.target_pose_callback,
        #     10,
        # )

        self.subscription = self.create_subscription(
            PoseStamped,
            "/ball_pose_estimation/rob_pose",
            self.target_pose_callback,
            10,
        )

        # self.tag_sub = self.create_subscription(
        #     AprilTagDetectionArray,
        #     "/apriltag/detections",
        #     self.tag_callback,
        #     10,
        # )

    # def tag_callback(self, msg: AprilTagDetectionArray):
    #     self.tag_detected = len(msg.detections) > 0

    def target_pose_callback(self, msg: PoseStamped):

        # if not self.tag_detected:
        #     self.logger.info("No AprilTag detected — holding position")
        #     return

        if not msg.header.frame_id:
            self.logger.warn("Received pose with empty frame_id")
            return

        # ⚠️ IMPORTANT:
        # This MUST match MoveIt's planning frame.
        # If MoveIt planning frame is base_link, change here.
        if msg.header.frame_id != "world":
            self.logger.warn(
                f"Pose received in frame '{msg.header.frame_id}'. "
                "Ensure this matches MoveIt planning frame."
            )

        if not self._exec_lock.acquire(blocking=False):
            self.logger.warn("Motion already executing — ignoring pose")
            return

        try:
            self.logger.info(
                f"Moving to: x={msg.pose.position.x:.3f}, "
                f"y={msg.pose.position.y:.3f}, "
                f"z={msg.pose.position.z:.3f}"
            )

            self.omy_arm.set_start_state_to_current_state()

            self.omy_arm.set_goal_state(
                pose_stamped_msg=msg,
                pose_link="end_effector_link",
            )

            plan_and_execute(self.omy, self.omy_arm, self.logger)

        except Exception as e:
            self.logger.error(f"Motion failed: {e}")

        finally:
            self._exec_lock.release()


def main():
    rclpy.init()
    node = TrackAprilTag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
