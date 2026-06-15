""""
Save all ball position pub'ed to /vision/ball_pose_robot and plot is upon Ctrl+C.


"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np
from matplotlib import pyplot as plt

class BallTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('ball_trajectory_planner')
        self.positions = []

        ball_pose_robot_topic = '/vision/ball_pose_robot'
        self.subscription = self.create_subscription(
            PoseStamped,
            ball_pose_robot_topic,
            self.pose_callback,
            10
        )
        self.get_logger().info(f"Ball trajectory planner started. Listening to {ball_pose_robot_topic}")

        self.ball_position_in_cam = []
        self.ball_position_in_robot = []


    def pose_callback(self, msg: PoseStamped):
        position = msg.pose.position
        self.ball_position_in_robot.append((position.x, position.y, position.z))

    def display_plots(self):
        self.ball_position_in_robot = np.array(self.ball_position_in_robot, dtype=np.float32)

        print(self.ball_position_in_robot)

        # position
        x_pos = self.ball_position_in_robot[:, 0]
        y_pos = self.ball_position_in_robot[:, 1]
        z_pos = self.ball_position_in_robot[:, 2]
        self.plot_3D(x_pos, y_pos, z_pos, "Ball Trajectory in Robot Frame")


    def plot_3D(self, x, y, z, title): 
        # plt.style.use('_mpl-gallery')   

        # Plot
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(10, 7))
        ax.scatter(x, y, z)

        # identify origin
        ax.scatter([0], [0], [0], color='red', s=100, zorder=5)
        ax.text(0, 0, 0, '  Origin', color='red', fontsize=9)
        
        ax.grid(True)

        # Title
        ax.set_title(title, loc='center', y=1.01)

        # set axis labels
        ax.set_xlabel("x-axis (m)")
        ax.set_ylabel("y-axis (m)")
        ax.set_zlabel("z-axis (m)")
        
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = BallTrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.display_plots()
        node.destroy_node()
        rclpy.shutdown()
