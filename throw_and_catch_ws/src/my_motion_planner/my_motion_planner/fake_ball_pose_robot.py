""""
How to run:
    ros2 run my_motion_planner fake_ball_pose_robot

"""


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakeBallPoseRobot(Node):
    def __init__(self):
        super().__init__('fake_ball_pose_robot')

        pub_period = 1.0/100.0  # seconds
        self.publisher = self.create_publisher(PoseStamped, '/vision/ball_pose_robot', 10)
        self.timer = self.create_timer(pub_period, self.timer_callback)

        self.step_distance = 0.01
        self.position = [0.0, -0.4, 0.0]

        # self.waypoints = [
        #     [0.0, -0.4, 0.6],
        #     [-0.3, -0.4, 0.3],
        #     [0.3, -0.4, 0.3],
        # ]

        self.position = [0.0, -0.6, 0.0]
        self.waypoints = [
            [0.0, -0.6, 0.8],
            [-0.4, -0.6, 0.4],
            [0.4, -0.6, 0.4],
        ]
        
        self.current_index = 0

        self.plot_enabled = False
        self.plot_history = {'x': [], 'y': [], 'z': []}
        self._init_plot()

        self.get_logger().info(f'FakeBallPoseRobot started, publishing /vision/ball_pose_robot at {1 / pub_period:.2f} Hz')

    def _init_plot(self):
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

            plt.ion()
            self.plt = plt
            self.fig = plt.figure(figsize=(8, 6))
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_title('Fake Ball Robot Trajectory')
            self.ax.set_xlabel('X [m]')
            self.ax.set_ylabel('Y [m]')
            self.ax.set_zlabel('Z [m]')
            self.ax.set_xlim(-0.6, 0.6)
            self.ax.set_ylim(-0.8, -0.4)
            self.ax.set_zlim(0.0, 1.2)
            self.line, = self.ax.plot([], [], [], '-o', color='tab:blue', markersize=5)
            self.current_point = self.ax.scatter([], [], [], color='tab:red', s=40)
            self.plot_enabled = True
            self.plt.show(block=False)
        except ImportError:
            self.get_logger().warning('matplotlib is not installed; ball plotting disabled')
            self.plot_enabled = False

    def timer_callback(self):
        target = self.waypoints[self.current_index]
        delta = [target[i] - self.position[i] for i in range(3)]
        distance = sum(d * d for d in delta) ** 0.5

        if distance <= 1e-4:
            self._advance_waypoint()
            target = self.waypoints[self.current_index]
            delta = [target[i] - self.position[i] for i in range(3)]
            distance = sum(d * d for d in delta) ** 0.5

        if distance <= self.step_distance:
            self.position = target.copy()
            self._advance_waypoint()
        else:
            scale = self.step_distance / distance
            self.position = [
                self.position[i] + delta[i] * scale
                for i in range(3)
            ]

        self._publish_pose()
        self._update_plot()

    def _advance_waypoint(self):
        self.current_index = (self.current_index + 1) % len(self.waypoints)
        self.get_logger().info(
            f'Advancing to waypoint {self.current_index}: '
            f'{self.waypoints[self.current_index]}'
        )

    def _publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'link0'
        pose_msg.pose.position.x = float(self.position[0])
        pose_msg.pose.position.y = float(self.position[1])
        pose_msg.pose.position.z = float(self.position[2])
        pose_msg.pose.orientation.w = 1.0

        self.publisher.publish(pose_msg)

    def _update_plot(self):
        if not self.plot_enabled:
            return

        self.plot_history['x'].append(self.position[0])
        self.plot_history['y'].append(self.position[1])
        self.plot_history['z'].append(self.position[2])

        self.line.set_data(self.plot_history['x'], self.plot_history['y'])
        self.line.set_3d_properties(self.plot_history['z'])

        self.current_point._offsets3d = (
            [self.position[0]],
            [self.position[1]],
            [self.position[2]],
        )

        self.ax.set_title(
            f'Fake Ball Robot Trajectory - x={self.position[0]:.2f}, '
            f'y={self.position[1]:.2f}, z={self.position[2]:.2f}'
        )
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def destroy_node(self):
        self.get_logger().info('FakeBallPoseRobot shutting down')
        super().destroy_node()


def main():
    rclpy.init()
    node = FakeBallPoseRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
