#!/usr/bin/env python3


import math
import numpy as np
import os
import pickle
import pandas as pd
import pprint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


G = 9.81  # gravity


def predict_catch_point(p1, t1, p2, t2, catch_y, floor_z=0.0):
    x1, y1, z1 = p1
    x2, y2, z2 = p2

    dt = t2 - t1
    print(f"dt = {dt}")
    if dt < 0.02 or dt > 0.25:
        print(f"Reject: dt out of range ({dt:.4f})")
        return None

    vx = (x2 - x1) / dt
    vy = (y2 - y1) / dt
    vz = (z2 - z1) / dt + 0.5 * G * dt
    print(f"vx={vx}, vy={vy}, vz={vz}")

    speed = math.sqrt(vx*vx + vy*vy + vz*vz)
    if speed > 8.0:
        print(f"Reject: speed too large ({speed:.3f})")
        return None

    if abs(vy) < 0.02:
        print("Reject: vy too small")
        return None

    dy = catch_y - y2

    if dy * vy <= 0.0:
        print("Reject: ball not moving toward catch plane")
        return None

    time_to_catch = dy / vy
    print(f"time_to_catch={time_to_catch}")

    if time_to_catch <= 0.0:
        print("Reject: catch point is in the past")
        return None

    catch_x = x2 + vx * time_to_catch
    catch_z = z2 + vz * time_to_catch - 0.5 * G * (time_to_catch ** 2)
    print(f"catch_z={catch_z}")

    if catch_z < floor_z:
        print("Reject: below floor")
        return None

    return (catch_x, catch_y, catch_z, time_to_catch, vx, vy, vz)




class BallTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('ball_trajectory_planner')


        self.subscription = self.create_subscription(
            PoseStamped,
            '/ball_pose/pose',
            self.pose_callback,
            10
        )


        self.catch_y = -0.45


        # self.prev_point = None
        # self.prev_time = None


        #self.curr_point = None
        # self.curr_time = None


        # Store recent measured points for plotting
        self.history = []
        self.measurements = []  # list of (t, p)

         # New: store predictions that we will evaluate later
        self.pending_predictions = []

        # New: all resolved prediction-vs-actual comparisons
        self.comparison_data = []

        # New: pickle file name
        self.pickle_file = 'raw_predictions.pkl'
        


        # Set up live plotting
        plt.ion()
        self.fig = plt.figure(figsize=(6,6))


        # self.ax1 = self.fig.add_subplot(1, 3, 1)  # x-y view
        # self.ax2 = self.fig.add_subplot(1, 3, 2)  # x-z view
        self.ax3 = self.fig.add_subplot(111, projection='3d')  # 3D view

        plt.show(block=False)



        self.get_logger().info("Ball trajectory planner started. Listening to /vision/ball_pose_robot")


    def pose_callback(self, msg: PoseStamped):
        # Use ROS message timestamp if available
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


        p = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

        if self.measurements and (t - self.measurements[-1][0] > 0.5):
            self.get_logger().info("Long gap detected, resetting measurement history.")
            self.measurements.clear()
            self.history.clear()
            self.pending_predictions.clear()

        self.measurements.append((t, p))
        if len(self.measurements) > 200:
            self.measurements.pop(0)

        self.history.append(p)
        if len(self.history) > 50:
            self.history.pop(0)

        # New: try to resolve any older predictions whose target time has arrived
        self.resolve_predictions()

        
        if len(self.measurements) < 2:
            return

        # recent = self.measurements[-4:]
        # ys = [pt[1][1] for pt in recent]
        # dist_to_plane = [abs(y - self.catch_y) for y in ys]

        # improving_steps = sum(
        #     dist_to_plane[i+1] < dist_to_plane[i]
        #     for i in range(len(dist_to_plane) - 1)
        # )

        # print(f"ys = {ys}")
        # print(f"dist_to_plane = {dist_to_plane}")
        # print(f"improving_steps = {improving_steps}")

        # if improving_steps < 2:
        #     print("Reject: ball not reliably moving toward catch plane")
        #     return
            

        min_dt = 0.03
        older = None

        for t_old, p_old in reversed(self.measurements[:-1]):
            if t - t_old >= min_dt:
                older = (t_old, p_old)
                break

        if older is None:
            return

        t1, p1 = older
        t2, p2 = t, p

        result = predict_catch_point(
            p1, t1,
            p2, t2,
            self.catch_y
)


        if result is None:
            return


        catch_x, catch_y, catch_z, time_to_catch, vx, vy, vz = result


        self.get_logger().info(
            f"Predicted catch point -> x={catch_x:.3f}, y={catch_y:.3f}, "
            f"z={catch_z:.3f}, t={time_to_catch:.3f}s"
        )


        # New: store this prediction so we can compare it later
        predicted_time = t2 + time_to_catch
        self.pending_predictions.append({
            'prediction_time': t2,
            'target_time': predicted_time,
            'predicted_position': (catch_x, catch_y, catch_z),
            'source_position': p2,
            'time_to_catch': time_to_catch,
            'velocity': (vx, vy, vz)
        })


        self.update_plot(p2, vx, vy, vz, time_to_catch, (catch_x, catch_y, catch_z))
    
    def resolve_predictions(self):
        """
        For each stored prediction:
        once current measurement time has passed the prediction target time,
        estimate the actual ball position at that exact time and save the comparison.
        """
        if len(self.measurements) < 2:
            return

        current_time = self.measurements[-1][0]
        resolved = []

        for pred in self.pending_predictions:
            target_time = pred['target_time']

            # Wait until we have measurements beyond the prediction's target time
            if current_time < target_time:
                continue

            actual_position = self.interpolate_position(target_time)
            if actual_position is None:
                continue

            px, py, pz = pred['predicted_position']
            ax, ay, az = actual_position

            record = {
                'prediction_time': pred['prediction_time'],
                'target_time': pred['target_time'],
                'time_to_catch': pred['time_to_catch'],
                'predicted_position': (px, py, pz),
                'actual_position': (ax, ay, az),
                'error': (ax - px, ay - py, az - pz),
                'source_position': pred['source_position'],
                'velocity': pred['velocity']
            }

            self.comparison_data.append(record)
            self.append_to_pickle(record)

            self.get_logger().info(
                f"Saved prediction comparison -> "
                f"pred=({px:.3f}, {py:.3f}, {pz:.3f}), "
                f"actual=({ax:.3f}, {ay:.3f}, {az:.3f}), "
                f"error=({ax - px:.3f}, {ay - py:.3f}, {az - pz:.3f})"
            )

            resolved.append(pred)

        for pred in resolved:
            self.pending_predictions.remove(pred)

    def interpolate_position(self, target_time):
        """
        Linearly interpolate the ball position at target_time using
        the two surrounding measurements.
        """
        for i in range(len(self.measurements) - 1):
            t0, p0 = self.measurements[i]
            t1, p1 = self.measurements[i + 1]

            if t0 <= target_time <= t1:
                if t1 == t0:
                    return p0

                alpha = (target_time - t0) / (t1 - t0)

                x = p0[0] + alpha * (p1[0] - p0[0])
                y = p0[1] + alpha * (p1[1] - p0[1])
                z = p0[2] + alpha * (p1[2] - p0[2])

                return (x, y, z)

        return None

    def append_to_pickle(self, record):
        """
        Append each resolved prediction comparison to a pickle file.
        """
        if os.path.exists(self.pickle_file):
            with open(self.pickle_file, 'rb') as f:
                data = pickle.load(f)
        else:
            data = []

        data.append(record)

        with open(self.pickle_file, 'wb') as f:
            pickle.dump(data, f)


    def update_plot(self, current_point, vx, vy, vz, time_to_catch, catch_point):
        x0, y0, z0 = current_point
        cx, cy, cz = catch_point

        # Generate projected trajectory from current point to catch point
        t_vals = np.linspace(0.0, time_to_catch, 100)
        x_vals = x0 + vx * t_vals
        y_vals = y0 + vy * t_vals
        z_vals = z0 + vz * t_vals - 0.5 * G * (t_vals ** 2)

        # self.ax1.clear()
        # self.ax2.clear()
        self.ax3.clear()

        # Measured history
        if len(self.history) > 0:
            hist = np.array(self.history)
            # self.ax1.plot(hist[:, 0], hist[:, 1], 'bo-', label='Measured ball path')
            # self.ax2.plot(hist[:, 0], hist[:, 2], 'bo-', label='Measured ball path')
            self.ax3.plot(hist[:, 0], hist[:, 1], hist[:, 2], 'bo-', label='Measured ball path')

        # Predicted path
        # self.ax1.plot(x_vals, y_vals, 'r--', label='Predicted trajectory')
        # self.ax2.plot(x_vals, z_vals, 'r--', label='Predicted trajectory')
        self.ax3.plot(x_vals, y_vals, z_vals, 'r--', label='Predicted trajectory')

        # Current point
        # self.ax1.plot(x0, y0, 'go', markersize=8, label='Current ball position')
        # self.ax2.plot(x0, z0, 'go', markersize=8, label='Current ball position')
        self.ax3.plot([x0], [y0], [z0], 'go', markersize=8, label='Current ball position')

        # Catch point
        # self.ax1.plot(cx, cy, 'ms', markersize=8, label='Predicted catch point')
        # self.ax2.plot(cx, cz, 'ms', markersize=8, label='Predicted catch point')
        self.ax3.plot([cx], [cy], [cz], 'ms', markersize=8, label='Predicted catch point')

        # Catch plane line in XY
        # self.ax1.axhline(self.catch_y, color='k', linestyle=':', label='Catch plane')

        # Optional: draw catch plane in 3D
        x_plane = np.linspace(min(x_vals.min(), x0, cx) - 0.1, max(x_vals.max(), x0, cx) + 0.1, 2)
        z_plane = np.linspace(min(z_vals.min(), z0, cz, 0.0) - 0.1, max(z_vals.max(), z0, cz) + 0.1, 2)
        Xp, Zp = np.meshgrid(x_plane, z_plane)
        Yp = np.full_like(Xp, self.catch_y)
        self.ax3.plot_surface(Xp, Yp, Zp, alpha=0.2)

        # self.ax1.set_title('Top View: X-Y')
        # self.ax1.set_xlabel('X')
        # self.ax1.set_ylabel('Y')
        # self.ax1.grid(True)
        # self.ax1.legend()

        # self.ax2.set_title('Side View: X-Z')
        # self.ax2.set_xlabel('X')
        # self.ax2.set_ylabel('Z')
        # self.ax2.grid(True)
        # self.ax2.legend()

        self.ax3.set_title('3D Trajectory View')
        self.ax3.set_xlabel('X')
        self.ax3.set_ylabel('Y')
        self.ax3.set_zlabel('Z')
        self.ax3.legend()

            # IMPORTANT: keep axes from jumping wildly
        self.ax3.set_xlim(-1, 1)
        self.ax3.set_ylim(-1, 1)
        self.ax3.set_zlim(0, 1.5)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # plt.tight_layout()
        # plt.draw()
        # plt.pause(0.001)


   

    with open('raw_predictions.pkl', 'rb') as f:
        data = pickle.load(f)

    # Convert to DataFrame and save
    # df = pd.DataFrame(data)
    # df.to_csv('raw_predictions.csv', index=False)

    with open('raw_predictions.txt', 'w') as f: 
        f.write(pprint.pformat(data))



def main(args=None):
    rclpy.init(args=args)
    node = BallTrajectoryPlanner()


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


    node.destroy_node()
    rclpy.shutdown()
    plt.ioff()
    plt.show()



if __name__ == '__main__':
    main()






