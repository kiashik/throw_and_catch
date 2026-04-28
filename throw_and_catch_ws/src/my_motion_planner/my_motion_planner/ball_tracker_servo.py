"""
from kayla

Improvements:
    TODO : add logging of error 
"""

import numpy as np
from rclpy.time import Time
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_ros import Buffer, TransformListener

from sensor_msgs.msg import JointState
import pickle

class PIDController:
    """
    PID controller for position-based visual servoing.
    """
    def __init__(self, dt, p, i, d, dim=3):
        # Initialize gains 
        self.p = p
        self.i = i
        self.d = d
       # Initialize matrix gains
        self.Kp = self.p * np.eye(dim) # Proportional gain 
        self.Ki = self.i * np.eye(dim) # Integral gain 
        self.Kd = self.d * np.eye(dim) # Derivative gain
        # Initialize error terms
        self.error_integral = np.zeros(dim)
        self.error_prev = np.zeros(dim)
        self.dt = dt # Control period in seconds
   
    def compute(self, error):

        # proportional
        P = self.Kp @ error

        # integral
        self.error_integral += error * self.dt
        I = self.Ki @ self.error_integral

        # derivative
        D = self.Kd @ (error - self.error_prev) / self.dt
        
        PID = P + I + D

        self.error_prev = error
        
        return PID


class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker_servo')

        def get_float_param(name, default):
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            return float(default if value is None else value)

        self.dt = 0.005   # 200 Hz
        self.pid = PIDController(
            dt=self.dt,
            p=get_float_param('pid_p', 3.5),
            i=get_float_param('pid_i', 0.005),
            d=get_float_param('pid_d', 0.001),
        )
        self.max_linear_velocity = get_float_param('max_linear_velocity', 0.4) # m/s
        self.near_goal_tolerance = get_float_param('near_goal_tolerance', 0.01) # m


        self.ball_pos_base = None  # initialize empty ball pose
        self.last_ball_time = None
        self.hold_target = None     # last valid ball position
        self.full_stop_timeout = get_float_param('full_stop_timeout', 0.3)


        # Subscriber to real-time ball pose topic
        self.ball_pose_sub = self.create_subscription(
            PoseStamped,
            '/vision/ball_pose_robot',
            self.ball_callback,
            10
        )


        # Publisher to MoveIt Servo
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        # TF listener to get EE pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.base_frame = "link0"
        self.ee_frame = "end_effector_link" #TODO replace with EE frame

        # ---------------- data saving for performance visualization
        self.start_time = self.get_clock().now()
        self.t_data = []
        self.ee_position_data = []
        self.position_error_data = []
        self.PID_output_data = []
        self.q_dot_data = []

        # to log joint vel, need to sub to joint_states
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_cb,
            10
        )
        self.current_q_dot = None
        # ------------------------------

    def joint_states_cb(self, msg: JointState):
        self.current_q_dot = list(msg.velocity)

    def save_to_pickle(self, filename='/home/ashik/throw_and_catch/throw_and_catch/data_visualize/ball_tracker_data.pkl'):
        data = {
            't_data': self.t_data,
            'ee_position_data': np.array(self.ee_position_data),
            'position_error_data': np.array(self.position_error_data),
            'PID_output_data': np.array(self.PID_output_data),
            'q_dot_data': np.array(self.q_dot_data),
        }
        with open(filename, 'wb') as f:
            pickle.dump(data, f)
        self.get_logger().info(f'Data saved to {filename}')

    # real-time ball detection
    def ball_callback(self, msg: PoseStamped):
        self.ball_pos_base = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        self.last_ball_time = self.get_clock().now()

        # Update hold target whenever ball is visible
        self.hold_target = self.ball_pos_base.copy()

    def get_ee_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time(),
                timeout=RclpyDuration(seconds=0.5) # may or may not need
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            return np.array([x, y, z])

        except Exception:
            self.get_logger().warn("TF lookup failed")
            return None
       
    def control_loop(self):
        now = self.get_clock().now()

        # If we have never seen a ball
        if self.hold_target is None or self.last_ball_time is None:
            self.publish_zero_twist()
            return


        # Time since last seen
        time_since_seen = (now - self.last_ball_time).nanoseconds * 1e-9


        if time_since_seen > self.full_stop_timeout:
            # Ball gone for long time → fully stop
            self.publish_zero_twist()
            return


        # If short timeout passed, we HOLD last position
        target_position = self.hold_target


        ee_pos = self.get_ee_position()
        if ee_pos is None:
            self.get_logger().warn("No end-effector pose available")
            return

        error = target_position - ee_pos


        # Stop if very close
        if np.linalg.norm(error) < self.near_goal_tolerance:
            velocity = np.zeros(3)
        else:
            velocity = self.pid.compute(error)

        # Safety clamp (VERY IMPORTANT)
        velocity = np.clip(velocity, -self.max_linear_velocity, self.max_linear_velocity)


        # Publish twist
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.base_frame


        twist_msg.twist.linear.x = float(velocity[0])
        twist_msg.twist.linear.y = float(velocity[1])
        twist_msg.twist.linear.z = float(velocity[2])


        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0


        self.twist_pub.publish(twist_msg)

        # ---------- data saving for performance visualization
        elapsed_s = (now - self.start_time).nanoseconds * 1e-9      # times elapsed since start in seconds
        self.t_data.append(float(elapsed_s))
        self.ee_position_data.append(ee_pos)
        self.position_error_data.append(error)
        self.PID_output_data.append(velocity)
        self.q_dot_data.append(self.current_q_dot)
        # --------------------------------------------------


    def publish_zero_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        # all twist fields default to 0
        self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_to_pickle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

