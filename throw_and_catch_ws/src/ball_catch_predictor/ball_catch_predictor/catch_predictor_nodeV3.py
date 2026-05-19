#!/usr/bin/env python3
"""
ROS 2 Jazzy ball catch-point predictor, V3.

Subscribes:
  /vision/ball_pose_robot    geometry_msgs/msg/PoseStamped

Publishes:
  /vision/catch_point        geometry_msgs/msg/PoseStamped
  /vision/ball_prediction_markers visualization_msgs/msg/MarkerArray

State:
  x = [px, py, pz, vx, vy, vz].T

Observation:
  z = [px, py, pz].T

Model after release:
  dp/dt = v
  dv/dt = -drag_coeff * v + [0, 0, -gravity]

The node uses a linear Kalman filter because the linear-drag dynamics are
linear/affine. The catch point is the predicted intersection with the bounded
catch plane:

  y = -0.40 m
  -0.30 <= x <= 0.40 m
   0.00 <= z <= 0.8575 m

Plot/CSV validation can be saved at toss end or deferred until Ctrl+C.
"""

from __future__ import annotations

import csv
import math
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
# from pyautogui import position

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time

from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


# Tunable defaults requested for this project. These are also exposed as ROS
# parameters so they can be changed from a launch file or command line.
DEFAULT_BALL_RADIUS_M = 0.03
DEFAULT_GRAVITY_MPS2 = 9.79728 # 9.80665
DEFAULT_DRAG_COEFF_1PS = 0.05
DEFAULT_CATCH_Y_M = -0.450
DEFAULT_CATCH_X_MIN_M = -0.30
DEFAULT_CATCH_X_MAX_M = 0.40
DEFAULT_CATCH_Z_MIN_M = 0.00
DEFAULT_CATCH_Z_MAX_M = 0.8575

# Sensor characterization. The vector below is the measurement bias, i.e.
# measured mean - ground truth. Incoming raw positions are corrected by
# z_corrected = z_raw - bias.
DEFAULT_MEASUREMENT_BIAS_M = np.array(
    [0.0061522260, -0.0082099440, -0.0222341760], dtype=float
)
DEFAULT_MEASUREMENT_VARIANCE_M2 = np.array(
    [0.0000009724, 0.0000013000, 0.0000217632], dtype=float
)


@dataclass
class MeasurementRecord:        # Stores all measurements for the current toss only.
    t: float
    receive_t: float
    raw: np.ndarray
    corrected: np.ndarray
    filtered: np.ndarray


@dataclass
class PredictionRecord:     # Stores every predicted catch point for the current toss.
    toss_id: int
    source: str
    t_pub: float
    t_catch: float
    lead_time: float
    catch_pos: np.ndarray
    state_pos: np.ndarray
    state_vel: np.ndarray
    in_plane_bounds: bool
    published: bool


@dataclass
class ActualCrossing:        # Stores the actual crossing point of the ball with the catch plane.
    t: float
    pos: np.ndarray
    method: str


@dataclass
class TossData:
    toss_id: int
    measurements: List[MeasurementRecord]
    predictions: List[PredictionRecord]
    rejected_measurements: int       # Number of measurements rejected as outliers for this toss.
    catch_y: float
    catch_x_min: float
    catch_x_max: float
    catch_z_min: float
    catch_z_max: float


class LinearDragKalmanFilter:
    """Small, fast Kalman filter for 3D ball position/velocity."""

    def __init__(
        self,
        gravity: float,
        drag_coeff: float,
        measurement_variance: np.ndarray,
        process_accel_std: float,
        initial_velocity_std: float,
    ) -> None:
        self.gravity = float(gravity)
        self.drag_coeff = float(drag_coeff)
        self.measurement_variance = np.maximum(measurement_variance.astype(float), 1e-12)
        
        # Kalman filter process noise. how much to trust motion model. 
        # higher value: 
        # filter adapts faster, velocity estimate changes more easily
        # trajectory may become noisier'
        # lower value: filter trusts the physics model more
        # smoother estimate
        # may lag if the model is imperfect
        self.process_accel_std = float(process_accel_std)           

        # Initial uncertainty in velocity when the ball is first detected.
        # picking a large value cuz The initial velocity value is probably wrong. want KF to learn it quickly from measurements.

        self.initial_velocity_std = float(initial_velocity_std)             
        self.x = np.zeros(6, dtype=float)       #  (6x1) state vector: [px, py, pz, vx, vy, vz]
        self.P = np.eye(6, dtype=float)         # (6x6) state covariance matrix
        self.R = np.diag(self.measurement_variance) # (3x3) measurement noise covariance (uses our camera noise characterization)
        self.H = np.zeros((3, 6), dtype=float)      # (3x6) measurement matrix maps state to measurement space [I_3x3, 0_3x3]. z = H @ x
        self.H[:, 0:3] = np.eye(3)
        self.I = np.eye(6, dtype=float)
        self.initialized = False

    @property
    def accel(self) -> np.ndarray:
        return np.array([0.0, 0.0, -self.gravity], dtype=float)

    def reset(self) -> None:
        self.x[:] = 0.0
        self.P[:] = np.eye(6)
        self.initialized = False

    def initialize(self, position: np.ndarray) -> None:
        self.x[:] = 0.0
        self.x[0:3] = position
        pos_var = np.maximum(self.measurement_variance, 1e-10)
        vel_var = np.ones(3, dtype=float) * (self.initial_velocity_std ** 2)
        self.P = np.diag(np.concatenate([pos_var, vel_var]))
        self.initialized = True

    def transition(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Return discrete transition Phi and offset b for x_next = Phi*x + b."""
        dt = max(0.0, float(dt))
        k = self.drag_coeff
        Phi = np.eye(6, dtype=float)
        b = np.zeros(6, dtype=float)
        a = self.accel

        if dt <= 0.0:
            return Phi, b

        if abs(k) < 1e-12:
            Phi[0:3, 3:6] = dt * np.eye(3)
            b[0:3] = 0.5 * a * dt * dt
            b[3:6] = a * dt
            return Phi, b

        e = math.exp(-k * dt)           # Velocity decay factor due to linear drag over time dt.
        b_v = (1.0 - e) / k         # Coefficient that maps velocity into position change under linear drag. if drag is 0, this limit to dt, consistent with no drag case.
        b_p = dt / k - (1.0 - e) / (k * k)  # Coefficient that maps constant acceleration into position change under linear drag.

        # b[0:3] = gravity contribution to position
        # b[3:6] = gravity contribution to velocity
        Phi[0:3, 3:6] = b_v * np.eye(3)     #  (6x6) state transition matrix for linear drag model. It maps the current state to the next state, excluding the constant gravity offset.
        Phi[3:6, 3:6] = e * np.eye(3)
        b[0:3] = b_p * a
        b[3:6] = b_v * a
        return Phi, b

    def predict(self, dt: float) -> None:
        if not self.initialized:
            return

        dt = max(0.0, float(dt))
        Phi, b = self.transition(dt)
        self.x = Phi @ self.x + b

        # Approximate continuous white acceleration noise. This lets the filter
        # absorb imperfect drag, small spin effects, and residual measurement
        # systematics without making the state too jittery.
        q = self.process_accel_std ** 2     # process acceleration variance.
        Q = np.zeros((6, 6), dtype=float)
        Q[0:3, 0:3] = (dt ** 4 / 4.0) * q * np.eye(3)
        Q[0:3, 3:6] = (dt ** 3 / 2.0) * q * np.eye(3)
        Q[3:6, 0:3] = Q[0:3, 3:6].T
        Q[3:6, 3:6] = (dt ** 2) * q * np.eye(3)

        self.P = Phi @ self.P @ Phi.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def update(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Kalman position update. Returns innovation and innovation covariance.
        z is the measurement vector [px, py, pz].T
        """
        z = z.astype(float)
        innovation = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R

        # K = P H.T inv(S), solved without explicitly inverting S.
        K = np.linalg.solve(S.T, (self.P @ self.H.T).T).T
        self.x = self.x + K @ innovation

        # Joseph form improves numerical stability for small covariances.
        KH = K @ self.H
        self.P = (self.I - KH) @ self.P @ (self.I - KH).T + K @ self.R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return innovation, S

    def propagate_state(self, state: np.ndarray, dt: float) -> np.ndarray:
        Phi, b = self.transition(dt)
        return Phi @ state + b


class BallCatchPredictorNode(Node):
    def __init__(self) -> None:
        super().__init__('ball_catch_predictor')

        self._declare_parameters()
        self._load_parameters()

        self.kf = LinearDragKalmanFilter(
            gravity=self.gravity,
            drag_coeff=self.drag_coeff,
            measurement_variance=self.measurement_variance,
            process_accel_std=self.process_accel_std,
            initial_velocity_std=self.initial_velocity_std,
        )

        catch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(depth=1)

        self.catch_pub = self.create_publisher(PoseStamped, self.catch_topic, catch_qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)
        self.sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )

        self.prediction_timer = self.create_timer(1.0 / self.prediction_rate_hz, self.timer_callback)

        self.state_time_s: Optional[float] = None       # Timestamp corresponding to the current Kalman filter state. this comes from image capture time from ball pose msgs.
        self.last_receive_time_s: Optional[float] = None    #  ROS timestamp when last pose msg was recived. used to detect toss timeout, measruemnt gaps.
        self.last_msg_sample_time_s: Optional[float] = None
        self.last_frame_id: str = self.default_frame_id
        self.using_header_time_basis = False        # if true, the node is using the header stamp as the time basis for prediction. if false, it uses the receive time of the message. 
        self.last_latency_log_s = -1e9
        self.last_marker_pub_s = -1e9

        self.toss_id = 0
        self.toss_active = False
        self.measurements: List[MeasurementRecord] = []
        self.predictions: List[PredictionRecord] = []
        self.rejected_measurements = 0
        self.deferred_tosses: List[TossData] = []       # List of completed tosses waiting to be plotted. used when plot_after_shutdown_only is true.
        self.plot_threads: List[threading.Thread] = []

        if self.save_plots and not self.plot_after_shutdown_only:
            self._try_make_plot_dir()

        self._log_startup_summary()

    def _declare_parameters(self) -> None:
        # Topics and frame behavior.
        self.declare_parameter('input_topic', '/vision/ball_pose_robot')
        self.declare_parameter('catch_topic', '/vision/catch_point')
        self.declare_parameter('marker_topic', '/vision/ball_prediction_markers')
        self.declare_parameter('default_frame_id', 'link0')
        self.declare_parameter('stamp_catch_pose_with_arrival_time', True)

        # Physics and catch-plane constants.
        self.declare_parameter('ball_radius_m', DEFAULT_BALL_RADIUS_M)
        self.declare_parameter('catch_y', DEFAULT_CATCH_Y_M)
        self.declare_parameter('catch_x_min', DEFAULT_CATCH_X_MIN_M)
        self.declare_parameter('catch_x_max', DEFAULT_CATCH_X_MAX_M)
        self.declare_parameter('catch_z_min', DEFAULT_CATCH_Z_MIN_M)
        self.declare_parameter('catch_z_max', DEFAULT_CATCH_Z_MAX_M)
        self.declare_parameter('require_catch_point_inside_plane', True)
        self.declare_parameter('gravity', DEFAULT_GRAVITY_MPS2)
        self.declare_parameter('drag_coeff', DEFAULT_DRAG_COEFF_1PS)

        # Prediction settings.
        self.declare_parameter('prediction_horizon_s', 1.50)            # Maximum future time that the node will search for a catch-plane intersection. If the ball will not reach y = catch_y within this time, no catch point is produced.
        self.declare_parameter('prediction_rate_hz', 50.0)
        self.declare_parameter('publish_on_measurement', True)
        self.declare_parameter('trajectory_dt_s', 0.02)                 # Step size used to generate the RViz predicted trajectory line. Smaller value: smoother RViz line, more CPU. num of points in rviz = prediction_horizon_s / trajectory_dt_s

        # Sensor characterization and filter tuning.
        self.declare_parameter('apply_measurement_bias_correction', True)
        self.declare_parameter('measurement_bias_x_m', float(DEFAULT_MEASUREMENT_BIAS_M[0]))
        self.declare_parameter('measurement_bias_y_m', float(DEFAULT_MEASUREMENT_BIAS_M[1]))
        self.declare_parameter('measurement_bias_z_m', float(DEFAULT_MEASUREMENT_BIAS_M[2]))
        self.declare_parameter('measurement_variance_x_m2', float(DEFAULT_MEASUREMENT_VARIANCE_M2[0]))
        self.declare_parameter('measurement_variance_y_m2', float(DEFAULT_MEASUREMENT_VARIANCE_M2[1]))
        self.declare_parameter('measurement_variance_z_m2', float(DEFAULT_MEASUREMENT_VARIANCE_M2[2]))
        self.declare_parameter('process_accel_std_mps2', 2.0)
        self.declare_parameter('initial_velocity_std_mps', 5.0)
        self.declare_parameter('gate_threshold_m', 0.30)        # Outlier rejection threshold. If the measurement is more than this distance from the predicted position, it is rejected and does not update the filter. This prevents one bad vision detection from corrupting the velocity estimate.

        # Timing robustness. The input PoseStamped is expected to be stamped
        # with image-capture time in the same ROS clock basis as this node.
        self.declare_parameter('use_header_stamp', True)                # ALWAYS TRUE
        self.declare_parameter('max_reasonable_latency_s', 2.0)         # DO NOT NEED
        self.declare_parameter('future_stamp_tolerance_s', 0.10)        

        # Maximum allowed Kalman filter time step between measurements. If the time between measurements is larger than this, the node starts a new toss/filter track instead of trying to continue the old one.
        self.declare_parameter('max_filter_dt_s', 0.20)
        self.declare_parameter('toss_timeout_s', 0.35)

        # RViz visualization. Marker publication is throttled separately from
        # catch-point publication so RViz cannot slow down prediction.
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_publish_rate_hz', 30.0)

        # Plotting/logging. By default plots are queued and written on Ctrl+C so
        # normal prediction does not spend CPU on matplotlib. Set
        # plot_after_shutdown_only=false to save after each toss in a background
        # thread, or save_plots=false to disable all CSV/PNG output.
        self.declare_parameter('save_plots', True)
        self.declare_parameter('plot_after_shutdown_only', True)
        self.declare_parameter('save_plots_async', True)
        self.declare_parameter('plot_dir', '~/ball_catch_plots_V3')
        self.declare_parameter('max_deferred_tosses', 2)

    def _load_parameters(self) -> None:
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.catch_topic = str(self.get_parameter('catch_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.default_frame_id = str(self.get_parameter('default_frame_id').value)
        self.stamp_catch_pose_with_arrival_time = bool(
            self.get_parameter('stamp_catch_pose_with_arrival_time').value
        )

        self.ball_radius_m = max(0.0, float(self.get_parameter('ball_radius_m').value))
        self.catch_y = float(self.get_parameter('catch_y').value)
        self.catch_x_min = float(self.get_parameter('catch_x_min').value)
        self.catch_x_max = float(self.get_parameter('catch_x_max').value)
        self.catch_z_min = float(self.get_parameter('catch_z_min').value)
        self.catch_z_max = float(self.get_parameter('catch_z_max').value)
        self.require_catch_point_inside_plane = bool(
            self.get_parameter('require_catch_point_inside_plane').value
        )
        self.gravity = float(self.get_parameter('gravity').value)
        self.drag_coeff = float(self.get_parameter('drag_coeff').value)

        self.prediction_horizon_s = max(0.01, float(self.get_parameter('prediction_horizon_s').value))
        self.prediction_rate_hz = max(8.0, float(self.get_parameter('prediction_rate_hz').value))
        self.publish_on_measurement = bool(self.get_parameter('publish_on_measurement').value)
        self.trajectory_dt_s = max(0.001, float(self.get_parameter('trajectory_dt_s').value))

        self.apply_measurement_bias_correction = bool(
            self.get_parameter('apply_measurement_bias_correction').value
        )
        self.measurement_bias = np.array([
            float(self.get_parameter('measurement_bias_x_m').value),
            float(self.get_parameter('measurement_bias_y_m').value),
            float(self.get_parameter('measurement_bias_z_m').value),
        ], dtype=float)
        self.measurement_variance = np.array([
            float(self.get_parameter('measurement_variance_x_m2').value),
            float(self.get_parameter('measurement_variance_y_m2').value),
            float(self.get_parameter('measurement_variance_z_m2').value),
        ], dtype=float)
        self.measurement_variance = np.maximum(self.measurement_variance, 1e-12)
        self.measurement_std = np.sqrt(self.measurement_variance)
        self.process_accel_std = float(self.get_parameter('process_accel_std_mps2').value)
        self.initial_velocity_std = float(self.get_parameter('initial_velocity_std_mps').value)
        self.gate_threshold_m = float(self.get_parameter('gate_threshold_m').value)

        self.use_header_stamp = bool(self.get_parameter('use_header_stamp').value)
        self.max_reasonable_latency_s = float(self.get_parameter('max_reasonable_latency_s').value)
        self.future_stamp_tolerance_s = float(self.get_parameter('future_stamp_tolerance_s').value)
        self.max_filter_dt_s = float(self.get_parameter('max_filter_dt_s').value)
        self.toss_timeout_s = float(self.get_parameter('toss_timeout_s').value)

        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        self.marker_publish_rate_hz = float(self.get_parameter('marker_publish_rate_hz').value)
        self.marker_min_period_s = 0.0
        if self.marker_publish_rate_hz > 0.0:
            self.marker_min_period_s = 1.0 / self.marker_publish_rate_hz

        self.save_plots = bool(self.get_parameter('save_plots').value)
        self.plot_after_shutdown_only = bool(self.get_parameter('plot_after_shutdown_only').value)
        self.save_plots_async = bool(self.get_parameter('save_plots_async').value)
        self.plot_dir = Path(os.path.expanduser(str(self.get_parameter('plot_dir').value)))
        self.max_deferred_tosses = max(1, int(self.get_parameter('max_deferred_tosses').value))

        if self.catch_x_min > self.catch_x_max:
            self.catch_x_min, self.catch_x_max = self.catch_x_max, self.catch_x_min
        if self.catch_z_min > self.catch_z_max:
            self.catch_z_min, self.catch_z_max = self.catch_z_max, self.catch_z_min

    def _try_make_plot_dir(self) -> None:
        try:
            self.plot_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Could not create plot_dir={self.plot_dir}: {exc}')

    def _log_startup_summary(self) -> None:
        self.get_logger().info(
            f'Subscribing to {self.input_topic}; publishing catch points to {self.catch_topic} '
            f'at timer rate {self.prediction_rate_hz:.1f} Hz plus measurement callbacks.'
        )
        self.get_logger().info(
            f'Model: state=[px,py,pz,vx,vy,vz], linear drag k={self.drag_coeff:.4f} 1/s, '
            f'gravity={self.gravity:.5f} m/s^2, ball_radius={self.ball_radius_m:.3f} m.'
        )
        self.get_logger().info(
            f'Catch plane: y={self.catch_y:.3f} m, '
            f'x=[{self.catch_x_min:.3f}, {self.catch_x_max:.3f}] m, '
            f'z=[{self.catch_z_min:.3f}, {self.catch_z_max:.3f}] m, '
            f'require_inside={self.require_catch_point_inside_plane}.'
        )
        self.get_logger().info(
            'Measurement bias [m]='
            f'({self.measurement_bias[0]:.8f}, {self.measurement_bias[1]:.8f}, '
            f'{self.measurement_bias[2]:.8f}), correction enabled='
            f'{self.apply_measurement_bias_correction}; variance [m^2]='
            f'({self.measurement_variance[0]:.8e}, {self.measurement_variance[1]:.8e}, '
            f'{self.measurement_variance[2]:.8e}).'
        )
        if self.save_plots:
            mode = 'deferred_until_shutdown' if self.plot_after_shutdown_only else 'after_each_toss'
            self.get_logger().info(f'Plot/CSV output enabled: mode={mode}, directory={self.plot_dir}')
        else:
            self.get_logger().info('Plot/CSV output disabled because save_plots=false.')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def header_stamp_to_s(msg: PoseStamped) -> Optional[float]:
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
        if sec == 0 and nanosec == 0:
            return None
        return sec + nanosec * 1e-9

    def choose_sample_time(self, msg: PoseStamped, receive_t: float) -> Tuple[float, bool, Optional[float]]:
        """Choose a usable sample timestamp.

        The expected case is that msg.header.stamp is image-capture time using
        the same ROS clock as this node. If the stamp is missing or clearly from
        a different time basis, fall back to receive time so prediction remains
        monotonic and stable.
        """
        header_t = self.header_stamp_to_s(msg)
        if not self.use_header_stamp or header_t is None:
            return receive_t, False, None

        latency = receive_t - header_t
        same_time_basis = (
            latency <= self.max_reasonable_latency_s
            and latency >= -self.future_stamp_tolerance_s
        )
        if same_time_basis:
            return header_t, True, latency
        return receive_t, False, latency

    def pose_callback(self, msg: PoseStamped) -> None:
        receive_t = self.now_s()
        z_raw = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)

        if not np.all(np.isfinite(z_raw)):
            self.get_logger().warn('Ignoring ball pose with non-finite position.')
            return

        # Correct the static sensor bias before filtering/prediction.
        # bias = measured_mean - ground_truth, so corrected = raw - bias.
        if self.apply_measurement_bias_correction:
            z = z_raw - self.measurement_bias
        else:
            z = z_raw.copy()

        frame_id = msg.header.frame_id if msg.header.frame_id else self.default_frame_id
        self.last_frame_id = frame_id

        sample_t, used_header, latency = self.choose_sample_time(msg, receive_t)
        self.using_header_time_basis = used_header

        if latency is not None and receive_t - self.last_latency_log_s > 2.0:
            self.last_latency_log_s = receive_t
            if used_header:
                self.get_logger().info(f'Pose header latency estimate: {latency * 1000.0:.1f} ms')
            else:
                self.get_logger().warn(
                    f'Pose header stamp not usable as node-clock time '
                    f'(latency estimate {latency:.3f} s); using receive time.'
                )

        # If a new message arrives long after the previous one, finish/reset the previous toss.
        if self.toss_active and self.last_receive_time_s is not None:
            if receive_t - self.last_receive_time_s > self.toss_timeout_s:
                self.finish_toss(reason='measurement_gap')
                self.kf.reset()
                self.state_time_s = None
                self.last_msg_sample_time_s = None

        if not self.kf.initialized:
            self.start_new_toss_if_needed()
            self.kf.initialize(z)
            self.state_time_s = sample_t
            self.last_msg_sample_time_s = sample_t
            self.last_receive_time_s = receive_t
            self.measurements.append(
                MeasurementRecord(sample_t, receive_t, z_raw.copy(), z.copy(), self.kf.x[0:3].copy())
            )
            if self.publish_on_measurement:
                self.publish_prediction(receive_t, source='measurement_init')
            return

        if self.state_time_s is None:
            self.state_time_s = sample_t

        dt = sample_t - self.state_time_s
        if dt <= 0.0:
            # Repeated or out-of-order stamp. Receive-time fallback usually fixes bad stamps.
            receive_dt = 0.0 if self.last_receive_time_s is None else receive_t - self.last_receive_time_s
            if receive_dt > 0.0 and receive_dt <= self.max_filter_dt_s:
                dt = receive_dt
                sample_t = self.state_time_s + receive_dt
            else:
                self.get_logger().warn(f'Ignoring out-of-order ball pose, dt={dt:.4f} s.')
                return

        if dt > self.max_filter_dt_s:
            self.get_logger().warn(
                f'Large measurement gap dt={dt:.3f} s. Starting a new toss/filter track.'
            )
            self.finish_toss(reason='large_filter_dt')
            self.kf.reset()
            self.start_new_toss_if_needed(force=True)
            self.kf.initialize(z)
            self.state_time_s = sample_t
            self.last_msg_sample_time_s = sample_t
            self.last_receive_time_s = receive_t
            self.measurements.append(
                MeasurementRecord(sample_t, receive_t, z_raw.copy(), z.copy(), self.kf.x[0:3].copy())
            )
            if self.publish_on_measurement:
                self.publish_prediction(receive_t, source='measurement_reinit')
            return

        self.kf.predict(dt)
        self.state_time_s = sample_t

        innovation = z - self.kf.x[0:3]
        innovation_norm = float(np.linalg.norm(innovation))
        if innovation_norm > self.gate_threshold_m:
            self.rejected_measurements += 1
            self.get_logger().warn(
                f'Rejected outlier pose. innovation={innovation_norm:.3f} m > '
                f'{self.gate_threshold_m:.3f} m'
            )
        else:
            self.kf.update(z)

        self.last_msg_sample_time_s = sample_t
        self.last_receive_time_s = receive_t
        self.measurements.append(
            MeasurementRecord(sample_t, receive_t, z_raw.copy(), z.copy(), self.kf.x[0:3].copy())
        )

        if self.publish_on_measurement:
            self.publish_prediction(receive_t, source='measurement')

    def timer_callback(self) -> None:
        now_t = self.now_s()
        if self.toss_active and self.last_receive_time_s is not None:
            if now_t - self.last_receive_time_s > self.toss_timeout_s:
                self.finish_toss(reason='timeout')
                self.kf.reset()
                self.state_time_s = None
                self.last_msg_sample_time_s = None
                self.publish_delete_markers()
                return

        self.publish_prediction(now_t, source='timer')

    def start_new_toss_if_needed(self, force: bool = False) -> None:
        if force or not self.toss_active:
            self.toss_id += 1
            self.toss_active = True
            self.measurements = []
            self.predictions = []
            self.rejected_measurements = 0
            self.get_logger().info(f'Started toss {self.toss_id}.')

    def current_state_at(self, query_t: float) -> Optional[np.ndarray]:
        if not self.kf.initialized or self.state_time_s is None:
            return None
        dt = query_t - self.state_time_s
        # If query_t is receive time and the filter state is image-capture time,
        # this propagation compensates for perception latency.
        if dt < -1e-3:
            dt = 0.0
        elif dt < 0.0:
            dt = 0.0
        if dt > self.prediction_horizon_s:
            return None
        return self.kf.propagate_state(self.kf.x.copy(), dt)

    def solve_time_to_y_plane(self, state: np.ndarray, y_target: float) -> Optional[float]:
        y0 = float(state[1])
        vy = float(state[4])
        delta = float(y_target - y0)
        k = self.drag_coeff

        if abs(delta) < 1e-9:
            return 0.0

        if abs(k) < 1e-12:
            if abs(vy) < 1e-12:
                return None
            t = delta / vy
            if 0.0 <= t <= self.prediction_horizon_s:
                return t
            return None

        if abs(vy) < 1e-12:
            return None

        # y(t) = y0 + vy/k * (1 - exp(-k t)).
        arg = 1.0 - (k * delta / vy)
        if arg <= 0.0:
            return None
        t = -math.log(arg) / k
        if 0.0 <= t <= self.prediction_horizon_s and math.isfinite(t):
            return t
        return None

    def find_y_plane_intersection(self, state: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
        t_catch = self.solve_time_to_y_plane(state, self.catch_y)
        if t_catch is None:
            return None
        catch_state = self.kf.propagate_state(state, t_catch)
        catch_pos = catch_state[0:3].copy()
        catch_pos[1] = self.catch_y
        return catch_pos, t_catch

    def is_inside_catch_plane_bounds(self, pos: np.ndarray) -> bool:
        x = float(pos[0])
        z = float(pos[2])
        return (
            self.catch_x_min <= x <= self.catch_x_max
            and self.catch_z_min <= z <= self.catch_z_max
        )

    def should_publish_marker_now(self, query_t: float) -> bool:
        if not self.publish_markers:
            return False
        if self.marker_min_period_s <= 0.0:
            return True
        return query_t - self.last_marker_pub_s >= self.marker_min_period_s

    def publish_prediction(self, query_t: float, source: str) -> None:
        state = self.current_state_at(query_t)
        if state is None:
            return

        result = self.find_y_plane_intersection(state)
        catch_pos: Optional[np.ndarray] = None
        published = False
        in_bounds = False
        lead_time = 0.0
        catch_abs_t = query_t

        if result is not None:
            catch_pos, lead_time = result
            catch_abs_t = query_t + lead_time
            in_bounds = self.is_inside_catch_plane_bounds(catch_pos)
            published = in_bounds or not self.require_catch_point_inside_plane

            if published:
                msg = PoseStamped()
                msg.header.frame_id = self.last_frame_id or self.default_frame_id
                if self.stamp_catch_pose_with_arrival_time:
                    msg.header.stamp = Time(nanoseconds=int(catch_abs_t * 1e9)).to_msg()
                else:
                    msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.x = float(catch_pos[0])
                msg.pose.position.y = float(catch_pos[1])
                msg.pose.position.z = float(catch_pos[2])
                msg.pose.orientation.w = 1.0
                self.catch_pub.publish(msg)

            if self.toss_active:
                self.predictions.append(
                    PredictionRecord(
                        toss_id=self.toss_id,
                        source=source,
                        t_pub=query_t,
                        t_catch=catch_abs_t,
                        lead_time=lead_time,
                        catch_pos=catch_pos.copy(),
                        state_pos=state[0:3].copy(),
                        state_vel=state[3:6].copy(),
                        in_plane_bounds=in_bounds,
                        published=published,
                    )
                )

        if self.should_publish_marker_now(query_t):
            valid_marker_pos = catch_pos if (catch_pos is not None and published) else None
            self.publish_future_markers(state, valid_marker_pos, query_t)
            self.last_marker_pub_s = query_t

    def future_trajectory_points(self, state: np.ndarray) -> List[Point]:
        points: List[Point] = []
        steps = max(2, int(self.prediction_horizon_s / self.trajectory_dt_s) + 1)
        for i in range(steps):
            t = i * self.trajectory_dt_s
            s = self.kf.propagate_state(state, t)
            p = Point()
            p.x = float(s[0])
            p.y = float(s[1])
            p.z = float(s[2])
            points.append(p)
        return points

    def publish_future_markers(
        self,
        state: np.ndarray,
        catch_pos: Optional[np.ndarray],
        stamp_t: float,
    ) -> None:
        marker_array = MarkerArray()
        stamp_msg = Time(nanoseconds=int(stamp_t * 1e9)).to_msg()
        frame_id = self.last_frame_id or self.default_frame_id

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp_msg
        line.ns = 'ball_catch_predictor'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.01
        line.color.a = 1.0
        line.color.r = 0.1
        line.color.g = 0.8
        line.color.b = 1.0
        line.points = self.future_trajectory_points(state)
        marker_array.markers.append(line)

        catch = Marker()
        catch.header.frame_id = frame_id
        catch.header.stamp = stamp_msg
        catch.ns = 'ball_catch_predictor'
        catch.id = 1
        catch.type = Marker.SPHERE
        catch.action = Marker.ADD if catch_pos is not None else Marker.DELETE
        diameter = max(0.001, 2.0 * self.ball_radius_m)
        catch.scale.x = diameter
        catch.scale.y = diameter
        catch.scale.z = diameter
        catch.color.a = 1.0
        catch.color.r = 1.0
        catch.color.g = 0.2
        catch.color.b = 0.2
        if catch_pos is not None:
            catch.pose.position.x = float(catch_pos[0])
            catch.pose.position.y = float(catch_pos[1])
            catch.pose.position.z = float(catch_pos[2])
            catch.pose.orientation.w = 1.0
        marker_array.markers.append(catch)

        plane = Marker()
        plane.header.frame_id = frame_id
        plane.header.stamp = stamp_msg
        plane.ns = 'ball_catch_predictor'
        plane.id = 2
        plane.type = Marker.LINE_STRIP
        plane.action = Marker.ADD
        plane.scale.x = 0.006
        plane.color.a = 0.7
        plane.color.r = 1.0
        plane.color.g = 1.0
        plane.color.b = 0.1
        y = self.catch_y
        corners = [
            (self.catch_x_min, y, self.catch_z_min),
            (self.catch_x_max, y, self.catch_z_min),
            (self.catch_x_max, y, self.catch_z_max),
            (self.catch_x_min, y, self.catch_z_max),
            (self.catch_x_min, y, self.catch_z_min),
        ]
        for x, yy, z in corners:
            p = Point()
            p.x = float(x)
            p.y = float(yy)
            p.z = float(z)
            plane.points.append(p)
        marker_array.markers.append(plane)

        self.marker_pub.publish(marker_array)

    def publish_delete_markers(self) -> None:
        if not self.publish_markers:
            return
        marker_array = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

    def finish_toss(self, reason: str = 'unknown', force_sync_save: bool = False) -> None:
        if not self.toss_active:
            return

        data = TossData(
            toss_id=self.toss_id,
            measurements=list(self.measurements),
            predictions=list(self.predictions),
            rejected_measurements=self.rejected_measurements,
            catch_y=self.catch_y,
            catch_x_min=self.catch_x_min,
            catch_x_max=self.catch_x_max,
            catch_z_min=self.catch_z_min,
            catch_z_max=self.catch_z_max,
        )

        self.toss_active = False
        self.measurements = []
        self.predictions = []
        self.rejected_measurements = 0

        self.get_logger().info(
            f'Finished toss {data.toss_id} ({reason}): {len(data.measurements)} measurements, '
            f'{len(data.predictions)} y-plane predictions, '
            f'{sum(1 for p in data.predictions if p.published)} published catch points, '
            f'{data.rejected_measurements} rejected measurements.'
        )

        if not self.save_plots:
            return

        if self.plot_after_shutdown_only and not force_sync_save:
            self.deferred_tosses.append(data)
            if len(self.deferred_tosses) > self.max_deferred_tosses:
                dropped = self.deferred_tosses.pop(0)
                self.get_logger().warn(
                    f'Dropped deferred toss {dropped.toss_id} because max_deferred_tosses was exceeded.'
                )
            self.get_logger().info(
                f'Toss {data.toss_id}: queued CSV/plot generation until node shutdown.'
            )
            return

        self.save_toss_data(data, async_save=(self.save_plots_async and not force_sync_save))

    def save_toss_data(self, data: TossData, async_save: bool) -> None:
        if async_save:
            thread = threading.Thread(
                target=self.save_toss_outputs,
                args=(data,),
                daemon=False,
            )
            self.plot_threads.append(thread)
            thread.start()
        else:
            self.save_toss_outputs(data)

    def save_deferred_tosses(self) -> None:
        if not self.save_plots:
            return
        if not self.deferred_tosses:
            return

        pending = list(self.deferred_tosses)
        self.deferred_tosses = []
        self.get_logger().info(f'Saving {len(pending)} deferred toss report(s) to {self.plot_dir}.')
        for data in pending:
            self.save_toss_data(data, async_save=False)

    def wait_for_plot_threads(self) -> None:
        threads = list(self.plot_threads)
        self.plot_threads = []
        for thread in threads:
            if thread.is_alive():
                thread.join()

    @staticmethod
    def compute_actual_crossing(
        measurements: List[MeasurementRecord],
        catch_y: float,
    ) -> Optional[ActualCrossing]:
        if not measurements:
            return None

        # Prefer the bias-corrected measured crossing. Use interpolation between
        # the first pair that brackets the catch plane.
        for a, b in zip(measurements[:-1], measurements[1:]):
            y0 = float(a.corrected[1])
            y1 = float(b.corrected[1])
            d0 = y0 - catch_y
            d1 = y1 - catch_y
            if abs(d0) < 1e-12:
                pos = a.corrected.copy()
                pos[1] = catch_y
                return ActualCrossing(a.t, pos, 'corrected_exact')
            if d0 * d1 <= 0.0 and abs(y1 - y0) > 1e-12:
                alpha = (catch_y - y0) / (y1 - y0)
                alpha = float(np.clip(alpha, 0.0, 1.0))
                pos = a.corrected + alpha * (b.corrected - a.corrected)
                pos[1] = catch_y
                t = a.t + alpha * (b.t - a.t)
                return ActualCrossing(t, pos, 'corrected_interpolated')

        # If the ball never visibly crossed the plane, use the closest corrected point.
        idx = int(np.argmin([abs(float(m.corrected[1]) - catch_y) for m in measurements]))
        m = measurements[idx]
        pos = m.corrected.copy()
        return ActualCrossing(m.t, pos, 'closest_corrected_no_crossing')

    def save_toss_outputs(self, data: TossData) -> None:
        try:
            self.plot_dir.mkdir(parents=True, exist_ok=True)
            actual = self.compute_actual_crossing(data.measurements, data.catch_y)

            csv_path = self.plot_dir / f'toss_{data.toss_id:03d}_catch_prediction_errors.csv'
            self.write_prediction_csv(csv_path, data.predictions, actual)

            meas_csv_path = self.plot_dir / f'toss_{data.toss_id:03d}_measurements.csv'
            self.write_measurement_csv(meas_csv_path, data.measurements)

            if actual is None:
                self.get_logger().warn(f'Toss {data.toss_id}: no actual crossing could be computed.')
                return

            self.write_summary_csv(
                self.plot_dir / f'toss_{data.toss_id:03d}_summary.csv',
                data,
                actual,
            )
            self.make_plots(self.plot_dir, data, actual)
            self.get_logger().info(
                f'Toss {data.toss_id}: saved validation CSV/plots in {str(self.plot_dir)}. '
                f'Actual plane point method={actual.method}, '
                f'pos=({actual.pos[0]:.3f}, {actual.pos[1]:.3f}, {actual.pos[2]:.3f}).'
            )
        except Exception as exc:  # noqa: BLE001 - plotting should never kill the node
            self.get_logger().error(f'Failed to save toss {data.toss_id} outputs: {exc}')

    @staticmethod
    def write_measurement_csv(path: Path, measurements: List[MeasurementRecord]) -> None:
        with path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                't_sample_s', 't_receive_s', 'receive_minus_sample_s',
                'raw_x_m', 'raw_y_m', 'raw_z_m',
                'corrected_x_m', 'corrected_y_m', 'corrected_z_m',
                'filtered_x_m', 'filtered_y_m', 'filtered_z_m',
            ])
            for m in measurements:
                writer.writerow([
                    f'{m.t:.9f}',
                    f'{m.receive_t:.9f}',
                    f'{m.receive_t - m.t:.9f}',
                    f'{m.raw[0]:.9f}', f'{m.raw[1]:.9f}', f'{m.raw[2]:.9f}',
                    f'{m.corrected[0]:.9f}', f'{m.corrected[1]:.9f}', f'{m.corrected[2]:.9f}',
                    f'{m.filtered[0]:.9f}', f'{m.filtered[1]:.9f}', f'{m.filtered[2]:.9f}',
                ])

    @staticmethod
    def write_prediction_csv(
        path: Path,
        predictions: List[PredictionRecord],
        actual: Optional[ActualCrossing],
    ) -> None:
        with path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'toss_id', 'source', 'published', 'in_plane_bounds',
                't_pub_s', 't_catch_pred_s', 'lead_time_s',
                'pred_x_m', 'pred_y_m', 'pred_z_m',
                'state_x_m', 'state_y_m', 'state_z_m',
                'state_vx_mps', 'state_vy_mps', 'state_vz_mps',
                'actual_method', 'actual_t_s', 'actual_x_m', 'actual_y_m', 'actual_z_m',
                'error_x_m', 'error_y_m', 'error_z_m',
                'abs_error_x_m', 'abs_error_y_m', 'abs_error_z_m',
                'error_norm_3d_m', 'time_error_s',
            ])
            for p in predictions:
                if actual is None:
                    actual_values = ['', '', '', '', '']
                    errors = ['', '', '', '', '', '', '', '']
                else:
                    err = p.catch_pos - actual.pos
                    abs_err = np.abs(err)
                    norm = float(np.linalg.norm(err))
                    time_err = p.t_catch - actual.t
                    actual_values = [
                        actual.method,
                        f'{actual.t:.9f}',
                        f'{actual.pos[0]:.9f}',
                        f'{actual.pos[1]:.9f}',
                        f'{actual.pos[2]:.9f}',
                    ]
                    errors = [
                        f'{err[0]:.9f}', f'{err[1]:.9f}', f'{err[2]:.9f}',
                        f'{abs_err[0]:.9f}', f'{abs_err[1]:.9f}', f'{abs_err[2]:.9f}',
                        f'{norm:.9f}', f'{time_err:.9f}',
                    ]
                writer.writerow([
                    p.toss_id,
                    p.source,
                    int(p.published),
                    int(p.in_plane_bounds),
                    f'{p.t_pub:.9f}',
                    f'{p.t_catch:.9f}',
                    f'{p.lead_time:.9f}',
                    f'{p.catch_pos[0]:.9f}', f'{p.catch_pos[1]:.9f}', f'{p.catch_pos[2]:.9f}',
                    f'{p.state_pos[0]:.9f}', f'{p.state_pos[1]:.9f}', f'{p.state_pos[2]:.9f}',
                    f'{p.state_vel[0]:.9f}', f'{p.state_vel[1]:.9f}', f'{p.state_vel[2]:.9f}',
                    *actual_values,
                    *errors,
                ])

    @staticmethod
    def write_summary_csv(path: Path, data: TossData, actual: ActualCrossing) -> None:
        all_errors: List[float] = []
        published_errors: List[float] = []
        time_errors: List[float] = []
        abs_errors = []
        for p in data.predictions:
            err = p.catch_pos - actual.pos
            norm = float(np.linalg.norm(err))
            all_errors.append(norm)
            abs_errors.append(np.abs(err))
            time_errors.append(float(p.t_catch - actual.t))
            if p.published:
                published_errors.append(norm)

        def safe_stat(values: List[float], fn) -> str:
            return '' if not values else f'{fn(values):.9f}'

        with path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['key', 'value'])
            writer.writerow(['toss_id', data.toss_id])
            writer.writerow(['measurement_count', len(data.measurements)])
            writer.writerow(['prediction_count', len(data.predictions)])
            writer.writerow(['published_prediction_count', sum(1 for p in data.predictions if p.published)])
            writer.writerow(['in_plane_bounds_count', sum(1 for p in data.predictions if p.in_plane_bounds)])
            writer.writerow(['rejected_measurements', data.rejected_measurements])
            writer.writerow(['catch_y_m', f'{data.catch_y:.9f}'])
            writer.writerow(['catch_x_min_m', f'{data.catch_x_min:.9f}'])
            writer.writerow(['catch_x_max_m', f'{data.catch_x_max:.9f}'])
            writer.writerow(['catch_z_min_m', f'{data.catch_z_min:.9f}'])
            writer.writerow(['catch_z_max_m', f'{data.catch_z_max:.9f}'])
            writer.writerow(['actual_method', actual.method])
            writer.writerow(['actual_t_s', f'{actual.t:.9f}'])
            writer.writerow(['actual_x_m', f'{actual.pos[0]:.9f}'])
            writer.writerow(['actual_y_m', f'{actual.pos[1]:.9f}'])
            writer.writerow(['actual_z_m', f'{actual.pos[2]:.9f}'])
            writer.writerow(['mean_error_norm_all_predictions_m', safe_stat(all_errors, lambda v: float(np.mean(v)))])
            writer.writerow(['median_error_norm_all_predictions_m', safe_stat(all_errors, lambda v: float(np.median(v)))])
            writer.writerow(['max_error_norm_all_predictions_m', safe_stat(all_errors, lambda v: float(np.max(v)))])
            writer.writerow(['mean_error_norm_published_m', safe_stat(published_errors, lambda v: float(np.mean(v)))])
            writer.writerow(['mean_time_error_s', safe_stat(time_errors, lambda v: float(np.mean(v)))])
            if abs_errors:
                arr = np.vstack(abs_errors)
                writer.writerow(['mean_abs_x_error_m', f'{float(np.mean(arr[:, 0])):.9f}'])
                writer.writerow(['mean_abs_y_error_m', f'{float(np.mean(arr[:, 1])):.9f}'])
                writer.writerow(['mean_abs_z_error_m', f'{float(np.mean(arr[:, 2])):.9f}'])

    @staticmethod
    def make_plots(plot_dir: Path, data: TossData, actual: ActualCrossing) -> None:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        measurements = data.measurements
        predictions = data.predictions
        if not measurements:
            return

        t0 = measurements[0].t
        t_meas = np.array([m.t - t0 for m in measurements])
        raw = np.vstack([m.raw for m in measurements])
        corrected = np.vstack([m.corrected for m in measurements])
        filt = np.vstack([m.filtered for m in measurements])
        actual_t_rel = actual.t - t0

        prefix = plot_dir / f'toss_{data.toss_id:03d}'

        # 1. 3D position plot with measured ball, predicted catch points, and KF state.
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(corrected[:, 0], corrected[:, 1], corrected[:, 2], s=10, label='bias-corrected measured ball')
        ax.plot(filt[:, 0], filt[:, 1], filt[:, 2], linewidth=1.5, label='KF filtered ball')
        if predictions:
            pred = np.vstack([p.catch_pos for p in predictions])
            published_mask = np.array([p.published for p in predictions], dtype=bool)
            if np.any(published_mask):
                ax.scatter(
                    pred[published_mask, 0], pred[published_mask, 1], pred[published_mask, 2],
                    s=12, label='published predicted catch points'
                )
            if np.any(~published_mask):
                ax.scatter(
                    pred[~published_mask, 0], pred[~published_mask, 1], pred[~published_mask, 2],
                    s=8, marker='x', label='outside-bounds y-plane predictions'
                )
        ax.scatter([actual.pos[0]], [actual.pos[1]], [actual.pos[2]], s=80, marker='o', label='actual/closest plane point')
        verts = [[
            (data.catch_x_min, data.catch_y, data.catch_z_min),
            (data.catch_x_max, data.catch_y, data.catch_z_min),
            (data.catch_x_max, data.catch_y, data.catch_z_max),
            (data.catch_x_min, data.catch_y, data.catch_z_max),
        ]]
        plane = Poly3DCollection(verts, alpha=0.12)
        ax.add_collection3d(plane)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        ax.set_title(f'Toss {data.toss_id}: trajectory and catch predictions')
        ax.legend(loc='best')
        fig.tight_layout()
        fig.savefig(str(prefix) + '_trajectory_3d.png', dpi=160)
        plt.close(fig)

        # 2. x, y, z vs time since first detection.
        labels = ['x [m]', 'y [m]', 'z [m]']
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        for i, ax in enumerate(axes):
            ax.scatter(t_meas, corrected[:, i], s=10, label='bias-corrected measured ball')
            ax.plot(t_meas, filt[:, i], linewidth=1.5, label='KF filtered ball')
            ax.axvline(actual_t_rel, linestyle=':', label='actual plane-crossing time')
            if i == 1:
                ax.axhline(data.catch_y, linestyle='--', label='catch plane y')
            ax.set_ylabel(labels[i])
            ax.grid(True, linewidth=0.3)
            ax.legend(loc='best')
        axes[-1].set_xlabel('time since first detected [s]')
        fig.suptitle(f'Toss {data.toss_id}: ball position vs time')
        fig.tight_layout()
        fig.savefig(str(prefix) + '_xyz_vs_time.png', dpi=160)
        plt.close(fig)

        # Backward-compatible y-only plot.
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.scatter(t_meas, corrected[:, 1], s=10, label='bias-corrected measured y')
        ax.plot(t_meas, filt[:, 1], linewidth=1.5, label='KF filtered y')
        ax.axhline(data.catch_y, linestyle='--', label='catch plane y')
        ax.axvline(actual_t_rel, linestyle=':', label='actual plane-crossing time')
        ax.set_xlabel('time since first detected [s]')
        ax.set_ylabel('y [m]')
        ax.set_title(f'Toss {data.toss_id}: catch-plane crossing')
        ax.legend(loc='best')
        ax.grid(True, linewidth=0.3)
        fig.tight_layout()
        fig.savefig(str(prefix) + '_y_plane_crossing.png', dpi=160)
        plt.close(fig)

        if not predictions:
            return

        t_pred = np.array([p.t_pub - t0 for p in predictions])
        lead = np.array([p.lead_time for p in predictions])
        pred_pos = np.vstack([p.catch_pos for p in predictions])
        published_mask = np.array([p.published for p in predictions], dtype=bool)
        err = pred_pos - actual.pos.reshape(1, 3)
        abs_err = np.abs(err)
        err_norm = np.linalg.norm(err, axis=1)
        time_err = np.array([p.t_catch - actual.t for p in predictions])

        # 3. Catch coverage: predicted catch x/y/z vs prediction time.
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        actual_values = [actual.pos[0], actual.pos[1], actual.pos[2]]
        bounds = [
            (data.catch_x_min, data.catch_x_max),
            (data.catch_y, data.catch_y),
            (data.catch_z_min, data.catch_z_max),
        ]
        for i, ax in enumerate(axes):
            if np.any(published_mask):
                ax.scatter(t_pred[published_mask], pred_pos[published_mask, i], s=12, label='published catch prediction')
            if np.any(~published_mask):
                ax.scatter(t_pred[~published_mask], pred_pos[~published_mask, i], s=10, marker='x', label='not published/outside bounds')
            ax.axhline(actual_values[i], linestyle='--', label='actual value at plane')
            ax.axvline(actual_t_rel, linestyle=':', label='actual plane-crossing time')
            low, high = bounds[i]
            if i in (0, 2):
                ax.axhline(low, linestyle='-.', label='catch bound min')
                ax.axhline(high, linestyle='-.', label='catch bound max')
            else:
                ax.axhline(data.catch_y, linestyle='-.', label='catch plane y')
            ax.set_ylabel(labels[i])
            ax.grid(True, linewidth=0.3)
            ax.legend(loc='best')
        axes[-1].set_xlabel('prediction time since first detected [s]')
        fig.suptitle(f'Toss {data.toss_id}: catch coverage vs time')
        fig.tight_layout()
        fig.savefig(str(prefix) + '_catch_coverage_xyz.png', dpi=160)
        plt.close(fig)

        # 4. Prediction error vs time: 3D error and absolute component errors.
        fig, ax = plt.subplots(figsize=(11, 6))
        ax.scatter(t_pred, err_norm, s=14, label='3D catch-point error')
        ax.scatter(t_pred, abs_err[:, 0], s=10, label='abs x error')
        ax.scatter(t_pred, abs_err[:, 1], s=10, label='abs y error')
        ax.scatter(t_pred, abs_err[:, 2], s=10, label='abs z error')
        ax.axvline(actual_t_rel, linestyle=':', label='actual plane-crossing time')
        ax.set_xlabel('prediction time since first detected [s]')
        ax.set_ylabel('error [m]')
        ax.set_title(f'Toss {data.toss_id}: prediction error vs time')
        ax.legend(loc='best')
        ax.grid(True, linewidth=0.3)
        fig.tight_layout()
        fig.savefig(str(prefix) + '_prediction_error_vs_time.png', dpi=160)
        plt.close(fig)

        # Additional helpful plot: x-z view of the bounded catch rectangle.
        fig, ax = plt.subplots(figsize=(8, 7))
        rect_x = [data.catch_x_min, data.catch_x_max, data.catch_x_max, data.catch_x_min, data.catch_x_min]
        rect_z = [data.catch_z_min, data.catch_z_min, data.catch_z_max, data.catch_z_max, data.catch_z_min]
        ax.plot(rect_x, rect_z, linestyle='-', label='bounded catch plane')
        if np.any(published_mask):
            ax.scatter(pred_pos[published_mask, 0], pred_pos[published_mask, 2], s=12, label='published catch predictions')
        if np.any(~published_mask):
            ax.scatter(pred_pos[~published_mask, 0], pred_pos[~published_mask, 2], s=10, marker='x', label='outside-bounds predictions')
        ax.scatter([actual.pos[0]], [actual.pos[2]], s=80, marker='o', label='actual/closest plane point')
        ax.set_xlabel('x at catch plane [m]')
        ax.set_ylabel('z at catch plane [m]')
        ax.set_title(f'Toss {data.toss_id}: catch-plane x-z coverage')
        ax.axis('equal')
        ax.legend(loc='best')
        ax.grid(True, linewidth=0.3)
        fig.tight_layout()
        fig.savefig(str(prefix) + '_catch_plane_xz_coverage.png', dpi=160)
        plt.close(fig)

        # Additional helpful plot: error vs remaining lead time.
        fig, ax = plt.subplots(figsize=(11, 6))
        ax.scatter(lead, err_norm, s=14, label='3D catch-point error')
        ax.scatter(lead, abs_err[:, 0], s=10, label='abs x error')
        ax.scatter(lead, abs_err[:, 1], s=10, label='abs y error')
        ax.scatter(lead, abs_err[:, 2], s=10, label='abs z error')
        ax.set_xlabel('predicted time until catch [s]')
        ax.set_ylabel('error [m]')
        ax.set_title(f'Toss {data.toss_id}: prediction error vs lead time')
        ax.legend(loc='best')
        ax.grid(True, linewidth=0.3)
        fig.tight_layout()
        fig.savefig(str(prefix) + '_prediction_error_vs_lead_time.png', dpi=160)
        plt.close(fig)

        # Additional helpful plot: catch-time error.
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.scatter(t_pred, time_err, s=12, label='predicted catch time - actual plane time')
        ax.axhline(0.0, linestyle='--')
        ax.axvline(actual_t_rel, linestyle=':', label='actual plane-crossing time')
        ax.set_xlabel('prediction time since first detected [s]')
        ax.set_ylabel('time error [s]')
        ax.set_title(f'Toss {data.toss_id}: catch-time prediction error')
        ax.legend(loc='best')
        ax.grid(True, linewidth=0.3)
        fig.tight_layout()
        fig.savefig(str(prefix) + '_catch_time_error.png', dpi=160)
        plt.close(fig)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallCatchPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish_toss(reason='shutdown', force_sync_save=False)
        node.save_deferred_tosses()
        node.wait_for_plot_threads()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
