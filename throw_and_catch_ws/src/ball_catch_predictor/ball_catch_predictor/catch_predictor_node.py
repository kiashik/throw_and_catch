#!/usr/bin/env python3
"""
ROS 2 Jazzy ball catch-point predictor.

Subscribes:
  /vision/ball_pose_robot    geometry_msgs/msg/PoseStamped

Publishes:
  /vision/catch_point        geometry_msgs/msg/PoseStamped
  /vision/ball_prediction_markers visualization_msgs/msg/MarkerArray

Model:
  state x = [px, py, pz, vx, vy, vz]
  dp/dt = v
  dv/dt = -drag_coeff * v + [0, 0, -gravity]

At toss end, the node saves CSV logs and PNG plots comparing every predicted
catch point against the measured/interpolated point where the ball reached the
catch plane y = catch_y.
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


@dataclass
class MeasurementRecord:
    t: float
    raw: np.ndarray
    corrected: np.ndarray
    filtered: np.ndarray


@dataclass
class PredictionRecord:
    toss_id: int
    t_pub: float
    t_catch: float
    lead_time: float
    catch_pos: np.ndarray
    state_pos: np.ndarray
    state_vel: np.ndarray
    source: str


@dataclass
class ActualCrossing:
    t: float
    pos: np.ndarray
    method: str

#region -- Kalman filter --
class LinearDragKalmanFilter:
    """Small, fast Kalman filter for 3D ball position/velocity."""

    def __init__(
        self,
        gravity: float,
        drag_coeff: float,
        meas_std: np.ndarray,
        process_accel_std: float,
        initial_velocity_std: float,
    ) -> None:
        self.gravity = float(gravity)
        self.drag_coeff = float(drag_coeff)
        self.meas_std = meas_std.astype(float)
        self.process_accel_std = float(process_accel_std)
        self.initial_velocity_std = float(initial_velocity_std)

        self.x = np.zeros(6, dtype=float)           # 6x1 state: [px, py, pz, vx, vy, vz]
        self.P = np.eye(6, dtype=float)             # (6x6) process noise (model accuracy). same as matrix Q
        self.R = np.diag(self.meas_std ** 2)        # (3x3) measurement noise. measurement vector: [px, py, pz] 
        self.H = np.zeros((3, 6), dtype=float)      # observation matrix (C from State Space model): maps state to measurement.
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
        pos_var = np.maximum(self.meas_std ** 2, 1e-8)
        vel_var = np.ones(3) * (self.initial_velocity_std ** 2)
        self.P = np.diag(np.concatenate([pos_var, vel_var]))
        self.initialized = True

    def transition(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Return discrete transition Phi and constant offset b for x_next=Phi*x+b."""
        dt = max(0.0, float(dt))
        k = self.drag_coeff
        Phi = np.eye(6, dtype=float)
        b = np.zeros(6, dtype=float)
        a = self.accel

        if dt <= 0.0:
            return Phi, b

        if abs(k) < 1e-9:
            Phi[0:3, 3:6] = dt * np.eye(3)
            b[0:3] = 0.5 * a * dt * dt
            b[3:6] = a * dt
            return Phi, b

        e = math.exp(-k * dt)
        B = (1.0 - e) / k
        C = dt / k - (1.0 - e) / (k * k)

        Phi[0:3, 3:6] = B * np.eye(3)
        Phi[3:6, 3:6] = e * np.eye(3)
        b[0:3] = C * a
        b[3:6] = B * a
        return Phi, b

    def predict(self, dt: float) -> None:
        if not self.initialized:
            return
        dt = max(0.0, float(dt))
        Phi, b = self.transition(dt)
        self.x = Phi @ self.x + b

        # Approximate acceleration-process noise. This intentionally allows the
        # filter to absorb imperfect drag, camera noise, and small spin effects.
        q = self.process_accel_std ** 2
        Q = np.zeros((6, 6), dtype=float)
        Q[0:3, 0:3] = (dt ** 4 / 4.0) * q * np.eye(3)
        Q[0:3, 3:6] = (dt ** 3 / 2.0) * q * np.eye(3)
        Q[3:6, 0:3] = Q[0:3, 3:6].T
        Q[3:6, 3:6] = (dt ** 2) * q * np.eye(3)

        self.P = Phi @ self.P @ Phi.T + Q
        self.P = 0.5 * (self.P + self.P.T)  # keep covariance symmetric

    def update(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Kalman position update. Returns innovation and innovation covariance."""
        z = z.astype(float)
        innovation = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R

        # K = P H^T S^-1, solved without explicitly inverting S.
        K = np.linalg.solve(S.T, (self.P @ self.H.T).T).T
        self.x = self.x + K @ innovation

        # Joseph form improves numerical stability for tiny covariances.
        KH = K @ self.H
        self.P = (self.I - KH) @ self.P @ (self.I - KH).T + K @ self.R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return innovation, S

    def propagate_state(self, state: np.ndarray, dt: float) -> np.ndarray:
        Phi, b = self.transition(dt)
        return Phi @ state + b
#endregion

#region -- node --
class BallCatchPredictorNode(Node):
    def __init__(self) -> None:
        super().__init__('ball_catch_predictor')

        # Topics and frame behavior.
        self.declare_parameter('input_topic', '/vision/ball_pose_robot')
        self.declare_parameter('catch_topic', '/vision/catch_point')
        self.declare_parameter('marker_topic', '/vision/ball_prediction_markers')
        self.declare_parameter('default_frame_id', 'base_link')
        self.declare_parameter('stamp_catch_pose_with_arrival_time', True)

        # Physics and catch-point search.
        self.declare_parameter('catch_y', -0.30)
        self.declare_parameter('gravity', 9.80665)
        self.declare_parameter('drag_coeff', 0.05)  # 1/s; tune with logs.
        self.declare_parameter('prediction_horizon_s', 1.50)
        self.declare_parameter('prediction_rate_hz', 60.0)
        self.declare_parameter('publish_on_measurement', True)
        self.declare_parameter('trajectory_dt_s', 0.02)

        # Filter tuning.
        # Characterized sensor noise. These defaults come from the supplied
        # static calibration variances: sqrt([2.18014155e-6, 9.53959917e-7, 3.44773465e-5]).
        self.declare_parameter('measurement_std_x_m', 0.00147653024)
        self.declare_parameter('measurement_std_y_m', 0.00097670872)
        self.declare_parameter('measurement_std_z_m', 0.00587174135)

        # Fixed measurement bias = measured mean - ground truth. The node subtracts
        # this from every incoming raw pose before feeding the Kalman filter.
        self.declare_parameter('apply_measurement_bias_correction', True)
        self.declare_parameter('measurement_bias_x_m', 0.01031956)
        self.declare_parameter('measurement_bias_y_m', -0.01392079)
        self.declare_parameter('measurement_bias_z_m', -0.01027870)
        self.declare_parameter('process_accel_std_mps2', 2.0)
        self.declare_parameter('initial_velocity_std_mps', 5.0)
        self.declare_parameter('gate_threshold_m', 0.30)

        # Timing robustness.
        self.declare_parameter('use_header_stamp', True)
        self.declare_parameter('max_reasonable_latency_s', 2.0)
        self.declare_parameter('future_stamp_tolerance_s', 0.10)
        self.declare_parameter('max_filter_dt_s', 0.20)
        self.declare_parameter('toss_timeout_s', 0.35)

        # Workspace filters. Set use_workspace_filter=false to publish all plane intersections.
        self.declare_parameter('use_workspace_filter', False)
        self.declare_parameter('workspace_x_min', -0.80)
        self.declare_parameter('workspace_x_max', 0.80)
        self.declare_parameter('workspace_y_min', -0.90)
        self.declare_parameter('workspace_y_max', 0.10)
        self.declare_parameter('workspace_z_min', 0.05)
        self.declare_parameter('workspace_z_max', 1.50)

        # Plotting and RViz visualization.
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('save_plots', True)
        self.declare_parameter('plot_dir', '~/ball_catch_plots')
        self.declare_parameter('catch_plane_marker_x_min', -0.80)
        self.declare_parameter('catch_plane_marker_x_max', 0.80)
        self.declare_parameter('catch_plane_marker_z_min', 0.00)
        self.declare_parameter('catch_plane_marker_z_max', 1.50)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.catch_topic = str(self.get_parameter('catch_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.default_frame_id = str(self.get_parameter('default_frame_id').value)
        self.stamp_catch_pose_with_arrival_time = bool(
            self.get_parameter('stamp_catch_pose_with_arrival_time').value
        )

        self.catch_y = float(self.get_parameter('catch_y').value)
        self.gravity = float(self.get_parameter('gravity').value)
        self.drag_coeff = float(self.get_parameter('drag_coeff').value)
        self.prediction_horizon_s = float(self.get_parameter('prediction_horizon_s').value)
        self.prediction_rate_hz = max(8.0, float(self.get_parameter('prediction_rate_hz').value))
        self.publish_on_measurement = bool(self.get_parameter('publish_on_measurement').value)
        self.trajectory_dt_s = float(self.get_parameter('trajectory_dt_s').value)

        meas_std = np.array([
            float(self.get_parameter('measurement_std_x_m').value),
            float(self.get_parameter('measurement_std_y_m').value),
            float(self.get_parameter('measurement_std_z_m').value),
        ], dtype=float)
        self.apply_measurement_bias_correction = bool(
            self.get_parameter('apply_measurement_bias_correction').value
        )
        self.measurement_bias = np.array([
            float(self.get_parameter('measurement_bias_x_m').value),
            float(self.get_parameter('measurement_bias_y_m').value),
            float(self.get_parameter('measurement_bias_z_m').value),
        ], dtype=float)
        process_accel_std = float(self.get_parameter('process_accel_std_mps2').value)
        initial_velocity_std = float(self.get_parameter('initial_velocity_std_mps').value)
        self.gate_threshold_m = float(self.get_parameter('gate_threshold_m').value)

        self.use_header_stamp = bool(self.get_parameter('use_header_stamp').value)
        self.max_reasonable_latency_s = float(self.get_parameter('max_reasonable_latency_s').value)
        self.future_stamp_tolerance_s = float(self.get_parameter('future_stamp_tolerance_s').value)
        self.max_filter_dt_s = float(self.get_parameter('max_filter_dt_s').value)
        self.toss_timeout_s = float(self.get_parameter('toss_timeout_s').value)

        self.use_workspace_filter = bool(self.get_parameter('use_workspace_filter').value)
        self.workspace_bounds = (
            float(self.get_parameter('workspace_x_min').value),
            float(self.get_parameter('workspace_x_max').value),
            float(self.get_parameter('workspace_y_min').value),
            float(self.get_parameter('workspace_y_max').value),
            float(self.get_parameter('workspace_z_min').value),
            float(self.get_parameter('workspace_z_max').value),
        )

        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        self.save_plots = bool(self.get_parameter('save_plots').value)
        self.plot_dir = Path(os.path.expanduser(str(self.get_parameter('plot_dir').value)))
        self.catch_plane_marker_x_min = float(self.get_parameter('catch_plane_marker_x_min').value)
        self.catch_plane_marker_x_max = float(self.get_parameter('catch_plane_marker_x_max').value)
        self.catch_plane_marker_z_min = float(self.get_parameter('catch_plane_marker_z_min').value)
        self.catch_plane_marker_z_max = float(self.get_parameter('catch_plane_marker_z_max').value)

        self.kf = LinearDragKalmanFilter(
            gravity=self.gravity,
            drag_coeff=self.drag_coeff,
            meas_std=meas_std,
            process_accel_std=process_accel_std,
            initial_velocity_std=initial_velocity_std,
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

        self.state_time_s: Optional[float] = None
        self.last_receive_time_s: Optional[float] = None
        self.last_msg_sample_time_s: Optional[float] = None
        self.last_frame_id: str = self.default_frame_id
        self.using_header_time_basis = False
        self.last_latency_log_s = -1e9

        self.toss_id = 0
        self.toss_active = False
        self.measurements: List[MeasurementRecord] = []
        self.predictions: List[PredictionRecord] = []
        self.rejected_measurements = 0

        if self.save_plots:
            self.plot_dir.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(
            f'Subscribing to {self.input_topic}; publishing catch points to {self.catch_topic} '
            f'at timer rate {self.prediction_rate_hz:.1f} Hz plus measurement callbacks.'
        )
        self.get_logger().info(
            f'Model: linear drag k={self.drag_coeff:.4f} 1/s, catch_y={self.catch_y:.3f} m, '
            f'prediction horizon={self.prediction_horizon_s:.2f} s.'
        )
        self.get_logger().info(
            'Measurement std [m]='
            f'({meas_std[0]:.6f}, {meas_std[1]:.6f}, {meas_std[2]:.6f}); '
            f'bias correction enabled={self.apply_measurement_bias_correction}; '
            'bias [m]='
            f'({self.measurement_bias[0]:.6f}, {self.measurement_bias[1]:.6f}, '
            f'{self.measurement_bias[2]:.6f}).'
        )

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

        If the PoseStamped header stamp appears to share the node's time base,
        use it. Otherwise, fall back to receive time so prediction timers remain
        consistent with the filter's state timestamp.
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

        # Header time may be from a different clock/epoch. Use receive time.
        return receive_t, False, latency

    def pose_callback(self, msg: PoseStamped) -> None:
        receive_t = self.now_s()
        z_raw = np.array([          # Measurement vector: [px, py, pz]
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
                self.finish_toss_async()
                self.kf.reset()
                self.state_time_s = None
                self.last_msg_sample_time_s = None

        if not self.kf.initialized:
            self.start_new_toss_if_needed()
            self.kf.initialize(z)
            self.state_time_s = sample_t
            self.last_msg_sample_time_s = sample_t
            self.last_receive_time_s = receive_t
            self.measurements.append(MeasurementRecord(sample_t, z_raw.copy(), z.copy(), self.kf.x[0:3].copy()))
            if self.publish_on_measurement:
                self.publish_prediction(receive_t, source='measurement_init')
            return

        assert self.state_time_s is not None
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
            self.finish_toss_async()
            self.kf.reset()
            self.start_new_toss_if_needed(force=True)
            self.kf.initialize(z)
            self.state_time_s = sample_t
            self.last_msg_sample_time_s = sample_t
            self.last_receive_time_s = receive_t
            self.measurements.append(MeasurementRecord(sample_t, z_raw.copy(), z.copy(), self.kf.x[0:3].copy()))
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
        self.measurements.append(MeasurementRecord(sample_t, z_raw.copy(), z.copy(), self.kf.x[0:3].copy()))

        if self.publish_on_measurement:
            self.publish_prediction(receive_t, source='measurement')

    def timer_callback(self) -> None:
        now_t = self.now_s()
        if self.toss_active and self.last_receive_time_s is not None:
            if now_t - self.last_receive_time_s > self.toss_timeout_s:
                self.finish_toss_async()
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
        # If query_t is from receive time but filter is still in header time, this dt is latency.
        # Clamp small negative values caused by scheduling jitter.
        if dt < -1e-3:
            dt = 0.0
        elif dt < 0.0:
            dt = 0.0
        if dt > self.prediction_horizon_s:
            # State is stale; timer will soon end the toss.
            return None
        return self.kf.propagate_state(self.kf.x.copy(), dt)

    def solve_time_to_y_plane(self, state: np.ndarray, y_target: float) -> Optional[float]:
        y0 = float(state[1])
        vy = float(state[4])
        delta = float(y_target - y0)
        k = self.drag_coeff

        if abs(delta) < 1e-9:
            return 0.0

        if abs(k) < 1e-9:
            if abs(vy) < 1e-9:
                return None
            t = delta / vy
            if t >= 0.0 and t <= self.prediction_horizon_s:
                return t
            return None

        if abs(vy) < 1e-9:
            return None

        # y(t) = y0 + vy/k * (1 - exp(-k t)).
        arg = 1.0 - (k * delta / vy)
        if arg <= 0.0:
            return None
        t = -math.log(arg) / k
        if t >= 0.0 and t <= self.prediction_horizon_s and math.isfinite(t):
            return t
        return None

    def find_catch_point(self, state: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
        t_catch = self.solve_time_to_y_plane(state, self.catch_y)
        if t_catch is None:
            return None
        catch_state = self.kf.propagate_state(state, t_catch)
        catch_pos = catch_state[0:3].copy()
        catch_pos[1] = self.catch_y
        if not self.in_workspace(catch_pos):
            return None
        return catch_pos, t_catch

    def in_workspace(self, pos: np.ndarray) -> bool:
        if not self.use_workspace_filter:
            return True
        x_min, x_max, y_min, y_max, z_min, z_max = self.workspace_bounds
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        return x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max

    def publish_prediction(self, query_t: float, source: str) -> None:
        state = self.current_state_at(query_t)
        if state is None:
            return

        result = self.find_catch_point(state)
        if result is None:
            if self.publish_markers:
                self.publish_future_markers(state, None, query_t)
            return

        catch_pos, lead_time = result
        catch_abs_t = query_t + lead_time

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
                    t_pub=query_t,
                    t_catch=catch_abs_t,
                    lead_time=lead_time,
                    catch_pos=catch_pos.copy(),
                    state_pos=state[0:3].copy(),
                    state_vel=state[3:6].copy(),
                    source=source,
                )
            )

        if self.publish_markers:
            self.publish_future_markers(state, catch_pos, query_t)

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
        catch.scale.x = 0.06
        catch.scale.y = 0.06
        catch.scale.z = 0.06
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
        x0 = self.catch_plane_marker_x_min
        x1 = self.catch_plane_marker_x_max
        z0 = self.catch_plane_marker_z_min
        z1 = self.catch_plane_marker_z_max
        corners = [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1), (x0, y, z0)]
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

    def finish_toss_async(self) -> None:
        if not self.toss_active:
            return

        toss_id = self.toss_id
        measurements = list(self.measurements)
        predictions = list(self.predictions)
        rejected = self.rejected_measurements
        catch_y = self.catch_y
        plot_dir = self.plot_dir
        save_plots = self.save_plots

        self.toss_active = False
        self.measurements = []
        self.predictions = []
        self.rejected_measurements = 0

        self.get_logger().info(
            f'Finished toss {toss_id}: {len(measurements)} measurements, '
            f'{len(predictions)} catch predictions, {rejected} rejected measurements.'
        )

        if not save_plots:
            return

        thread = threading.Thread(
            target=self.save_toss_outputs,
            args=(toss_id, measurements, predictions, rejected, catch_y, plot_dir),
            daemon=True,
        )
        thread.start()

    def compute_actual_crossing(
        self,
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
            if d0 == 0.0:
                return ActualCrossing(a.t, a.corrected.copy(), 'corrected_exact')
            if d0 * d1 <= 0.0 and abs(y1 - y0) > 1e-9:
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

    def save_toss_outputs(
        self,
        toss_id: int,
        measurements: List[MeasurementRecord],
        predictions: List[PredictionRecord],
        rejected_measurements: int,
        catch_y: float,
        plot_dir: Path,
    ) -> None:
        try:
            plot_dir.mkdir(parents=True, exist_ok=True)
            actual = self.compute_actual_crossing(measurements, catch_y)

            csv_path = plot_dir / f'toss_{toss_id:03d}_catch_prediction_errors.csv'
            self.write_prediction_csv(csv_path, predictions, actual)

            meas_csv_path = plot_dir / f'toss_{toss_id:03d}_measurements.csv'
            self.write_measurement_csv(meas_csv_path, measurements)

            if actual is None:
                self.get_logger().warn(f'Toss {toss_id}: no actual crossing could be computed.')
                return

            self.write_summary_csv(
                plot_dir / f'toss_{toss_id:03d}_summary.csv',
                toss_id,
                measurements,
                predictions,
                actual,
                rejected_measurements,
            )
            self.make_plots(plot_dir, toss_id, measurements, predictions, actual, catch_y)
            self.get_logger().info(
                f'Toss {toss_id}: saved validation CSV/plots in {str(plot_dir)}. '
                f'Actual plane point method={actual.method}, '
                f'pos=({actual.pos[0]:.3f}, {actual.pos[1]:.3f}, {actual.pos[2]:.3f}).'
            )
        except Exception as exc:  # noqa: BLE001 - plotting should never kill the node
            self.get_logger().error(f'Failed to save toss {toss_id} outputs: {exc}')

    @staticmethod
    def write_measurement_csv(path: Path, measurements: List[MeasurementRecord]) -> None:
        with path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                't_s',
                'raw_x_m', 'raw_y_m', 'raw_z_m',
                'corrected_x_m', 'corrected_y_m', 'corrected_z_m',
                'filtered_x_m', 'filtered_y_m', 'filtered_z_m',
            ])
            for m in measurements:
                writer.writerow([
                    f'{m.t:.9f}',
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
                'toss_id', 'source', 't_pub_s', 't_catch_pred_s', 'lead_time_s',
                'pred_x_m', 'pred_y_m', 'pred_z_m',
                'state_x_m', 'state_y_m', 'state_z_m',
                'state_vx_mps', 'state_vy_mps', 'state_vz_mps',
                'actual_method', 'actual_t_s', 'actual_x_m', 'actual_y_m', 'actual_z_m',
                'error_x_m', 'error_y_m', 'error_z_m', 'error_norm_m', 'time_error_s',
            ])
            for p in predictions:
                if actual is None:
                    actual_values = ['', '', '', '', '']
                    errors = ['', '', '', '', '']
                else:
                    err = p.catch_pos - actual.pos
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
                        f'{norm:.9f}', f'{time_err:.9f}',
                    ]
                writer.writerow([
                    p.toss_id,
                    p.source,
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
    def write_summary_csv(
        path: Path,
        toss_id: int,
        measurements: List[MeasurementRecord],
        predictions: List[PredictionRecord],
        actual: ActualCrossing,
        rejected_measurements: int,
    ) -> None:
        errors = []
        time_errors = []
        for p in predictions:
            errors.append(float(np.linalg.norm(p.catch_pos - actual.pos)))
            time_errors.append(float(p.t_catch - actual.t))

        def safe_stat(values: List[float], fn) -> str:
            return '' if not values else f'{fn(values):.9f}'

        with path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['key', 'value'])
            writer.writerow(['toss_id', toss_id])
            writer.writerow(['measurement_count', len(measurements)])
            writer.writerow(['prediction_count', len(predictions)])
            writer.writerow(['rejected_measurements', rejected_measurements])
            writer.writerow(['actual_method', actual.method])
            writer.writerow(['actual_t_s', f'{actual.t:.9f}'])
            writer.writerow(['actual_x_m', f'{actual.pos[0]:.9f}'])
            writer.writerow(['actual_y_m', f'{actual.pos[1]:.9f}'])
            writer.writerow(['actual_z_m', f'{actual.pos[2]:.9f}'])
            writer.writerow(['mean_error_norm_m', safe_stat(errors, lambda v: float(np.mean(v)))])
            writer.writerow(['median_error_norm_m', safe_stat(errors, lambda v: float(np.median(v)))])
            writer.writerow(['max_error_norm_m', safe_stat(errors, lambda v: float(np.max(v)))])
            writer.writerow(['mean_time_error_s', safe_stat(time_errors, lambda v: float(np.mean(v)))])

    @staticmethod
    def make_plots(
        plot_dir: Path,
        toss_id: int,
        measurements: List[MeasurementRecord],
        predictions: List[PredictionRecord],
        actual: ActualCrossing,
        catch_y: float,
    ) -> None:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        if not measurements:
            return

        t0 = measurements[0].t
        t_meas = np.array([m.t - t0 for m in measurements])
        raw = np.vstack([m.raw for m in measurements])
        corrected = np.vstack([m.corrected for m in measurements])
        filt = np.vstack([m.filtered for m in measurements])

        # 3D trajectory and catch-point cloud.
        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(raw[:, 0], raw[:, 1], raw[:, 2], '.', label='raw measured ball')
        ax.plot(corrected[:, 0], corrected[:, 1], corrected[:, 2], '.', label='bias-corrected ball')
        ax.plot(filt[:, 0], filt[:, 1], filt[:, 2], '-', label='KF filtered ball')
        if predictions:
            pred = np.vstack([p.catch_pos for p in predictions])
            ax.plot(pred[:, 0], pred[:, 1], pred[:, 2], '.', label='predicted catch points')
        ax.plot([actual.pos[0]], [actual.pos[1]], [actual.pos[2]], 'o', markersize=8, label='actual plane point')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        ax.set_title(f'Toss {toss_id}: trajectory and predicted catch points')
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir / f'toss_{toss_id:03d}_trajectory_3d.png', dpi=150)
        plt.close(fig)

        # y-vs-time plane crossing plot.
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.plot(t_meas, raw[:, 1], '.', label='raw y')
        ax.plot(t_meas, corrected[:, 1], '.', label='bias-corrected y')
        ax.plot(t_meas, filt[:, 1], '-', label='filtered y')
        ax.axhline(catch_y, linestyle='--', label='catch plane y')
        ax.axvline(actual.t - t0, linestyle=':', label='actual plane time')
        ax.set_xlabel('time since first measurement [s]')
        ax.set_ylabel('y [m]')
        ax.set_title(f'Toss {toss_id}: catch-plane crossing')
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir / f'toss_{toss_id:03d}_y_plane_crossing.png', dpi=150)
        plt.close(fig)

        if predictions:
            t_pred = np.array([p.t_pub - t0 for p in predictions])
            lead = np.array([p.lead_time for p in predictions])
            pred_pos = np.vstack([p.catch_pos for p in predictions])
            errors = np.linalg.norm(pred_pos - actual.pos.reshape(1, 3), axis=1)
            x_errors = pred_pos[:, 0] - actual.pos[0]
            z_errors = pred_pos[:, 2] - actual.pos[2]

            # Predicted x/z catch point convergence.
            fig, ax = plt.subplots(figsize=(9, 5))
            ax.plot(t_pred, pred_pos[:, 0], '.', label='predicted catch x')
            ax.axhline(actual.pos[0], linestyle='--', label='actual x at plane')
            ax.set_xlabel('prediction publish time since first measurement [s]')
            ax.set_ylabel('x [m]')
            ax.set_title(f'Toss {toss_id}: catch x convergence')
            ax.legend()
            fig.tight_layout()
            fig.savefig(plot_dir / f'toss_{toss_id:03d}_catch_x_convergence.png', dpi=150)
            plt.close(fig)

            fig, ax = plt.subplots(figsize=(9, 5))
            ax.plot(t_pred, pred_pos[:, 2], '.', label='predicted catch z')
            ax.axhline(actual.pos[2], linestyle='--', label='actual z at plane')
            ax.set_xlabel('prediction publish time since first measurement [s]')
            ax.set_ylabel('z [m]')
            ax.set_title(f'Toss {toss_id}: catch z convergence')
            ax.legend()
            fig.tight_layout()
            fig.savefig(plot_dir / f'toss_{toss_id:03d}_catch_z_convergence.png', dpi=150)
            plt.close(fig)

            # Error vs lead time. Lead time should decrease as the ball approaches the plane.
            fig, ax = plt.subplots(figsize=(9, 5))
            ax.plot(lead, errors, '.', label='3D catch-point error')
            ax.plot(lead, np.abs(x_errors), '.', label='abs x error')
            ax.plot(lead, np.abs(z_errors), '.', label='abs z error')
            ax.set_xlabel('predicted time until catch [s]')
            ax.set_ylabel('error [m]')
            ax.set_title(f'Toss {toss_id}: prediction error vs lead time')
            ax.legend()
            fig.tight_layout()
            fig.savefig(plot_dir / f'toss_{toss_id:03d}_error_vs_lead_time.png', dpi=150)
            plt.close(fig)

            # Time error plot.
            fig, ax = plt.subplots(figsize=(9, 5))
            time_err = np.array([p.t_catch - actual.t for p in predictions])
            ax.plot(t_pred, time_err, '.', label='predicted catch time - actual plane time')
            ax.axhline(0.0, linestyle='--')
            ax.set_xlabel('prediction publish time since first measurement [s]')
            ax.set_ylabel('time error [s]')
            ax.set_title(f'Toss {toss_id}: catch-time prediction error')
            ax.legend()
            fig.tight_layout()
            fig.savefig(plot_dir / f'toss_{toss_id:03d}_time_error.png', dpi=150)
            plt.close(fig)
#endregion 

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallCatchPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish_toss_async()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

