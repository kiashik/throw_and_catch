#!/usr/bin/env python3
"""
ashik's adaption from chatgpt v3

Subscribes:
  /vision/ball_pose_robot    geometry_msgs/msg/PoseStamped

Publishes:
  /vision/catch_point        geometry_msgs/msg/PoseStamped
  /vision/ball_prediction_markers visualization_msgs/msg/MarkerArray

"""


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


# Tunable defaults requested for this project. 
DEFAULT_BALL_RADIUS_M = 0.03
DEFAULT_GRAVITY_MPS2 = 9.79728 # 9.80665
DEFAULT_DRAG_COEF_1PS = 0.05
DEFAULT_CATCH_Y_M = -0.50
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

PROCESS_ACCEL_STD = 2.0  # m/s^2. Tune this to balance responsiveness vs noise sensitivity.
INITIAL_VELOCITY_STD = 1.0  # m/s. Initial uncertainty in velocity estimate.

@dataclass
class MeasurementRecord:
    t: float
    receive_t: float
    raw: np.ndarray
    corrected: np.ndarray
    filtered: np.ndarray


@dataclass
class PredictionRecord:
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
class ActualCrossing:
    t: float
    pos: np.ndarray
    method: str


@dataclass
class TossData:
    toss_id: int
    measurements: List[MeasurementRecord]
    predictions: List[PredictionRecord]
    rejected_measurements: int
    catch_y: float
    catch_x_min: float
    catch_x_max: float
    catch_z_min: float
    catch_z_max: float



class LinearDragKalmanFilter:
    """Small, fast Kalman filter for 3D ball position/velocity."""

    def __init__(self):
        self.drag_coef = DEFAULT_DRAG_COEF_1PS
        self.measurement_variance = np.array([
            DEFAULT_MEASUREMENT_VARIANCE_M2[0],
            DEFAULT_MEASUREMENT_VARIANCE_M2[1],
            DEFAULT_MEASUREMENT_VARIANCE_M2[2],
        ], dtype=float)


        # Kalman filter process noise. how much to trust motion model. 
        # higher value: 
            # filter adapts faster, velocity estimate changes more easily
            # trajectory may become noisier'
        # lower value: filter trusts the physics model more
            # smoother estimate
            # may lag if the model is imperfect
        self.process_accel_std = PROCESS_ACCEL_STD  # Tune this to balance responsiveness vs noise sensitivity.
        self.initial_velocity_std = INITIAL_VELOCITY_STD  #Initial uncertainty in velocity estimate.

        self.x = np.zeros(6, dtype=float)                #  (6x1) state vector: [px, py, pz, vx, vy, vz]
        self.P = np.eye(6, dtype=float)                 # (6x6) state covariance matrix
        self.R = np.diag(self.measurement_variance)     # (3x3) measurement noise covariance (uses our camera noise characterization)
        self.H = np.zeros((3, 6), dtype=float)           # (3x6) measurement matrix maps state to measurement space [I_3x3, 0_3x3]. z = H @ x
        self.H[:, 0:3] = np.eye(3)                      # Measurement matrix: we measure position directly, so the top 3 columns are identity and the bottom 3 columns are zero.
        self.I = np.eye(6, dtype=float)
        self.initialized = False

        self.accel = np.array([0.0, 0.0, -DEFAULT_GRAVITY_MPS2], dtype=float)

        def initialize(self, position: np.ndarray) -> None:
            self.x[:] = 0.0
            self.x[0:3] = position
            pos_var = self.measurement_variance
            vel_var = np.ones(3, dtype=float) * (self.initial_velocity_std ** 2)
            self.P = np.diag(np.concatenate([pos_var, vel_var]))
            self.initialized = True
        
        def transition(self, dt: float) -> None:
            """
            same as method f from lab 2
            updates in-place self.x one dt forward:  x_next = Phi*x + b.
            Phi: discrete transition matrix (aka matrix A from state space representation)  
            b: offset vector (aka matrix B from state space representation).
            """
            phi = np.eye(6, dtype=float)
            b = np.zeros(6, dtype=float)

            e = np.exp(-self.drag_coef * dt)           # Velocity decay factor due to linear drag over time dt.
            b_v = (1.0 - e) / self.drag_coef         # Coefficient that maps velocity into position change under linear drag. if drag is 0, this limit to dt, consistent with no drag case.
            b_p = dt / self.drag_coef - (1.0 - e) / (self.drag_coef * self.drag_coef)  # Coefficient that maps constant acceleration into position change under linear drag.

            # b[0:3] = gravity contribution to position
            # b[3:6] = gravity contribution to velocity
            phi[0:3, 3:6] = b_v * np.eye(3)     #  (6x6) state transition matrix for linear drag model. It maps the current state to the next state, excluding the constant gravity offset.
            phi[3:6, 3:6] = e * np.eye(3)
            b[0:3] = b_p * self.accel
            b[3:6] = b_v * self.accel
            
            self.x = phi @ self.x + b
        
        def predict(self, dt: float) -> None:
            self.transition(dt)     # state prediction using linear drag model

            # Approximate continuous white acceleration noise. This lets the filter
            # absorb imperfect drag, small spin effects, and residual measurement
            # systematics without making the state too jittery.
            q = self.process_accel_std ** 2     # process acceleration variance.
            Q = np.zeros((6, 6), dtype=float)
            Q[0:3, 0:3] = (dt ** 4 / 4.0) * q * np.eye(3)       # see lec 5, pg 19 and chat gpt honestly
            Q[0:3, 3:6] = (dt ** 3 / 2.0) * q * np.eye(3)
            Q[3:6, 0:3] = Q[0:3, 3:6].T
            Q[3:6, 3:6] = (dt ** 2) * q * np.eye(3)

            self.P = self.phi @ self.P @ self.phi.T + self.Q    # covariance prediction
            self.P = 0.5 * (self.P + self.P.T)                  # enforce symmetry to prevent numerical issues. due to the way we construct Q, P should be symmetric in theory, but numerical errors can cause it to become slightly non-symmetric over time, which can lead to issues with the Kalman gain calculation and filter stability. this enforces symmetry at each step to mitigate that.


        def update(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
            """Kalman position update. Returns innovation and innovation covariance.
            z is the measurement vector [px, py, pz].T
            """
            S = self.H @ self.P @ self.H.T + self.R

            # K = P H.T inv(S)
            K = self.P @ self.H.T @ np.linalg.inv(S)

            innovation = z - self.H @ self.x
            self.x = self.x + K @ innovation
            self.P = (self.I - K @ self.H) @ self.P
            return innovation, S

class CatchPredictorNodeV3(Node):
    def __init__(self):
        super().__init__("catch_predictor_v3")

        self.kf = LinearDragKalmanFilter()

        catch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(depth=1)

        self.catch_topic = "/vision/catch_point"
        self.marker_topic = "/vision/ball_prediction_markers"
        self.input_topic = "/vision/ball_pose_robot"

        self.catch_pub = self.create_publisher(PoseStamped, self.catch_topic, catch_qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)
        self.sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )

        self.prediction_horizon_s = 1.50    # Maximum future time that the node will search for a catch-plane intersection. If the ball will not reach y = catch_y within this time, no catch point is produced.
        self.prediction_rate_hz = 30.0  
        self.trajectory_dt_s = 0.02          # # Step size used to generate the RViz predicted trajectory line. Smaller value: smoother RViz line, more CPU. num of points in rviz = prediction_horizon_s / trajectory_dt_s. only used for rviz visualization
