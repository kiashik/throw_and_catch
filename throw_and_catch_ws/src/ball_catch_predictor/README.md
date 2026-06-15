# ball_catch_predictor

ROS 2 Jazzy Python node for predicting a tossed ball's catch point at a fixed `y` plane in robot base frame.

## Topics

Subscribes:

- `/vision/ball_pose_robot` (`geometry_msgs/msg/PoseStamped`)

Publishes:

- `/vision/catch_point` (`geometry_msgs/msg/PoseStamped`)
- `/vision/ball_prediction_markers` (`visualization_msgs/msg/MarkerArray`) for RViz

## Build

```bash
cd ~/ros2_ws/src
cp -r /path/to/ball_catch_predictor_pkg ./ball_catch_predictor
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --packages-select ball_catch_predictor
source install/setup.bash
```

## Run

```bash
ros2 run ball_catch_predictor catch_predictor --ros-args \
  -p catch_y:=-0.40 \
  -p drag_coeff:=0.05 \
  -p prediction_rate_hz:=60.0
```

## Important parameters

- `catch_y`: catch plane in robot/base/world frame, default `-0.40` m.
- `drag_coeff`: linear drag coefficient in `1/s`, default `0.05`.
- `prediction_rate_hz`: timer prediction rate, clamped to at least `8.0` Hz.
- `publish_on_measurement`: also publish immediately when a new pose arrives, default `true`.
- `measurement_std_x_m`, `measurement_std_y_m`, `measurement_std_z_m`: tune from your vision noise.
- `process_accel_std_mps2`: tune higher if predictions lag real motion; lower if too jittery.
- `toss_timeout_s`: no-pose timeout that ends a toss and triggers CSV/plot generation.
- `plot_dir`: output directory for validation plots and CSV logs, default `~/ball_catch_plots`.

At the end of each toss, the node computes the measured/interpolated closest crossing at `y = catch_y`, compares every predicted catch point against that actual plane point, and saves CSV + PNG reports.
