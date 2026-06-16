<!-- **DO THE FOLLOWING TO GET ALL REQUIRED REPOSITORIES INCLUDING SUBMODULES AND RECURSIVE MODULES(see setup guide for OMY):**

```
git clone https://github.com/kiashik/throw_and_catch.git
cd throw_and_catch
git submodule update --init --recursive
```

How to update a specific submodule (replace submodule name):
```
cd throw_and_catch_ws/src/image_pipeline
git pull origin jazzy    # specific branch name is jazzy
cd ../../..
git add throw_and_catch_ws/src/image_pipeline
git commit -m "Update image_pipeline submodule"
```

Remember to manually modify the appropriate files:
```	
open_manipulator_bringup/config/omy_f3m/hardware_controller_manager.yaml
    allow_nonzero_velocity_at_trajectory_end: true    # ashik added this. Allows the controller to end a trajectory with nonzero velocity, which may be important for our use case of 
    # tracking a moving target. If false, the controller will try to decelerate to zero velocity at the end of each trajectory, which can cause issues when trying to track a moving target.

# May need to make small modification to the following two files:
open_manipulator_bringup/launch/omy_f3m_gazebo.launch.py
modified:   open_manipulator_moveit_config/launch/omy_f3m_moveit.launch.py
``` -->

# Robotic Arm Catch

ROS 2 workspace for a senior project that detects a tossed tennis ball, estimates its 3D position with an Intel RealSense D455, predicts a future catch point, and commands an OpenMANIPULATOR-Y robotic arm toward the predicted interception point.

**Authors**: Ashik Islam, Kayla Go-Oco, Raegan Gritzmacher <br>
**Advisor**: Siavash Farzan

## Video Demos 
### Demo with overlaid ball trajectory 
https://github.com/user-attachments/assets/1fe64234-4e06-4e40-97e2-6f174bede03d

### Demo without ball trajectory
https://github.com/user-attachments/assets/0cb73a11-aef0-4745-8f79-20b0b4260299

## What is included

This repository contains a ROS 2 workspace at `throw_and_catch_ws/`.

### Custom packages

| Package | Purpose |
| --- | --- |
| `vision` | RealSense camera launch/configuration, YOLO tennis ball detection, depth/PnP ball pose estimation, AprilTag camera-to-robot calibration, and camera-to-robot pose conversion support. |
| `ball_catch_predictor` | Linear-drag Kalman filter for ball state estimation, future trajectory prediction, catch-plane intersection prediction, RViz markers, and optional CSV/plot logging. |
| `my_motion_planner2` | Main MoveIt Servo configuration and nodes for real-time Cartesian control of the OpenMANIPULATOR-Y. |
| `move_arm_to` | MoveIt/OMPL test nodes, simple MoveIt-based catching experiments, AprilTag/pose helper nodes, and earlier control examples. |
| `my_motion_planner` | Ball tracking experiments, and camera-to-robot pose conversion node used by the vision launch file. |
| `charcterize_sensor` | RealSense sensor characterization node used to estimate measurement noise and bias. |

### Submodules

The repository also uses submodules for external dependencies:

- `throw_and_catch_ws/src/open_manipulator`
- `throw_and_catch_ws/src/physical_ai_tools`
- `throw_and_catch_ws/src/realsense-ros`
- `throw_and_catch_ws/src/apriltag_ros`

-------------------------

## Hardware used

- OpenMANIPULATOR-Y, OMY F3M configuration not including the Intel RealSense D405 depth camera
- Intel RealSense D455 RGB-D camera
- AprilTags for eye-to-hand camera-to-robot calibration
- User PC: CUDA-capable computer for YOLO inference, tested with an Lenovo Legion Slim 7 equipped with a NVIDIA GeForce RTX 3060 Laptop GPU running Ubuntu 24.04.4 LTS 
- Tennis ball

YOLO inference can be run on non CUDA-capable devices as long as the appropriate YOLO model format is used in `throw_and_catch_ws/src/vision/vision/ball_detector.py`

## Physical Hardware Setup

This project uses two computers when running on the physical OpenMANIPULATOR-Y hardware.

### USER PC

The USER PC is the external development computer that runs:

- RealSense camera drivers
- Ball detection and pose estimation
- Trajectory prediction
- Motion planning and tracking nodes
- RViz visualization and debugging tools

### ROBOT PC

The ROBOT PC is located onboard the OpenMANIPULATOR-Y platform and runs:

- Robot hardware drivers
- Dynamixel communication
- MoveIt Servo
- Robot state publishing
- Hardware bringup

### Network Requirements

The USER PC and ROBOT PC must be connected to the same network.

For physical hardware operation, an Ethernet connection between the USER PC and ROBOT PC is recommended.

Both computers must use the same ROS 2 Domain ID: 

```bash
export ROS_DOMAIN_ID=25
```
Verify that the same value is configured on both machines before launching the system.

For complete OpenMANIPULATOR-Y hardware setup instructions, refer to the official ROBOTIS documentation:

https://ai.robotis.com/omy/setup_guide_omy.html <br>

https://docs.robotis.com/docs/systems/omy/introduction


-------------------------

## Software requirements

This project was developed for ROS 2 Jazzy on Ubuntu 24.04.4 LTS. Install ROS 2 Jazzy and the OMY/OpenMANIPULATOR dependencies first, following the ROBOTIS OMY setup guide.

Dependencies include:

- ROS 2 Jazzy
- MoveIt 2 / MoveIt Servo
- OpenMANIPULATOR-Y ROS 2 packages
- Intel RealSense SDK and ROS 2 wrapper
- `apriltag_ros`
- Python packages: `numpy`, `matplotlib`, `opencv-python`, `pyrealsense2`, `torch`, `ultralytics`


# Install ROS dependencies
```bash
cd throw_and_catch_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

# Install Python Dependencies

It is highly recommended to use a Python virtual environment so the project dependencies do not conflict with system Python packages.

Since this is a ROS 2 workspace, create the virtual environment with access to system site packages. This allows the virtual environment to still use ROS 2 Python packages installed through `apt`.

```bash
cd throw_and_catch/throw_and_catch_ws

# Create a virtual environment with access to system Python packages
python3 -m venv ros_yolo_venv --system-site-packages

# Activate the virtual environment
source ros_yolo_venv/bin/activate

# Upgrade pip
python3 -m pip install --upgrade pip

# Install Python dependencies
python3 -m pip install -r requirements.txt
```
> Note: Remember to activate the virtual environment before running Python-based scripts or nodes that depend on packages from `requirements.txt`. <br>
> To activate the virtual environment later, run: `source ~/throw_and_catch/throw_and_catch_ws/ros_yolo_venv/bin/activate`
--------------------

## Clone the repository

Use the `main` branch and initialize all submodules recursively.

```bash
git clone https://github.com/kiashik/throw_and_catch.git
cd throw_and_catch
git submodule update --init --recursive
```

### If the AprilTag submodule fails to clone

The `apriltag_ros` submodule may use an SSH GitHub URL. If the submodule fails because SSH keys are not configured, change that submodule to HTTPS and sync again:

```bash
git config submodule.throw_and_catch_ws/src/apriltag_ros.url https://github.com/kiashik/apriltag_ros.git
git submodule sync --recursive
git submodule update --init --recursive
```

## Manual OMY configuration changes

Before running on hardware, manually check the OMY controller configuration files inside the OpenMANIPULATOR submodule.

### Required controller change

Add or confirm this `ros__parameters` under `arm_controller` to `throw_and_catch_ws/src/open_manipulator/open_manipulator_bringup/config/omy_f3m/hardware_controller_manager.yaml`. Make sure to apply this change to both USER PC (for simulation) and ROBOT PC (for hardware). 

```yaml
allow_nonzero_velocity_at_trajectory_end: true
```

This is needed for MoveIt Servo commands to actuate the arm. Otherwise, the lower-level controller may reject Servo commands because Servo trajectory endpoints are not necessarily zero velocity.

### Adjust Robot Velocity and Acceleration Limits

The robot velocity and acceleration limits can be adjusted in the MoveIt joint limits configuration file:

```bash
throw_and_catch_ws/src/open_manipulator/open_manipulator_moveit_config/config/omy_f3m/joint_limits.yaml
```

## Build the workspace

From a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd throw_and_catch/throw_and_catch_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build
source install/setup.bash
```

If you use a Python virtual environment for YOLO/Ultralytics, activate it before running the vision nodes and make sure it can access the ROS 2 Python packages. If `colcon build` fails inside the venv, consider building outside the venv.

--------------------------------------------------------------------------------------------------------------------------------------------------------------------
# Running the system: Ball Tracker

The system is modular. Run each subsystem in a separate terminal after sourcing the workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd throw_and_catch/throw_and_catch_ws
source install/setup.bash
export ROS_DOMAIN_ID=25
```
### Terminal 0. Bring up the physical robot

This step is only required when running on physical hardware.

All commands in this section are executed on the **ROBOT PC**.

SSH into the robot:

```bash
ssh root@omy-<ROBOT_SERIAL_NUMBER>.local
```

Enter the OMY Docker container:

```bash
cd /data/docker/open_manipulator
./docker/container.sh enter
```

Source the ROS 2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

If the robot is currently packed, unpack it first:

```bash
ros2 launch open_manipulator_bringup omy_f3m_unpack.launch.py
```

> [**DANGER**]
> After the initial setup, the unpacking script must be executed before operating the robot to prevent self-collision.
> Run this script **only when the robot is in the packed posture**. Running it from any other orientation may cause damage to the robot.
Then bring up the robot hardware:

```bash
ros2 launch open_manipulator_bringup omy_f3m.launch.py
```

The robot should now be visible on the ROS 2 network and ready to receive commands from the USER PC. Verify by running `ros2 topic list` on a terminal in USER PC.

For additional hardware setup details, see:

https://ai.robotis.com/omy/setup_guide_omy.html

### Aside: Servo requires that the robot starts in a safe pose
Move the arm to a safe pose by running the following commands from any terminal on the correct ROS network:
```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],
  points: [{
    positions: [-1.5708, 0.0, 1.5708, -1.5708, 1.50708, 0.0],
    time_from_start: {sec: 3}
  }]
}" # goes to home pose
```
It is highly recommended to keep this terminal open and move the arm to this pose or another safe pose before running an experiment.


### Terminal 1. Start the RealSense D455 camera in a terminal on USER PC

```bash
ros2 launch vision rs_launch_custom_params.launch.py
```

This launches the RealSense ROS 2 wrapper using the project camera configuration in `throw_and_catch_ws/src/vision/config/rs_launch_config.yaml`.

The configuration enables color and depth streams, aligns depth to color, enables RGB-D output, and uses the `848x480x60` color/depth profiles.

### 2. Perform camera-to-robot calibration in a terminal on USER PC

Run this when the camera or robot base has moved, or when the calibration file needs to be regenerated. Before running the calibration script, ensure that enough AprilTags are clearly visible from the camera.

```bash
ros2 launch vision cal_cam_robot.launch.py
```

This launch file starts AprilTag detection, tag visualization, and the `cal_cam_robot` node. The calibration node computes the eye-to-hand transform from the RealSense camera frame to the robot base frame using AprilTag correspondences, then saves:

```text
install/vision/share/vision/config/camera_robot_calibration.npy
install/vision/share/vision/config/camera_robot_calibration.txt
```

### Terminal 3. Start ball detection and 3D ball pose estimation in a terminal on USER PC

```bash
ros2 launch vision ball_pose.launch.py pose_estimation_method:=depth visualize:=true
```

The depth mode is the main mode for this project. It starts:

- `vision/ball_detector`
- `vision/ball_pose_estimation_depth`
- `my_motion_planner/convert_pose_cam_to_rob`

Main topics:

| Topic | Type | Description |
| --- | --- | --- |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense RGB image input. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth image aligned to the RGB image. |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics. |
| `/vision/ball_detections` | `vision_msgs/Detection2D` | YOLO ball detection in image coordinates. |
| `/vision/ball_pose_cam` | `geometry_msgs/PoseStamped` | Estimated 3D ball pose in the camera frame. |
| `/vision/ball_pose_robot` | `geometry_msgs/PoseStamped` | Ball pose transformed into the robot base frame. |

The current YOLO detector loads this model from the installed `vision` package share directory:

```text
throw_and_catch_ws/src/vision/vision/yolo_models/yolo26n_my_ds_v2_best.engine
```

### Terminal 4. Launch servo in a terminal on USER PC
Ensure the robot is in a safe pose(not collided or near a singularity).

Simulation-oriented servo launch:

```bash
ros2 launch my_motion_planner2 my_servo2.launch.py use_sim_time:=true
```

Hardware-oriented servo launch:

```bash
ros2 launch my_motion_planner2 my_servo2.launch.py use_sim_time:=false
```


### Terminal 5. Tracking a ball in hand using MoveIt Servo


Explicitly pass the ball topic to the `ball_tracker_servo` node and run it:
```bash 
ros2 run my_motion_planner2 ball_tracker_servo --ros-args -p target_topic:=/vision/ball_pose_robot
```

With the RealSense camera running, hold a tennis ball in front of the robot and launch the ball pose estimation pipeline. The system should detect the tennis ball, estimate its 3D position, and publish the ball pose in both the camera frame and robot frame. The robot end-effector should continuously track the predicted catch point generated from the observed ball position. The motion should appear smooth and responsive, with the end-effector following the ball as it moves through the workspace.

> Note: The default value of the `target_topic` parameter can be changed in the `ball_tracker_servo` node source code as desired. For example, set the default to `/vision/ball_pose_robot` for real-time ball tracking or `/vision/catch_point` for predicted catch point tracking.

Useful debugging commands:

```bash
ros2 topic echo /vision/ball_pose_cam

ros2 topic echo /vision/ball_pose_robot
```

If the ball pose is not being published, verify that:

1. The camera node is running.
2. The USER PC and ROBOT PC are connected to the same network.
3. Both computers use the same ROS_DOMAIN_ID.
4. The camera-to-robot calibration file has been generated.
5. Rviz2 may be used to visualize the ball's position in camera or robot frame. This is a great way to verify that the estimated ball's position is correct.

--------------------------------------------------------------------------------------------------------------------------------------------------------------------
# Running the system: Ball Catcher
Keep in mind, the arm will not actually catch the ball as the arm is incapable of reacting fast enough.

### Terminal 0 to 4: Follow the same steps in **Running the system: Ball Tracker**.
The robot is brought up, ball pose in robot frame is publishing to `/vision/ball_pose_robot` and Servo is launched.

### Terminal 5. Start catch-point prediction

Recommended direct run command:

```bash
ros2 run ball_catch_predictor catch_predictor_v3 --ros-args \
  -p catch_y:=-0.45 \
  -p prediction_rate_hz:=60.0 \
  -p drag_coeff:=0.05
```

The predictor subscribes to:

```text
/vision/ball_pose_robot
```

and publishes:

```text
/vision/catch_point
/vision/ball_prediction_markers
```

The predictor estimates the full ball state with a linear-drag Kalman filter, predicts the future trajectory, and publishes the future intersection with a fixed catch plane.

A launch file also exists:

```bash
ros2 launch ball_catch_predictor catch_predictor.launch.py
```

### Visualization: use RViz2 to visualize the ball's current and estimated position
Start Rviz with the provided config file:
```bash
rviz2 -d ~/throw_and_catch/catch_config.rviz
```

--------------------------------------------------------------------------------------------------------------------------------------------------------------------
## Optional/testing commands

### Run the simple MoveIt catching experiment

```bash
ros2 launch move_arm_to catch_ball_simple.launch.py use_sim_time:=true
```

For real hardware:

```bash
ros2 launch move_arm_to catch_ball_simple.launch.py use_sim_time:=false
```

### Run MoveIt waypoint/OMPL experiments

```bash
ros2 launch move_arm_to my_omy_moveit.launch.py use_sim_time:=false
```

### Run RealSense sensor characterization

```bash
ros2 run charcterize_sensor characterize_rs
```


Useful debugging commands:

```bash
ros2 topic echo /vision/ball_pose_robot
```

```bash
ros2 topic echo /vision/catch_point
```

```bash
rviz2 -d ~/throw_and_catch/catch_config.rviz
```

Verify the following:

1. The tennis ball is detected consistently.
2. `/vision/ball_pose_robot` publishes valid robot-frame ball positions.
3. `/vision/catch_point` publishes predicted interception points.
4. RViz displays the predicted trajectory markers.
5. The robot end-effector follows the published catch point.

If the robot does not move, verify that:

* The robot hardware has been successfully brought up.
* MoveIt Servo is running.
* `ball_tracker_servo` is running.
* The USER PC and ROBOT PC share the same `ROS_DOMAIN_ID`.
* The camera-to-robot calibration file exists and is valid.




## Suggested full run order

Use separate terminals for each step.

1. Bring up the OMY robot or Gazebo simulation.
2. Start the RealSense camera:

   ```bash
   ros2 launch vision rs_launch_custom_params.launch.py
   ```

3. If needed, recalibrate the camera-to-robot transform:

   ```bash
   ros2 launch vision cal_cam_robot.launch.py
   ```

4. Start ball detection and pose estimation:

   ```bash
   ros2 launch vision ball_pose.launch.py pose_estimation_method:=depth visualize:=false
   ```

5. Start catch-point prediction (if catching):

   ```bash
   ros2 run ball_catch_predictor catch_predictor_v3 --ros-args -p catch_y:=-0.45
   ```
6. Start ball tracking (run only 1 at a time):
   If tracking ball:
   ```bash
      ros2 run my_motion_planner2 ball_tracker_servo --ros-args -p target_topic:=/vision/ball_pose_robot    
    ```
   If catching ball:
   ```bash
      ros2 run my_motion_planner2 ball_tracker_servo --ros-args -p target_topic:=/vision/catch_point    
    ```   

8. Start MoveIt Servo / robot tracking:

   ```bash
   ros2 launch my_motion_planner2 my_servo2.launch.py use_sim_time:=false
   ```


## Troubleshooting

### Submodules are missing

```bash
git submodule update --init --recursive
```

### ROS cannot find a package

Rebuild and source the workspace:

```bash
cd throw_and_catch/throw_and_catch_ws
colcon build --symlink-install
source install/setup.bash
```

### YOLO model not found

Confirm the model exists here before building:

```text
throw_and_catch_ws/src/vision/vision/yolo_models/yolo26n_my_ds_v2_best.engine
```

Then rebuild the `vision` package:

```bash
colcon build --packages-select vision --symlink-install
source install/setup.bash
```
If the path to the YOLO model is different on your device, then provide the correct path to `/throw_and_catch_ws/src/vision/vision/ball_detector`.


### Calibration file not found

Run the AprilTag calibration step again:

```bash
ros2 launch vision cal_cam_robot.launch.py
```

Then confirm that the generated `camera_robot_calibration.npy` exists in the installed `vision` package's share directory.

### Ball pose is detected in the camera frame but not the robot frame

Check that the camera-to-robot conversion node is running and that it can load `camera_robot_calibration.npy`.

```bash
ros2 topic echo /vision/ball_pose_cam
ros2 topic echo /vision/ball_pose_robot
```

### Catch point is not published

Check that `/vision/ball_pose_robot` is publishing and that the predicted trajectory crosses the configured catch plane:

```bash
ros2 topic echo /vision/ball_pose_robot
ros2 topic echo /vision/catch_point
```

Try adjusting the catch plane:

```bash
ros2 run ball_catch_predictor catch_predictor --ros-args -p catch_y:=-0.45
```

# Misc Development Documentation

Additional development notes and project logbooks are included below:

- [Vision Logbook](docs/Vision_Logbook.pdf)
- [Motion Planning Logbook](docs/Motion_Planning_Logbook.pdf)
- [Noise Characterization Logbook](docs/Ball_catch_predictor_Logbook.pdf)
