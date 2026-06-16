<!-- **DO THE FOLLOWING TO GET ALL REQUIRED REPOS INCUDING SUBMODULSAND RECURSIVE MODULES(see setup guide for OMY):**

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

Remember to manually modify the the apprriate files:
```	
open_manipulator_bringup/config/omy_f3m/hardware_controller_manager.yaml
    allow_nonzero_velocity_at_trajectory_end: true    # ashik added this. Allows the controller to end a trajectory with nonzero velocity, which maybe important for our use case of 
    # tracking a moving target. If false, the controller will try to decelerate to zero velocity at the end of each trajectory, which can cause issues when trying to track a moving target.

# May need to make small modification to the followign two files:
open_manipulator_bringup/launch/omy_f3m_gazebo.launch.py
modified:   open_manipulator_moveit_config/launch/omy_f3m_moveit.launch.py
``` -->

# Video demo
https://github.com/user-attachments/assets/1fe64234-4e06-4e40-97e2-6f174bede03d

# Video Demo 2
[Watch demo video](assets/OMY_ball_interception_demo.MOV)

# Robotic Arm Catch

ROS 2 workspace for a senior project that detects a tossed tennis ball, estimates its 3D position with an Intel RealSense D455, predicts a future catch point, and commands an OpenMANIPULATOR-Y robotic arm toward the predicted interception point.


## What is included

This repository contains a ROS 2 workspace at `throw_and_catch_ws/`.

### Custom packages

| Package | Purpose |
| --- | --- |
| `vision` | RealSense camera launch/configuration, YOLO tennis ball detection, depth/PnP ball pose estimation, AprilTag camera-to-robot calibration, and camera-to-robot pose conversion support. |
| `ball_catch_predictor` | Linear-drag Kalman filter for ball state estimation, future trajectory prediction, catch-plane intersection prediction, RViz markers, and optional CSV/plot logging. |
| `my_motion_planner2` | Main MoveIt Servo configuration and nodes for real-time Cartesian control of the OpenMANIPULATOR-Y. |
| `move_arm_to` | MoveIt/OMPL test nodes, simple MoveIt-based catching experiments, AprilTag/pose helper nodes, and earlier control examples. |
| `my_motion_planner` | Earlier servo-control experiments, fake pose publishers, ball tracking experiments, and camera-to-robot pose conversion node used by the vision launch file. |
| `charcterize_sensor` | RealSense sensor characterization node used to estimate measurement noise and bias. |

### Submodules

The repository also uses submodules for external dependencies:

- `throw_and_catch_ws/src/open_manipulator`
- `throw_and_catch_ws/src/physical_ai_tools`
- `throw_and_catch_ws/src/realsense-ros`
- `throw_and_catch_ws/src/apriltag_ros`

## Hardware used

- OpenMANIPULATOR-Y, OMY F3M configuration
- Intel RealSense D455 RGB-D camera
- AprilTags for eye-to-hand camera-to-robot calibration
- CUDA-capable computer for YOLO inference, tested with an NVIDIA GeForce RTX 3060 Laptop GPU
- Tennis ball

YOLO inference can be run on non CUDA-capable devices as long as the approirate YOLO model format is useed in `throw_and_catch_ws/src/vision/vision/ball_detector.py`

## Software requirements

This project was developed for ROS 2 Jazzy on Ubuntu. Install ROS 2 Jazzy and the OMY/OpenMANIPULATOR dependencies first, following the ROBOTIS OMY setup guide.

Common dependencies include:

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
# Install Python dependencies
Highly recoomend using a python virtual enviroment. 
TODO: add instruction to create venv
```bash
cd /throw_and_catch
python3 -m pip install -r requirements.txt
````

## Clone the repository

Use the `ashik_ubuntu` branch and initialize all submodules recursively.

```bash
git clone -b ashik_ubuntu https://github.com/kiashik/throw_and_catch.git
cd throw_and_catch
git submodule update --init --recursive
```

Equivalent one-line clone:

```bash
git clone -b ashik_ubuntu --recurse-submodules https://github.com/kiashik/throw_and_catch.git
```

### If the AprilTag submodule fails to clone

The `apriltag_ros` submodule may use an SSH GitHub URL. If the submodule fails because SSH keys are not configured, change that submodule to HTTPS and sync again:

```bash
git config submodule.throw_and_catch_ws/src/apriltag_ros.url https://github.com/kiashik/apriltag_ros.git
git submodule sync --recursive
git submodule update --init --recursive
```

## Updating submodules

To update a specific submodule, enter the submodule directory, pull the desired branch, then commit the updated submodule pointer from the parent repository.

Example:

```bash
cd throw_and_catch_ws/src/<submodule_name>
git pull origin <branch_name>
cd ../../..
git add throw_and_catch_ws/src/<submodule_name>
git commit -m "Update <submodule_name> submodule"
```

## Manual OMY configuration changes

Before running on hardware, manually check the OMY controller configuration files inside the OpenMANIPULATOR submodule.

### Required controller change

Edit:

```text
throw_and_catch_ws/src/open_manipulator/open_manipulator_bringup/config/omy_f3m/hardware_controller_manager.yaml
```

Add or confirm this parameter under the appropriate joint trajectory controller configuration:

```yaml
allow_nonzero_velocity_at_trajectory_end: true
```

This was added for the ball-tracking use case so the controller does not force every trajectory endpoint to decelerate to zero velocity.

### Other files that may need local edits

Check these files if simulation or MoveIt bringup does not match the project setup:

```text
throw_and_catch_ws/src/open_manipulator/open_manipulator_bringup/launch/omy_f3m_gazebo.launch.py
throw_and_catch_ws/src/open_manipulator/open_manipulator_moveit_config/launch/omy_f3m_moveit.launch.py
```

TODO: need to add the exact final diffs for these two launch files.

## Build the workspace

From a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd throw_and_catch/throw_and_catch_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

If you use a Python virtual environment for YOLO/Ultralytics, activate it before running the vision nodes and make sure it can access the ROS 2 Python packages.

TODO: Add the final virtual environment setup command, if the project requires one.

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
export ROS_DOMAIN_ID=30
```
Verify that the same value is configured on both machines before launching the system.

For complete OpenMANIPULATOR-Y hardware setup instructions, refer to the official ROBOTIS documentation:

https://ai.robotis.com/omy/setup_guide_omy.html

## Running the system

The system is modular. Run each subsystem in a separate terminal after sourcing the workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd throw_and_catch/throw_and_catch_ws
source install/setup.bash
```
### 0. Bring up the physical robot

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

Then launch the robot hardware:

```bash
ros2 launch open_manipulator_bringup omy_f3m.launch.py
```

The robot should now be visible on the ROS 2 network and ready to receive commands from the USER PC.

For additional hardware setup details, see:

https://ai.robotis.com/omy/setup_guide_omy.html


### 1. Start the RealSense D455 camera

```bash
ros2 launch vision rs_launch_custom_params.launch.py
```

This launches the RealSense ROS 2 wrapper using the project camera configuration in:

```text
throw_and_catch_ws/src/vision/config/rs_launch_config.yaml
```

The configuration enables color and depth streams, aligns depth to color, enables RGB-D output, and uses the `848x480x60` color/depth profiles.

### 2. Perform camera-to-robot calibration

Run this when the camera or robot base has moved, or when the calibration file needs to be regenerated.

```bash
ros2 launch vision cal_cam_robot.launch.py
```

This launch file starts AprilTag detection, tag visualization, and the `cal_cam_robot` node. The calibration node computes the eye-to-hand transform from the RealSense camera frame to the robot base frame using AprilTag correspondences, then saves:

```text
install/vision/share/vision/config/camera_robot_calibration.npy
install/vision/share/vision/config/camera_robot_calibration.txt
```

TODO: Confirm the final installed calibration path on your machine. `install/vision/share/vision/config/` 

### 3. Start ball detection and 3D ball pose estimation

```bash
ros2 launch vision ball_pose.launch.py pose_estimation_method:=depth visualize:=false
```

Useful variations:

```bash
ros2 launch vision ball_pose.launch.py pose_estimation_method:=depth visualize:=true
ros2 launch vision ball_pose.launch.py pose_estimation_method:=pnp
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
yolo_models/yolo26n_my_ds_v2_best.engine
```

#### Tracking a ball in hand

This launch file can be used to verify that the vision system is operating correctly before attempting full ball-catching experiments.

With the RealSense camera running, hold a tennis ball in front of the camera and launch the ball pose estimation pipeline. The system should detect the tennis ball, estimate its 3D position, and publish the ball pose in both the camera frame and robot frame.

Useful debugging commands:

```bash
ros2 topic echo /vision/ball_pose_cam

ros2 topic echo /vision/ball_pose_robot

rviz2
```

If the ball pose is not being published, verify that:

1. The camera node is running.
2. The USER PC and ROBOT PC are connected to the same network.
3. Both computers use the same ROS_DOMAIN_ID.
4. The camera-to-robot calibration file has been generated.


### 4. Start catch-point prediction

Recommended direct run command:

```bash
ros2 run ball_catch_predictor catch_predictor --ros-args \
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

### 5. Start MoveIt Servo control

Simulation-oriented servo launch:

```bash
ros2 launch my_motion_planner2 my_servo2.launch.py use_sim_time:=true
```

Hardware-oriented servo launch:

```bash
ros2 launch my_motion_planner2 my_servo2.launch.py use_sim_time:=false
```
<!-- 
TODO: Confirm the final hardware launch behavior. In the current branch, the real-hardware MoveIt bringup include is present but commented out in `my_servo2.launch.py`, so the hardware bringup may need to be launched separately. -->
TODO: add hardware bring up instruction or link to omy wbsite

After Servo is running, start the ball tracking servo node if it is not already included in your launch sequence:

```bash
ros2 run my_motion_planner2 ball_tracker_servo
```

<!-- TODO: Confirm whether the final tracking node should be `my_motion_planner2 ball_tracker_servo`, `my_motion_planner ball_tracker_servo`, or a newer controller node. -->

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

TODO: Add the required setup for the characterization test, including where to place the ball, how many samples to collect, and where results are saved. link to pdf

### Verify Ball Tracking

Before attempting ball-catching experiments, verify that the complete tracking pipeline is functioning correctly.

Start the required system components:

```bash
ros2 launch vision rs_launch_custom_params.launch.py
```

```bash
ros2 launch vision ball_pose.launch.py pose_estimation_method:=depth visualize:=false
```

```bash
ros2 run ball_catch_predictor catch_predictor --ros-args -p catch_y:=-0.45
```

```bash
ros2 launch my_motion_planner2 my_servo2.launch.py use_sim_time:=false
```

```bash
ros2 run my_motion_planner2 ball_tracker_servo
```

With all nodes running, slowly move a tennis ball throughout the camera field of view.

The robot end-effector should continuously track the predicted catch point generated from the observed ball position. The motion should appear smooth and responsive, with the end-effector following the ball as it moves through the workspace.

Useful debugging commands:

```bash
ros2 topic echo /vision/ball_pose_robot
```

```bash
ros2 topic echo /vision/catch_point
```

```bash
rviz2
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

5. Start catch-point prediction:

   ```bash
   ros2 run ball_catch_predictor catch_predictor --ros-args -p catch_y:=-0.45
   ```

6. Start MoveIt Servo / robot tracking:

   ```bash
   ros2 launch my_motion_planner2 my_servo2.launch.py use_sim_time:=false
   ros2 run my_motion_planner2 ball_tracker_servo
   ```

TODO: Replace this with one final launch command once all subsystem launches are integrated.

## Safety notes

This project moves a physical robotic arm toward dynamically changing targets. Before running on hardware:

- Start at reduced speed.
- Keep the workspace clear.
- Use conservative joint, velocity, acceleration, and workspace limits.
- Verify emergency stop behavior.
- Test each subsystem in simulation before hardware tests.
- Do not stand inside the robot workspace during motion.

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

### Calibration file not found

Run the AprilTag calibration step again:

```bash
ros2 launch vision cal_cam_robot.launch.py
```

Then confirm that the generated `camera_robot_calibration.npy` exists in the installed `vision` package share directory.

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

## Repository notes and TODOs

- TODO: Add a single integrated launch file for the complete system.
- TODO: Add final hardware bringup commands for the OMY arm.
- TODO: Add exact Python virtual environment setup.
- TODO: Add final YOLO training/export instructions.
- TODO: Add final calibration procedure with AprilTag size, IDs, and placement.
- TODO: Add example RViz configuration for the full system.
- TODO: Add a topic diagram showing camera, vision, prediction, and motion-control data flow.
