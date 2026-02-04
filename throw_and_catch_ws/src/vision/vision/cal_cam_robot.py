#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import TransformListener, Buffer
# import time
import os
from ament_index_python.packages import get_package_share_directory

import matplotlib.pyplot as plt

# ensure realsense2_camera is running and apriltag_ros is detecting tags.

class CalCamRobot(Node):
    def __init__(self):
        super().__init__('cal_cam_robot')
        
        # Define robot frame positions of each AprilTag (in robot base frame)
        # TODO: Update these with your actual robot measurements
        self.robot_points = {
            0: np.array([0.2370, -0.1450, 0.0]),      # tag_0 position in robot base frame
            1: np.array([0.1470, -0.1450, 0.0]),      # tag_1
            2: np.array([0.2370, -0.0550, 0.0]),      # tag_2
            3: np.array([0.1470, -0.0550, 0.0]),      # tag_3
            4: np.array([0.2370, 0.04500, 0.0]),      # tag_4
            5: np.array([0.1470, 0.04500, 0.0]),      # tag_5

            6: np.array([0.4535, -0.1445, 0.0]),      # tag_6
            7: np.array([0.3635, -0.1445, 0.0]),      # tag_7
            8: np.array([0.4535, -0.0545, 0.0]),      # tag_8
            9: np.array([0.3635, -0.0545, 0.0]),      # tag_9
            10: np.array([0.4535, 0.04550, 0.0]),     # tag_10
            11: np.array([0.3635, 0.04550, 0.0]),     # tag_11

            12: np.array([0.4530, -0.2350, 0.0]),     # tag_12
            13: np.array([0.4530, -0.3250, 0.0]),     # tag_13
            14: np.array([0.3630, -0.2350, 0.0]),     # tag_14
            15: np.array([0.3630, -0.3250, 0.0]),     # tag_15
            16: np.array([0.2730, -0.2350, 0.0]),     # tag_16
            17: np.array([0.2730, -0.3250, 0.0]),     # tag_17
        }
        
        # TF listener to get camera -> tag transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Camera-Robot Calibration Node Started")
        self.get_logger().info(f"Loaded {len(self.robot_points)} robot points")
        
        self.timer = self.create_timer(2.0, self.compute_calibration)
        self.calibration_done = False

    def compute_calibration(self):
        """Collect camera positions and compute transformation."""
        camera_points_list = []
        robot_points_list = []
        valid_tags = []
        
        # Collect camera points from TF transforms
        for tag_id, robot_point in self.robot_points.items():
            frame_name = f'tag36h11:{tag_id}'
            try:
                # Get transform from camera to tag
                transform = self.tf_buffer.lookup_transform(
                    'camera_color_optical_frame',
                    frame_name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                
                # Extract position
                pos = transform.transform.translation
                camera_point = np.array([pos.x, pos.y, pos.z])
                
                camera_points_list.append(camera_point)
                robot_points_list.append(robot_point)
                valid_tags.append(tag_id)
                
                self.get_logger().debug(f"Tag {tag_id} camera pos: {camera_point}")
                
            except Exception as e:
                self.get_logger().debug(f"Could not get transform for tag {tag_id}: {e}")
                pass
        
        # Need at least 3 points for calibration
        if len(camera_points_list) >= 18:
            camera_points = np.array(camera_points_list)
            robot_points = np.array(robot_points_list)
            
            self.get_logger().info(f"\nUsing {len(valid_tags)} tags for calibration: {valid_tags}")
            self.get_logger().info(f"Camera points:\n{camera_points}")
            self.get_logger().info(f"Robot points:\n{robot_points}")
            
            # Compute transformation using SVD
            T = self.compute_transform_svd(camera_points, robot_points)
            
            self.get_logger().info("\n" + "="*50)
            self.get_logger().info("CAMERA TO ROBOT TRANSFORMATION")
            self.get_logger().info("="*50)
            self.get_logger().info(f"\nTransformation Matrix:\n{T}")
            
            # Extract rotation and translation
            R = T[:3, :3]
            t = T[:3, 3]
            
            self.get_logger().info(f"\nRotation Matrix:\n{R}")
            self.get_logger().info(f"\nTranslation Vector:\n{t}")
            
            # Compute error
            error = self.compute_error(camera_points, robot_points, T)
            self.get_logger().info(f"\nMean reprojection error: {error:.6f} meters")
            
            if not self.calibration_done:
                self.calibration_done = True
                self.save_calibration(T)
                #############
                self.get_logger().info("Displaying calibration plot...")
                plot_3D_calibration(camera_points, robot_points, T)
                #############
        else:
            if not self.calibration_done:
                self.get_logger().warn(f"Waiting for tags... Got {len(camera_points_list)}/3 minimum")

    def compute_transform_svd(self, camera_points, robot_points):
        """
        Compute rigid transformation from camera frame to robot frame.
        Uses SVD-based method (Horn's algorithm).
        
        robot_point = R @ camera_point + t
        """
        # Compute centroids
        p_cam_mean = camera_points.mean(axis=0)
        p_rob_mean = robot_points.mean(axis=0)
        
        # Center the points
        P_cam = camera_points - p_cam_mean
        P_rob = robot_points - p_rob_mean
        
        # Compute covariance matrix
        H = P_cam.T @ P_rob
        
        # SVD
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure proper rotation (det = 1, not -1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Compute translation
        t = p_rob_mean - R @ p_cam_mean
        
        # Build 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        
        return T

    def compute_error(self, camera_points, robot_points, T):
        """Compute mean reprojection error."""
        # Transform all camera points
        camera_homo = np.hstack([camera_points, np.ones((camera_points.shape[0], 1))])
        predicted = (T @ camera_homo.T).T
        predicted = predicted[:, :3]
        
        # Compute error
        errors = np.linalg.norm(predicted - robot_points, axis=1)
        return errors.mean()

    def save_calibration(self, T):
        pkg_share = get_package_share_directory('vision')
        config_dir = os.path.join(pkg_share, 'config')
        os.makedirs(config_dir, exist_ok=True)

        filename = os.path.join(config_dir, 'camera_robot_calibration.npy')
        np.save(filename, T)

        self.get_logger().info(f"\nCalibration saved to {filename}")
        
        # Also save as readable format
        filename_txt = os.path.join(config_dir, 'camera_robot_calibration.txt')
        with open(filename_txt, 'w') as f:
            f.write("Camera to Robot Transformation Matrix\n")
            f.write("="*50 + "\n\n")
            f.write("4x4 Transformation Matrix:\n")
            f.write(str(T))
            f.write("\n\nRotation Matrix:\n")
            f.write(str(T[:3, :3]))
            f.write("\n\nTranslation Vector:\n")
            f.write(str(T[:3, 3]))
        self.get_logger().info(f"Calibration saved to {filename_txt}")

def plot_3D_calibration(camera_points, robot_points, T_cam_robot):
    """
    Visual verification for cal_cam_robot.py
    @camera_points: Nx3 numpy array of points in camera frame
    @robot_points: Nx3 numpy array of points in robot frame
    @T_cam_robot: 4x4 homogeneous transformation matrix
    """
    # 1. Transform camera points into robot frame
    # Convert Nx3 to Nx4 by adding a column of ones
    num_points = camera_points.shape[0]
    camera_homo = np.hstack([camera_points, np.ones((num_points, 1))])
    
    # Transform (T @ P.T).T gives us Nx4 back
    transformed_homo = (T_cam_robot @ camera_homo.T).T
    transformed_points = transformed_homo[:, :3]

    # 2. Set up the figure and 3D axis
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 3. Plot the three sets of points
    # Raw Camera Points (usually near origin)
    ax.scatter(camera_points[:, 0], camera_points[:, 1], camera_points[:, 2], 
               color='b', label="Camera Points (Raw)", alpha=0.5)
    
    # Ground Truth Robot Points
    ax.scatter(robot_points[:, 0], robot_points[:, 1], robot_points[:, 2], 
               color='r', s=100, label="Robot Points (Target)")

    # Transformed Camera Points (Should overlay the Red points)
    ax.scatter(transformed_points[:, 0], transformed_points[:, 1], transformed_points[:, 2], 
               color='g', marker='x', s=100, label="Transformed Camera -> Robot")

    # 4. Draw error lines between Robot and Transformed points
    for i in range(num_points):
        ax.plot([robot_points[i, 0], transformed_points[i, 0]],
                [robot_points[i, 1], transformed_points[i, 1]],
                [robot_points[i, 2], transformed_points[i, 2]], 
                color='black', linestyle='--', alpha=0.3)

    # Labeling
    ax.set_title("Calibration Verification (Meters)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()

    # Equalize axis scaling to prevent geometric warping
    max_range = np.array([robot_points[:,0].max()-robot_points[:,0].min(), 
                          robot_points[:,1].max()-robot_points[:,1].min(), 
                          robot_points[:,2].max()-robot_points[:,2].min()]).max() / 2.0
    mid_x = (robot_points[:,0].max()+robot_points[:,0].min()) * 0.5
    mid_y = (robot_points[:,1].max()+robot_points[:,1].min()) * 0.5
    mid_z = (robot_points[:,2].max()+robot_points[:,2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()

##########################


def main():
    rclpy.init()
    node = CalCamRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
