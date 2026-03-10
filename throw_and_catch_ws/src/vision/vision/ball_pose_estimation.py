#!/usr/bin/env python3
"""
--- Purpose:
Estimate the 3D pose (position and orientation) of a detected tennis ball in the camera frame
using PnP (Perspective-n-Point) algorithm. Subscribes to ball detections (bounding box) and 
camera intrinsics, then computes the 6DOF pose assuming the ball is a sphere of known radius.

--- How to use:
Run this node directly:
    ros2 run vision ball_pose_estimation

Or include it in a launch file. Requires:
- ball_detector node publishing ball centroids
- Camera node publishing CameraInfo (e.g., RealSense)

--- Parameters / Configuration:
- Ball radius: BALL_RADIUS constant (default: 0.0335 meters for tennis ball)
- Ball centroid topic: '/ball_detector/centroid' (geometry_msgs/Point)
- Camera info topic: '/camera/camera/color/camera_info' (sensor_msgs/CameraInfo)
- Image topic: '/camera/camera/color/image_raw' (sensor_msgs/Image) - for visualization
- Published topic: '/ball_pose_estimation/pose' (geometry_msgs/PoseStamped)

--- Improvements:
- TODO 1. Figure out what to do if PnP solver fails or a new detection didn't come through 
in a certain amount of time. Currently, latest detection persists, so it may be 
necessary to know if a detection was recent or not. if a ball suddenly goes out of frame, this will think that the ball is till at the 
last in frame position.

"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge

import tf_transformations       # TODO: (kayla) on lap computer: sudo apt install ros-jazzy-tf-transformations. see doc string comment below  

################################################################################
# maybe should SciPy (or transform3d) since tf_transformations is no longer maintained
"""
from scipy.spatial.transform import Rotation

# Replace this line:
quaternion = tf_transformations.quaternion_from_matrix(...)

# With:
rot = Rotation.from_matrix(rotation_matrix)
quaternion = rot.as_quat()  # Returns [x, y, z, w]
"""
################################################################################

# Constants
#TODO: measure our tennis ball
BALL_RADIUS = 0.07/2  # meters (standard tennis ball diameter ~6.7cm, radius ~3.35cm)

class BallPoseEstimation(Node):
    """
    ROS2 node that estimates 3D pose of a tennis ball using PnP algorithm.
    
    Subscribes to:
        - '/ball_detector/detection' (vision_msgs/BoundingBox2D): Ball detection in image pixels
        - '/camera/camera/color/camera_info' (sensor_msgs/CameraInfo): Camera intrinsics
        - '/camera/camera/color/image_raw' (sensor_msgs/Image): Camera image for visualization
    
    Publishes to:
        - 'ball_pose_estimation/pose' (geometry_msgs/PoseStamped): Ball pose in camera frame
    
    Displays:
        - OpenCV window showing ball pose visualization (press 'q' to quit)
    """

    def __init__(self):
        """
        Initialize the ball pose estimation node.
        
        Sets up:
        - Subscribers for ball centroid and camera info
        - Publisher for estimated ball pose
        - Camera intrinsic parameters storage
        - OpenCV visualization
        """
        super().__init__('ball_pose_estimation')

        self.bridge = CvBridge()        # Bridge for ROS-OpenCV conversion

        # Initialize storage for camera parameters
        self.camera_matrix = None       # 3x3 matrix
        self.dist_coeffs = None
        self.camera_frame_id = None
        
        # Initialize storage for ball detection
        self.ball_detection = None
        self.latest_image = None
    
        # Subscribe to ball detection from ball_detector node
        self.detection_sub = self.create_subscription(
            BoundingBox2D,
            'ball_detector/detection',
            self.detection_cb,
            10
        )

        # Subscribe to camera info to get intrinsic parameters
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_cb,
            10
        )
        
        # Subscribe to camera image for visualization
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_cb,
            10
        )

        # Publish ball pose
        self.pose_pub_ = self.create_publisher(
            PoseStamped,
            'ball_pose_estimation/pose',
            10
        )

        self.get_logger().info("Ball pose estimation node started")
        self.get_logger().info(f"Ball radius: {BALL_RADIUS} meters")

    def camera_info_cb(self, msg: CameraInfo):
        """
        Callback for camera info messages.
        TODO this gets called too often. camera intrinsic is not gonna change, so i only really need to run this once. 
        
        --- Parameters/Input:
        msg : sensor_msgs.msg.CameraInfo
            Camera calibration and metadata.
        
        --- Process:
        Extracts and stores camera intrinsic matrix and distortion coefficients.
        
        --- Notes:
        - Only processes once; subsequent messages are ignored
        - Camera matrix format: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        """
        if self.camera_matrix is None:
            # Extract camera intrinsic parameters
            self.camera_matrix = np.array([
                [msg.k[0], msg.k[1], msg.k[2]],  # [fx, 0, cx]
                [msg.k[3], msg.k[4], msg.k[5]],  # [0, fy, cy]
                [msg.k[6], msg.k[7], msg.k[8]]   # [0, 0, 1]
            ])
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame_id = msg.header.frame_id
            
            self.get_logger().info("Camera intrinsics received")
            self.get_logger().info(f"Camera matrix:\n{self.camera_matrix}")
            self.get_logger().info(f"Frame ID: {self.camera_frame_id}")

            # TODO test if unsubscribing is okay or not.
            # Unsubscribe after receiving first message (intrinsics won't change)
            # self.destroy_subscription(self.camera_info_sub)
            # self.get_logger().info("Unsubscribed from camera_info (intrinsics cached)")

    def detection_cb(self, msg: BoundingBox2D):
        """
        Callback for ball detection messages.
        
        --- Parameters/Input:
        msg : vision_msgs.msg.BoundingBox2D
            Ball detection containing centroid (center) and dimensions (size_x, size_y).
        
        --- Process:
        1. Store ball detection
        2. If camera intrinsics available, estimate ball pose using PnP
        3. Publish estimated pose
        """
        self.ball_detection = msg
        
        if self.camera_matrix is not None:
            pose = self.estimate_ball_pose_pnp()
            if pose is not None:
                self.pose_pub_.publish(pose)

    def image_cb(self, msg: Image):
        """
        Callback for camera image messages (for visualization).
        
        --- Parameters/Input:
        msg : sensor_msgs.msg.Image
            Camera image.
        
        --- Process:
        Stores latest image and displays visualization with ball pose overlay.
        """
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Visualize if we have a ball detection
        if self.ball_detection is not None and self.camera_matrix is not None:
            self.visualize_pose()

    def estimate_ball_pose_pnp(self) -> PoseStamped | None:
        """
        Called upon each new detections.
        Estimate the 3D pose of a detected ball using the Perspective-n-Point (PnP) algorithm.
        --- Process:
            1. Define 3D object points representing a sphere in its local coordinate frame
               (center point plus 4 cardinal points on the sphere surface at BALL_RADIUS distance)
            2. Map these to 2D image points based on the detected bounding box
               (centroid and 4 boundary points derived from pixel radius)
            3. Solve PnP using cv2.solvePnP with iterative method to estimate rotation and translation.
            4. Convert rotation vector to quaternion using Rodrigues + tf_transformations
            5. Package results into a ROS PoseStamped message
        --- Returns:
            PoseStamped | None: 
                - PoseStamped with ball position and orientation in camera frame if successful
                - None if no detection available, camera not calibrated, or PnP solving fails
        --- Notes:
            - Spheres are approximated using 5 coplanar points (center + 4 cardinal directions)
            - Orientation is less meaningful for spheres but position estimate is reliable
            - Pixel radius is computed as average of width/height divided by 4
            - Falls back to 50.0 pixel radius if bounding box dimensions are invalid (logs warning)
            - Assumes camera_matrix, dist_coeffs, and ball_detection are pre-populated
        Image Coordinate Convention:
            - Origin (0,0) is at top-left corner
            - X-axis increases rightward (positive = right)
            - Y-axis increases downward (positive = down)
            - Therefore: "centroid_y - pixel_radius" moves UP in the image (toward top)
        Raises:
            Logs error and returns None on exceptions during PnP solving or coordinate transforms
        """
        if self.ball_detection is None or self.camera_matrix is None:
            return None

        try:
            # Define 3D object points for a sphere (circular pattern on visible surface)
            # Using 5 points: center and 4 points on the circle at cardinal directions
            object_points = np.array([
                [0.0, 0.0, 0.0],                           # Center
                [BALL_RADIUS, 0.0, 0.0],                   # Right
                [0.0, BALL_RADIUS, 0.0],                   # Top
                [-BALL_RADIUS, 0.0, 0.0],                  # Left
                [0.0, -BALL_RADIUS, 0.0],                  # Bottom
            ], dtype=np.float64)

            # Image points: centroid and boundary points derived from bounding box
            if self.ball_detection.size_x > 0 and self.ball_detection.size_y > 0:
                # Use average of width/height as diameter, then divide by 2 for radius
                pixel_radius = (self.ball_detection.size_x + self.ball_detection.size_y) / 4.0
            else:
                # Fallback to estimated value if bbox not available
                pixel_radius = 50.0     # TODO is this reasonable? we should probably just raise an error and stop the program if this ever happends.
                self.get_logger().warn("Bounding box dimensions invalid, using estimated pixel radius")
            
            # Extract centroid from BoundingBox2D
            centroid_x = self.ball_detection.center.position.x
            centroid_y = self.ball_detection.center.position.y
            
            image_points = np.array([
                [centroid_x, centroid_y],                             # Center
                [centroid_x + pixel_radius, centroid_y],              # Right
                [centroid_x, centroid_y - pixel_radius],              # Top (y decreases upward)
                [centroid_x - pixel_radius, centroid_y],              # Left
                [centroid_x, centroid_y + pixel_radius],              # Bottom (y increases downward)
            ], dtype=np.float64)

            # Solve PnP to get rotation and translation vectors
            success, rvec, tvec = cv2.solvePnP(object_points, 
                                            image_points,
                                            self.camera_matrix,
                                            self.dist_coeffs,
                                            flags=cv2.SOLVEPNP_ITERATIVE
                                    )

            if not success:
                self.get_logger().warn("PnP estimation failed")
                return None

            # Convert rotation vector to quaternion
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # quaternion for only rotation matrix
            # rotation matrix's 4th row is [0, 0, 0, 1] and 4th col is [0, 0, 0, 1].T
            quaternion = tf_transformations.quaternion_from_matrix(
                np.vstack([
                    np.hstack([rotation_matrix, [[0], [0], [0]]]),     
                    [0, 0, 0, 1]
                ])
            )

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()         # this can be used to check if the pose is recent. see improvement 1
            pose_msg.header.frame_id = self.camera_frame_id or 'camera_color_optical_frame'
            
            pose_msg.pose.position.x = float(tvec[0][0])        # ROS2 msg needs float
            pose_msg.pose.position.y = float(tvec[1][0])
            pose_msg.pose.position.z = float(tvec[2][0])
            
            pose_msg.pose.orientation.x = quaternion[0]         # orientation does not matter for sphere, but it's here
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            self.get_logger().info(
                f"Ball pose: x={tvec[0][0]:.3f}, y={tvec[1][0]:.3f}, z={tvec[2][0]:.3f}"
            )

            return pose_msg

        except Exception as e:
            self.get_logger().error(f"Error in PnP estimation: {str(e)}")
            return None

    def visualize_pose(self):
        """
        Visualize ball detection and estimated pose on the image.
        Assumes that self.ball_detection is valid (i.e. not None)
        
        --- Process:
        1. Draw ball centroid on image
        2. Draw coordinate axes showing estimated pose
        3. Display in OpenCV window
        """
        if self.latest_image is None or self.ball_detection is None:
            return

        vis_image = self.latest_image.copy()

        # Calculate pixel radius from bounding box (same as in estimate_ball_pose_pnp)
        if self.ball_detection.size_x > 0 and self.ball_detection.size_y > 0:
            pixel_radius = int((self.ball_detection.size_x + self.ball_detection.size_y) / 4.0)
        else:
            pixel_radius = 50  # Fallback
        
        # Draw ball centroid and bounding circle
        center = (int(self.ball_detection.center.position.x), int(self.ball_detection.center.position.y))
        cv2.circle(vis_image, center, radius=5, color=(255, 0, 0), thickness=-1)
        cv2.circle(vis_image, center, radius=pixel_radius, color=(255, 0, 0), thickness=2)  # Actual detected boundary

        # Display
        cv2.imshow("Ball Pose Estimation", vis_image)
        
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        
        Ensures all OpenCV windows are closed before calling parent destructor.
        """
        try:
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main():
    """
    Initialize and run the ball pose estimation node.
    
    --- Process:
    1. Initialize ROS2 Python client library
    2. Create BallPoseEstimation node instance
    3. Spin node to process callbacks until interrupted
    4. Clean up on exit (Ctrl+C or 'q' keypress)
    
    --- Usage:
    Run directly:
        python3 ball_pose_estimation.py
    Or via ROS2:
        ros2 run vision ball_pose_estimation
    """
    rclpy.init()
    node = BallPoseEstimation()
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
