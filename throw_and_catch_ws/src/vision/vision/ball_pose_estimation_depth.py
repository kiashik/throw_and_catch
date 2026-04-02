#!/usr/bin/env python3

"""
--- Purpose:
Estimate the 3D pose (position) of a detected tennis ball in the camera frame using depth
information from RealSense camera. This is simpler and faster than PnP, providing direct
3D coordinates by combining 2D pixel location with depth measurement.

--- How to use:
Run this node directly:
    ros2 run vision ball_pose_detection_depth

Or include it in a launch file. Requires:
- ball_detector node publishing ball centroids
- RealSense camera node publishing aligned depth images and camera info

--- Parameters / Configuration:
- Ball detection topic: 'vision/ball_detections' (vision_msgs/BoundingBox2D). must only publish a new msg when a ball is detected.
- Depth image topic: '/camera/camera/aligned_depth_to_color/image_raw' (sensor_msgs/Image)
- Camera info topic: '/camera/camera/aligned_depth_to_color/camera_info' (sensor_msgs/CameraInfo)
- Color image topic: '/camera/camera/color/image_raw' (sensor_msgs/Image) - for visualization
- Published topic: 'vision/ball_pose_cam' (geometry_msgs/PointStamped)
- Depth units: Typically millimeters (automatically converted to meters)


--- Improvement
04-01-2026: added a "visualize" parameter to enable/disable visualization. If 
visualization is disabled, the node will not subscribe to the color image topic 
or display the OpenCV window, which may improve speed.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge
import pyrealsense2 as rs       # used to get depth scale from RealSense camera, but if pyrealsense2 is not available, it will default to 0.001 m/unit which is typical for D400 series

BALL_RADIUS = 0.033  # meters. used to adjust depth value from ball surface to ball center

class BallPoseEstimationDepth(Node):
    """
    ROS2 node that estimates 3D position of a tennis ball using depth information.
    
    Subscribes to:
        - 'vision/ball_detections' (vision_msgs/BoundingBox2D): Ball detection in image pixels
        - '/camera/camera/aligned_depth_to_color/image_raw' (sensor_msgs/Image): Depth image
        - '/camera/camera/aligned_depth_to_color/camera_info' (sensor_msgs/CameraInfo): Camera intrinsics
        - '/camera/camera/color/image_raw' (sensor_msgs/Image): Color image for visualization
    
    Publishes to:
        - 'vision/ball_pose_cam' (geometry_msgs/PoseStamped): Ball position in camera frame
    
    Displays:
        - OpenCV window showing ball position visualization (press 'q' to quit)
    """

    def __init__(self):
        """
        Initialize the ball pose detection node using depth.
        
        Sets up:
        - Subscribers for ball detection, depth image, and camera info
        - Publisher for estimated ball position
        - Camera intrinsic parameters storage
        - OpenCV visualization
        """
        super().__init__('ball_pose_estimation_depth')

        self.declare_parameter('visualize', False)

        # Initialize storage for camera parameters
        self.fx = None  # Focal length x
        self.fy = None  # Focal length y
        self.cx = None  # Principal point x
        self.cy = None  # Principal point y
        self.camera_frame_id = None
        self.depth_scale = 0.001  # meters. Depth scale in meters per unit (default for D400: 1mm)
        
        # Initialize storage for ball detection and depth
        self.ball_detection = None
        self.depth_image = None
        self.color_image = None

        
        # Bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()

        # Subscribe to ball detection from ball_detector node
        self.detection_sub = self.create_subscription(
            BoundingBox2D,
            'vision/ball_detections',
            self.detection_callback,
            10
        )

        # Subscribe to aligned depth image
        self.aligned_depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.aligned_depth_callback,
            10
        )

        # Subscribe to camera info to get intrinsic parameters
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Subscribe to color image for visualization if enabled
        self.visualize = self.get_parameter('visualize').value
        if self.visualize:
            self.image_sub = self.create_subscription(
                Image,
                '/camera/camera/color/image_raw',
                self.image_callback,
                10
            )
        else:
            self.get_logger().info('Visualization disabled; color image subscription not created.')

        # Publish ball 3D position in camera frame
        self.pose_pub_ = self.create_publisher(
            PoseStamped,
            'vision/ball_pose_cam',
            10
        )

        self.get_logger().info("Ball pose detection (depth-based) node started")

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for camera info messages.
        
        --- Parameters/Input:
        msg : sensor_msgs.msg.CameraInfo
            Camera calibration and metadata.
        
        --- Process:
        Extracts and stores camera intrinsic parameters for depth-to-3D conversion.
        Also retrieves depth scale from the RealSense camera if available.
        
        --- Notes:
        - Only processes once; subsequent messages are ignored
        - Intrinsics needed: fx, fy (focal lengths) and cx, cy (principal point)
        - Depth scale is retrieved from device info or uses D400 default (0.001 m)
        """
        if self.fx is None:
            # Extract camera intrinsic parameters
            # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_frame_id = msg.header.frame_id
            
            self.get_logger().info("Camera intrinsics received")
            self.get_logger().info(f"fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
            self.get_logger().info(f"Frame ID: {self.camera_frame_id}")
            
            # DEBUG: Log all camera info fields to inspect depth scale
            self.get_logger().info(f"CameraInfo distortion array (d): {msg.d}")
            self.get_logger().info(f"CameraInfo distortion model: {msg.distortion_model}")
            
            # Try to retrieve depth scale from RealSense device info if available
            # Get depth scale from RealSense camera using pyrealsense2
            try:
                ctx = rs.context()
                devices = ctx.query_devices()
                if devices.size() > 0:
                    sensor = devices[0].first_depth_sensor()
                    self.depth_scale = sensor.get_depth_scale()
                    self.get_logger().info(f"Depth scale from RealSense device: {self.depth_scale:.6f} m/unit")
                else:
                    self.get_logger().warn(f"No RealSense device found, using default depth scale: {self.depth_scale:.6f} m/unit")
            except ImportError:
                self.get_logger().warn(f"pyrealsense2 not available, using default depth scale: {self.depth_scale:.6f} m/unit")
            except Exception as e:
                self.get_logger().warn(f"Could not get depth scale from device ({str(e)}), using default: {self.depth_scale:.6f} m/unit")
    
    
    def aligned_depth_callback(self, msg: Image):
        """
        Callback for aligned depth image messages.
        
        --- Parameters/Input:
        msg : sensor_msgs.msg.Image
            Depth image (16-bit, typically in millimeters).
        
        --- Process:
        Converts ROS Image to OpenCV format and stores for pose estimation.
        
        --- Notes:
        - Depth encoding is typically '16UC1' (16-bit unsigned, single channel)
        - Values are in millimeters and will be converted to meters
        """
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')   # allow depth info to passthrough to a opencv matrix
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {str(e)}")

    def detection_callback(self, msg: BoundingBox2D):
        """
        Callback for ball detection messages.
        
        --- Parameters/Input:
        msg : vision_msgs.msg.BoundingBox2D
            Ball detection containing centroid and dimensions.
        
        --- Process:
        1. Store ball detection
        2. If depth image and camera intrinsics available, compute 3D position
        3. Publish estimated position
        
        --- Notes:
        - This is the main processing callback that triggers pose estimation
        """
        self.ball_detection = msg
        
        if self.depth_image is not None and self.fx is not None:
            position = self.compute_3d_position()
            if position is not None:
                self.pose_pub_.publish(position)

    def image_callback(self, msg: Image):
        """
        Callback for color image messages (for visualization).
        
        --- Parameters/Input:
        msg : sensor_msgs.msg.Image
            Color camera image.
        
        --- Process:
        Stores latest image and displays visualization with ball position overlay.
        """
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Visualize if we have a ball detection
        if self.ball_detection is not None:
            self.visualize_pose()

    def compute_3d_position(self) -> PoseStamped | None:
        """
        Compute ball 3D position using depth information.
        
        --- Process:
        1. Get depth value at ball centroid pixel location
        2. Use pinhole camera model to convert (u, v, depth) to (x, y, z)
        3. Apply averaging over small region for noise reduction
        
        --- Returns:
        geometry_msgs.msg.PointStamped or None
            Ball position in camera frame, or None if depth invalid.
        
        --- Notes:
        - Pinhole model: x = (u - cx) * z / fx, y = (v - cy) * z / fy
        - Depth units typically in mm, converted to meters
        - Averages 3x3 region around centroid to reduce noise
        """
        if self.ball_detection is None or self.depth_image is None or self.fx is None:
            return None

        try:
            # Get pixel coordinates (ball centroid) (ensure within image bounds)
            u = int(self.ball_detection.center.position.x)
            v = int(self.ball_detection.center.position.y)
            
            height, width = self.depth_image.shape
            if u < 1 or u >= width - 1 or v < 1 or v >= height - 1:
                self.get_logger().warn("Ball centroid outside depth image bounds")
                return None

            # Get depth value with averaging over small region to reduce noise  (or reduce effects of outliers)
            # Using 3x3 region around centroid
            depth_region = self.depth_image[v-1:v+2, u-1:u+2].astype(np.float32)        #TODO instead of using 3 pixels, it might be best to use maybe 1/10th of ball pixel
            
            # Filter out zero/invalid depth values
            valid_depths = depth_region[depth_region > 0]
            
            if len(valid_depths) == 0:
                self.get_logger().warn("No valid depth at ball centroid")
                return None
            
            # Use median for robustness (TODO need to experiment with mean as well)
            depth_raw = np.median(valid_depths)
            depth_m = depth_raw * self.depth_scale  # Convert raw depth units to meters using depth scale
            depth_m = depth_m + BALL_RADIUS  # Adjust depth to ball center by adding radius (since depth is to surface)

            if depth_m <= 0 or depth_m > 6.0:  # Sanity check (ball should be within 6m). Our room is like 8m wide.
                self.get_logger().warn(f"Invalid depth value: {depth_m:.3f}m. Ball must be within 6m of camera.")
                return None

            # Ensure camera intrinsics are available
            if self.cx is None or self.cy is None or self.fx is None or self.fy is None:
                self.get_logger().warn("Camera intrinsics not fully initialized")
                return None

            # Convert pixel coordinates to 3D using pinhole camera model
            # x-axis: right, y-axis: down, z-axis: forward (optical frame convention)
            x = (u - self.cx) * depth_m / self.fx
            y = (v - self.cy) * depth_m / self.fy
            z = depth_m

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.camera_frame_id or 'camera_color_optical_frame'
            
            pose_msg.pose.position.x = float(x)
            pose_msg.pose.position.y = float(y)
            pose_msg.pose.position.z = float(z)

            self.get_logger().info(
                f"Ball position: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m)"
            )

            return pose_msg

        except Exception as e:
            self.get_logger().error(f"Error computing 3D position: {str(e)}")
            return None

    def visualize_pose(self):
        """
        Visualize ball detection and estimated position on the image.
        
        --- Process:
        1. Draw ball centroid on image
        2. Display 3D coordinates as text overlay
        3. Draw estimated ball radius circle
        4. Display in OpenCV window
        """
        if self.color_image is None:
            return

        vis_image = self.color_image.copy()

        # Draw ball detection
        if self.ball_detection is not None:
            center = (int(self.ball_detection.center.position.x), int(self.ball_detection.center.position.y))
            
            # Calculate pixel radius from bounding box
            if self.ball_detection.size_x > 0 and self.ball_detection.size_y > 0:
                pixel_radius = int((self.ball_detection.size_x + self.ball_detection.size_y) / 4.0)
            else:
                pixel_radius = 50  # Fallback
            
            # Draw center point                 (bgr)
            cv2.circle(vis_image, center, 5, (0, 0, 255), -1)
            
            # Draw actual detected boundary circle
            cv2.circle(vis_image, center, pixel_radius, (0, 0, 255), 2)
            
            # Display depth value if available
            if self.depth_image is not None and self.fx is not None:
                u, v = center
                height, width = self.depth_image.shape
                if 0 <= u < width and 0 <= v < height:
                    depth_raw = self.depth_image[v, u]
                    depth_m = depth_raw * self.depth_scale
                    
                    # Display depth and 3D coordinates
                    # text = f"Depth: {depth_m:.3f}m"
                    # cv2.putText(vis_image, text, (u + 10, v - 10),
                    #            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    if depth_m > 0 and self.cx is not None and self.cy is not None and self.fx is not None and self.fy is not None:
                        x = (u - self.cx) * depth_m / self.fx
                        y = (v - self.cy) * depth_m / self.fy
                        coord_text = f"({x:.3f}, {y:.3f}, {depth_m:.3f})"
                        cv2.putText(vis_image, coord_text, (u + 10, v + 20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Display
        cv2.imshow("Ball Pose Detection (Depth)", vis_image)
        
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


# -----------------------------
# Main Entry Point
# -----------------------------
def main():
    """
    Initialize and run the ball pose detection node (depth-based).
    
    --- Process:
    1. Initialize ROS2 Python client library
    2. Create BallPoseDetectionDepth node instance
    3. Spin node to process callbacks until interrupted
    4. Clean up on exit (Ctrl+C or 'q' keypress)
    
    --- Usage:
    Run directly:
        python3 ball_pose_detection_depth.py
    Or via ROS2:
        ros2 run vision ball_pose_detection_depth
    """
    rclpy.init()
    node = BallPoseEstimationDepth()
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
