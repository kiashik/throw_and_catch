#!/usr/bin/env python3
"""
--- Purpose:
Detects tennis balls in real-time camera images using a YOLO object detection model.
The ball detections are published to a ROS2 topic.

--- How to use:
Run this node directly:
    ros2 run vision ball_detector

Or include it in a launch file. The node subscribes to a camera image topic and publishes
ball centroids. Ensure the camera node (e.g., RealSense or webcam) is running first.

--- Parameters / Configuration:
- Image topic: /camera/camera/color/image_raw for RealSense camera.
- YOLO model: Loaded from vision package share directory 
    ('yolo_models/yolo11n_last_tennis_ball_eudyi_xwxjf.pt'). Must be present.
- Device: Auto-detects CUDA GPU if available, otherwise uses CPU.
- YOLO parameters: conf=0.25 (confidence threshold), max_det=1 (detect at most 1 ball), etc.
- Published topic: 
    - 'vision/ball_detections' (vision_msgs/BoundingBox2D): centroid and bounding box

    
--- Improvements:
04-01-2026: Last detection no longer persists when no ball is detected. The message is 
only published when a ball is detected, so if no ball is detected.
Also created a parameter for visualization, so the user can choose to enable/disable 
the OpenCV window showing the detections. disabling visualization can speed up inference.
"""


import os
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D
import torch


class BallDetector(Node):
    """
    ROS2 node that detects tennis balls in camera images using YOLO and publishes 
    their centroids.
    
    Subscribes to:
        - Image topic (configurable, default: 'webcam/image_raw') or
          '/camera/camera/color/image_raw' for RealSense camera.
    Publishes to:
        - 'vision/ball_detections' (vision_msgs/BoundingBox2D): Ball centroid and width, height
    
    Displays:
        - OpenCV window showing annotated detections (press 'q' to quit)
    """

    def __init__(self):
        """
        Initialize the ball detector node.
        
        Sets up:
        - Image subscriber for camera feed
        - Publisher for ball centroid coordinates
        - YOLO model loading and device configuration
        - Optional OpenCV visualization timer
        """
        super().__init__('ball_detector')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('visualize', False)
        self.declare_parameter('visualization_rate', 30.0)
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('max_det', 1)
        self.declare_parameter('imgsz', [640, 640])

        self.bridge = CvBridge()

        topic = self.get_parameter('image_topic').value
        self.sub = self.create_subscription(Image, topic, self.cb, 10)

        self.get_logger().info(f"Subscribing to: {topic}")
        self.get_logger().info("Press 'q' in the OpenCV window to quit.")

        # Create PUBLISHER for ball detection (centroid + bounding box)
        self.ball_detection_publisher_ = self.create_publisher(BoundingBox2D, 'vision/ball_detections', 10)
        self.ball_detection_msg = BoundingBox2D()
        self.latest_annotated_img = None

        # Load YOLO model
        package_share_dir = get_package_share_directory('vision')
        # yolo_model_path = os.path.join(package_share_dir, 'yolo_models', 'yolo11n_last_tennis_ball_eudyi_xwxjf.pt')
        # yolo_model_path = os.path.join(package_share_dir, 'yolo_models', 'yolo11n_last_tennis_ball_eudyi_xwxjf.onnx')
        # yolo_model_path = os.path.join(package_share_dir, 'yolo_models', 'yolo11n_last_tennis_ball_eudyi_xwxjf.engine')
        # yolo_model_path = os.path.join(package_share_dir, 'yolo_models', 'yolo11n_last_tennis_ball_eudyi_xwxjf_openvino_model')
        yolo_model_path = os.path.join(package_share_dir, 'yolo_models', 'yolo26n_my_ds_v2_best.engine')
        
        self.ball_detector = YOLO(yolo_model_path, task="detect")      # load a yolo model
        
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        self.imgsz = tuple(self.get_parameter('imgsz').value)
        self.confidence = self.get_parameter('confidence').value
        self.max_det = self.get_parameter('max_det').value
        self.visualize = self.get_parameter('visualize').value

        self.get_logger().info(f"Using YOLO imgsz: {self.imgsz}")
        self.get_logger().info(f"Using YOLO confidence: {self.confidence}, max_det: {self.max_det}")

        if self.visualize:
            viz_rate = float(self.get_parameter('visualization_rate').value)
            self.create_timer(1.0 / viz_rate, self.visualization_timer_cb)

    def cb(self, msg: Image):
        """
        Callback function triggered on each new camera image.
        
        --- Parameters/Input:
        msg : sensor_msgs.msg.Image
            ROS2 Image message containing camera frame data.
        
        --- Process:
        1. Convert ROS Image to OpenCV format
        2. Run YOLO inference to detect tennis balls
        3. Extract and publish ball centroid coordinates
        4. Display annotated image in OpenCV window
        5. Check for 'q' keypress to quit
        
        --- Notes:
        - YOLO parameters: conf=confidence threshold, max_det=1 (single ball)
        - Centroid is published even if no ball detected (see get_ball_bbox)
        - half=True enables FP16 inference for faster processing on compatible GPUs
        """
        im = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


        # get ball centroid in image frame as a list of Results objects.
        # see https://docs.ultralytics.com/modes/predict/#working-with-results 
        # for .predict() arguments and Results object attributes.
        results_yolo = self.ball_detector.predict(
            im,
            conf=self.confidence,
            imgsz=self.imgsz,
            half=True,
            device=self.device,
            max_det=self.max_det,
            visualize=False,
            show_boxes=True,
            stream=False,
            show=False,
        )

        self.latest_annotated_img = results_yolo[0].plot() if len(results_yolo) > 0 else im

        if self.get_ball_bbox(results_yolo):
            self.ball_detection_publisher_.publish(self.ball_detection_msg)

    def get_ball_bbox(self, results_yolo):
        """
        Extract ball detection from YOLO results and update the publisher message.
        
        --- Parameters/Input:
        results_yolo : list[ultralytics.engine.results.Results]
            YOLO prediction results containing detected bounding boxes.
        
        --- Process:
        - If ball(s) detected: Extract centroid and dimensions from first bounding 
            box, and return True.
        - If multiple balls: Use first detection and log warning
        - If no balls: Log info message (detection not updated), return False.
        
        --- Side Effects:
        Updates self.ball_detection_msg (BoundingBox2D) with:
            - center.x: horizontal pixel coordinate of centroid (float)
            - center.y: vertical pixel coordinate of centroid (float)
            - center.theta: 0.0 (orientation not applicable for sphere)
            - size_x: bounding box width in pixels (float)
            - size_y: bounding box height in pixels (float)
        
        --- Notes:
        - Data extracted from YOLO's xywh format (center_x, center_y, width, height)
        """
        if results_yolo is not None and len(results_yolo) > 0:
            r = results_yolo[0]

            
            # Check if any boxes were detected
            if r.boxes is not None and len(r.boxes) > 0:
                if len(r.boxes) > 1:
                    self.get_logger().warning(f"Multiple({len(r.boxes)}) balls detected. Using the first one.")
                else:
                    self.get_logger().info("One ball detected.")

                box = r.boxes.xywh[0].cpu().numpy()
                
                # Populate BoundingBox2D message
                self.ball_detection_msg.center.position.x = float(box[0])  # x_center
                self.ball_detection_msg.center.position.y = float(box[1])  # y_center
                self.ball_detection_msg.center.theta = 0.0  # orientation (not meaningful for sphere)
                self.ball_detection_msg.size_x = float(box[2])  # width in pixels
                self.ball_detection_msg.size_y = float(box[3])  # height in pixels
                return True
            else:
                self.get_logger().info("No ball detected.")
        else:
            self.get_logger().info("Waiting for ball detection...")

        return False

    def visualization_timer_cb(self):
        if self.latest_annotated_img is None:
            return

        cv2.imshow("Ball Detector", self.latest_annotated_img)
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
    rclpy.init()
    node = BallDetector()
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
