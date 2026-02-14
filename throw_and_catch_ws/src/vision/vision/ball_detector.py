#!/usr/bin/env python3
"""
Ball position in image frame is detected and published to a ROS topic
"""


import os
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge      # bridge between openCV and ros
import cv2
from pathlib import Path

from ultralytics import YOLO        # may need to install ultralytics on lab computer (see vision logbook)
import matplotlib.pyplot as plt
from pathlib import Path
from geometry_msgs.msg import Point
import os

class BallDettector(Node):
    def __init__(self):
        super().__init__('ball_detector')

        self.bridge = CvBridge()

        # SUBSCRIBE to the appropriate topic to get RGB images
        # RealSense color topic when launched with rs_launch.py:
        # topic = '/camera/camera/color/image_raw'      
        topic = 'webcam/image_raw'      # test dummy node for webcam images
        self.sub = self.create_subscription(Image, topic, self.cb, 10)

        self.get_logger().info(f"Subscribing to: {topic}")
        self.get_logger().info("Press 'q' in the OpenCV window to quit.")

        # Create PUBLISHER for ball centroids
        self.ball_centroid_publisher_ = self.create_publisher(Point, 'ball_detector/centroid', 10)
        self.ball_centroid_msg = Point()

        package_share_dir = get_package_share_directory('vision')
        yolo_model_path = os.path.join(package_share_dir, 'yolo_models', 'yolo11n_last_tennis_ball_eudyi_xwxjf.pt')
        self.ball_detector = YOLO(yolo_model_path)      # load a yolo model
        

    def cb(self, msg: Image):   # called upon each new image
        im = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # get ball centroid in image frame as a list of Results objects.
        # see https://docs.ultralytics.com/modes/predict/#working-with-results 
        # for .predict() arguments and Results object attributes.
        results_yolo = self.ball_detector.predict(im, conf=0.25, 
                                    imgsz=640, half=True, device='cuda:0', 
                                    max_det=1, visualize=False, show_boxes=True, 
                                    stream=False, show=False)   # in px
        # TODO: note what hardware lab computer has. if has good enough GPU, use device="cuda:0", else use 'cpu'.
        
        self.pub_ball_centroid_in_img(results_yolo)
        self.ball_centroid_publisher_.publish(self.ball_centroid_msg)

        # Plot detections on the image
        annotated_img = results_yolo[0].plot() if len(results_yolo) > 0 else im
        cv2.imshow("Ball Detector", annotated_img)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            rclpy.shutdown()

    def pub_ball_centroid_in_img(self, results_yolo):
        if results_yolo is not None and len(results_yolo) > 0:
            r = results_yolo[0]
            
            # Check if any boxes were detected
            if r.boxes is not None and len(r.boxes) > 0:
                if len(r.boxes) > 1:
                    self.get_logger().warn(f"Multiple balls detected ({len(r.boxes)}). Using the first one.")
                else:
                    self.get_logger().warn(f"One ball detected.")
                
                # Get bounding box in xywh format (x_center, y_center, width, height)
                box = r.boxes.xywh[0].cpu().numpy()
                
                # Set the pub msg to this new detected ball centroid
                self.ball_centroid_msg.x = float(box[0])  # x_center
                self.ball_centroid_msg.y = float(box[1])  # y_center
                self.ball_centroid_msg.z = 0.0  # 2D image plane, z is set to 0
            else:
                self.get_logger().info(f"No ball detected.")
        else:
            self.get_logger().info(f"Waiting for ball detection...")



    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = BallDettector()
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

