import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pathlib import Path


class OpenCVROSNode(Node):
    def __init__(self):
        super().__init__('opencv_ros_node_rs')

        self.bridge = CvBridge()

        self.images_dir = Path(__file__).resolve().parent / 'd455_images' /'fps60'
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.img_index = 0

        # RealSense color topic when launched with rs_launch.py:
        topic = '/camera/camera/color/image_raw'  
        self.sub = self.create_subscription(Image, topic, self.cb, 10)

        self.get_logger().info(f"Subscribing to: {topic}")
        self.get_logger().info("Press 'q' in the OpenCV window to quit.")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.img_index % 30 == 0:
            image_path = self.images_dir / f'img_60fps_{self.img_index}.jpg'
            self.get_logger().info(f"Saving image to: {image_path}")
            cv2.imwrite(str(image_path), frame)
        
        self.img_index += 1
        
        # cv2.imshow("realsense_color", frame)

        # if (cv2.waitKey(1) & 0xFF) == ord('q'):
        #     rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = OpenCVROSNode()
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

