import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class OpenCVROSNode(Node):
    def __init__(self):
        super().__init__('opencv_ros_node_rs')

        self.bridge = CvBridge()

        # Common RealSense color topic when launched with rs_launch.py:
        topic = '/camera/camera/color/image_raw'  # change if your topic differs
        self.sub = self.create_subscription(Image, topic, self.cb, 10)

        self.get_logger().info(f"Subscribing to: {topic}")
        self.get_logger().info("Press 'q' in the OpenCV window to quit.")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("realsense_color", frame)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            rclpy.shutdown()

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

