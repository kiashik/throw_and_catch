import cv2
import rclpy
from rclpy.node import Node


class OpenCVNode(Node):
    def __init__(self):
        super().__init__('opencv_ros_node')
        self.get_logger().info('OpenCV ROS 2 Node Initialized')

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera (index 0).')
            raise RuntimeError('Camera open failed')

        # Timer drives the loop at ~30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.loop)

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera.')
            return

        cv2.imshow('frame', frame)

        # Press 'q' in the OpenCV window to quit
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            self.get_logger().info("Quit requested ('q' pressed). Shutting down...")
            rclpy.shutdown()

    def destroy_node(self):
        # Cleanup resources when node is destroyed
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = OpenCVNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()



