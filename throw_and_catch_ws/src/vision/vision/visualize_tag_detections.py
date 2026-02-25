import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from apriltag_msgs.msg import AprilTagDetectionArray


class AprilTagDetectionViewer(Node):
    def __init__(self):
        super().__init__('visualize_tag_detections')

        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('image_topic', 'camera/camera/color/image_raw')
        self.declare_parameter('log_period_sec', 0.5)
        self.declare_parameter('min_decision_margin', 0.0)
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'AprilTag Detections')

        self._detections_topic = (
            self.get_parameter('detections_topic').get_parameter_value().string_value
        )
        self._image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._log_period_sec = (
            self.get_parameter('log_period_sec').get_parameter_value().double_value
        )
        self._min_decision_margin = (
            self.get_parameter('min_decision_margin').get_parameter_value().double_value
        )
        self._show_window = (
            self.get_parameter('show_window').get_parameter_value().bool_value
        )
        self._window_name = (
            self.get_parameter('window_name').get_parameter_value().string_value
        )
        self._last_log_time = self.get_clock().now()
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._last_detections = []

        self._detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            self._detections_topic,
            self._detections_callback,
            10,
        )
        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_callback,
            10,
        )

        self.get_logger().info(
            f'Listening for detections on: {self._detections_topic}'
        )
        self.get_logger().info(f'Listening for images on: {self._image_topic}')

    def _detections_callback(self, msg: AprilTagDetectionArray) -> None:
        now = self.get_clock().now()
        if self._log_period_sec > 0.0:
            elapsed = (now - self._last_log_time).nanoseconds * 1e-9
            if elapsed < self._log_period_sec:
                return
            self._last_log_time = now

        detections = [
            det
            for det in msg.detections
            if det.decision_margin >= self._min_decision_margin
        ]
        with self._lock:
            self._last_detections = detections

        if not detections:
            self.get_logger().info('No detections in message.')
            return

        lines = []
        for det in detections:
            center = det.centre
            lines.append(
                f"id={det.id} family='{det.family}' dm={det.decision_margin:.3f} "
                f"center=({center.x:.3f}, {center.y:.3f})"
            )

        self.get_logger().info('Detections:\n' + '\n'.join(lines))

    def _image_callback(self, msg: Image) -> None:
        if not self._show_window:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        with self._lock:
            detections = list(self._last_detections)

        for det in detections:
            corners = [
                (int(round(pt.x)), int(round(pt.y)))
                for pt in det.corners
            ]
            if len(corners) != 4:
                continue

            points = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(
                frame,
                [points],
                isClosed=True,
                color=(0, 255, 0),
                thickness=2,
            )

            center = (int(round(det.centre.x)), int(round(det.centre.y)))
            cv2.circle(frame, center, 4, (0, 0, 255), -1)

            # label = f"{det.family}:{det.id}"
            label = f"{det.id}"

            cv2.putText(
                frame,
                label,
                (center[0] + 6, center[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow(self._window_name, frame)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            self.get_logger().info("Quit requested ('q' pressed). Shutting down...")
            rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = AprilTagDetectionViewer()
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
