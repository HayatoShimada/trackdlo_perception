"""HSV segmentation node with interactive GUI tuner.

Inherits from SegmentationNodeBase. Provides OpenCV trackbar GUI
for live HSV threshold adjustment.
"""
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from trackdlo_segmentation.base import SegmentationNodeBase


class HsvSegmentationNode(SegmentationNodeBase):
    """HSV color-based segmentation with optional GUI tuner."""

    def __init__(self):
        super().__init__('hsv_segmentation')

        self.declare_parameter('hsv_threshold_upper_limit', '135 255 255')
        self.declare_parameter('hsv_threshold_lower_limit', '85 50 20')
        self.declare_parameter('enable_gui', True)

        upper_str = self.get_parameter('hsv_threshold_upper_limit').value
        lower_str = self.get_parameter('hsv_threshold_lower_limit').value
        self.enable_gui = self.get_parameter('enable_gui').value

        upper_vals = [int(x) for x in upper_str.split()]
        lower_vals = [int(x) for x in lower_str.split()]
        self.h_min, self.s_min, self.v_min = lower_vals
        self.h_max, self.s_max, self.v_max = upper_vals

        self.latest_image = None

        if self.enable_gui:
            cv2.namedWindow('HSV Tuner', cv2.WINDOW_AUTOSIZE)
            cv2.createTrackbar('H Min', 'HSV Tuner', self.h_min, 179, lambda x: None)
            cv2.createTrackbar('S Min', 'HSV Tuner', self.s_min, 255, lambda x: None)
            cv2.createTrackbar('V Min', 'HSV Tuner', self.v_min, 255, lambda x: None)
            cv2.createTrackbar('H Max', 'HSV Tuner', self.h_max, 179, lambda x: None)
            cv2.createTrackbar('S Max', 'HSV Tuner', self.s_max, 255, lambda x: None)
            cv2.createTrackbar('V Max', 'HSV Tuner', self.v_max, 255, lambda x: None)
            self.gui_timer = self.create_timer(1.0 / 30.0, self._gui_callback)
            self.get_logger().info(
                'HSV Tuner GUI started. Adjust sliders, press Q to save & quit.')

    def _on_image(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        mask = self.segment(self.latest_image)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, self.s_max, self.v_max])
        return cv2.inRange(hsv, lower, upper)

    def _gui_callback(self):
        if self.latest_image is None:
            cv2.waitKey(1)
            return

        h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')

        if (h_min != self.h_min or s_min != self.s_min or v_min != self.v_min or
                h_max != self.h_max or s_max != self.s_max or v_max != self.v_max):
            self.get_logger().info(
                f'HSV: lower="{h_min} {s_min} {v_min}" upper="{h_max} {s_max} {v_max}"')
            self.h_min, self.s_min, self.v_min = h_min, s_min, v_min
            self.h_max, self.s_max, self.v_max = h_max, s_max, v_max

        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([h_min, s_min, v_min]),
                           np.array([h_max, s_max, v_max]))
        masked = cv2.bitwise_and(self.latest_image, self.latest_image, mask=mask)

        display = np.hstack([
            cv2.resize(self.latest_image, (640, 480)),
            cv2.resize(masked, (640, 480)),
        ])
        cv2.imshow('HSV Tuner', display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Final HSV values for realsense_params.yaml:')
            self.get_logger().info(
                f'  hsv_threshold_lower_limit: "{self.h_min} {self.s_min} {self.v_min}"')
            self.get_logger().info(
                f'  hsv_threshold_upper_limit: "{self.h_max} {self.s_max} {self.v_max}"')
            rclpy.shutdown()

    def destroy_node(self):
        if self.enable_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HsvSegmentationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
