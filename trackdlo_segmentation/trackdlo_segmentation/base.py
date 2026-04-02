"""Base class for all segmentation nodes in trackdlo_perception.

All segmentation implementations (HSV, SAM2, YOLO, DeepLab, etc.)
inherit from this class and implement the segment() method.
Output is published to /trackdlo/segmentation_mask (mono8).
"""
from abc import abstractmethod

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class SegmentationNodeBase(Node):
    """Base ROS2 node for DLO segmentation.

    Subscribes to an RGB image topic, calls the abstract segment() method,
    and publishes the resulting binary mask.
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.bridge = CvBridge()

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        rgb_topic = self.get_parameter('rgb_topic').value

        self.mask_pub = self.create_publisher(
            Image, '/trackdlo/segmentation_mask', 10
        )
        self.image_sub = self.create_subscription(
            Image, rgb_topic, self._on_image, 10
        )

    def _on_image(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        mask = self.segment(cv_image)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

    @abstractmethod
    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        """Segment a BGR image and return a binary mask.

        Args:
            cv_image: Input BGR image, shape (H, W, 3), dtype uint8.

        Returns:
            Binary mask, shape (H, W), dtype uint8, values 0 or 255.
        """
        ...
