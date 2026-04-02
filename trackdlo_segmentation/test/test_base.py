# Copyright 2026 Hayato Shimada
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Tests for SegmentationNodeBase."""
import numpy as np
import rclpy

from trackdlo_segmentation.base import SegmentationNodeBase


class DummySegNode(SegmentationNodeBase):
    """Concrete subclass for testing."""

    def __init__(self):
        super().__init__('dummy_seg')
        self.last_input = None

    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        """Segment by thresholding blue channel."""
        self.last_input = cv_image
        mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        mask[cv_image[:, :, 0] > 128] = 255
        return mask


def test_dummy_seg_instantiates():
    """Verify DummySegNode can be created and destroyed."""
    rclpy.init()
    try:
        node = DummySegNode()
        assert node.get_name() == 'dummy_seg'
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_segment_returns_correct_shape():
    """Verify segment returns correct shape and dtype."""
    rclpy.init()
    try:
        node = DummySegNode()
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        mask = node.segment(img)
        assert mask.shape == (480, 640)
        assert mask.dtype == np.uint8
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_segment_returns_binary_values():
    """Verify segment returns only 0 and 255."""
    rclpy.init()
    try:
        node = DummySegNode()
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[100:200, 100:200, 0] = 200
        mask = node.segment(img)
        assert set(np.unique(mask)).issubset({0, 255})
        assert mask[150, 150] == 255
        assert mask[0, 0] == 0
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_has_publisher_and_subscriber():
    """Verify node has expected pub/sub topics."""
    rclpy.init()
    try:
        node = DummySegNode()
        pub_topics = [
            t[0] for t in node.get_publisher_names_and_types_by_node(
                'dummy_seg', '/')]
        sub_topics = [
            t[0] for t in node.get_subscriber_names_and_types_by_node(
                'dummy_seg', '/')]
        assert '/trackdlo/segmentation_mask' in pub_topics
        assert '/camera/color/image_raw' in sub_topics
        node.destroy_node()
    finally:
        rclpy.shutdown()
