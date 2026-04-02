# Copyright 2026 Hayato Shimada
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Tests for HSV segmentation node."""
import numpy as np
import rclpy

from trackdlo_segmentation.hsv_node import HsvSegmentationNode


def test_hsv_node_instantiates():
    """Verify HsvSegmentationNode can be created."""
    rclpy.init()
    try:
        node = HsvSegmentationNode()
        assert node.get_name() == 'hsv_segmentation'
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_segment_returns_binary_mask():
    """Verify segment returns binary mask for blue image."""
    rclpy.init()
    try:
        node = HsvSegmentationNode()
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:, :] = [180, 100, 50]
        mask = node.segment(img)
        assert mask.shape == (480, 640)
        assert mask.dtype == np.uint8
        assert set(np.unique(mask)).issubset({0, 255})
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_segment_detects_blue_object():
    """Verify blue objects are detected by HSV thresholding."""
    rclpy.init()
    try:
        node = HsvSegmentationNode()
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[100:200, 100:300] = [200, 80, 30]
        mask = node.segment(img)
        assert mask[150, 200] == 255
        assert mask[0, 0] == 0
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_has_hsv_parameters():
    """Verify HSV threshold parameters are declared."""
    rclpy.init()
    try:
        node = HsvSegmentationNode()
        lower = node.get_parameter('hsv_threshold_lower_limit').value
        upper = node.get_parameter('hsv_threshold_upper_limit').value
        assert lower == '85 50 20'
        assert upper == '135 255 255'
        node.destroy_node()
    finally:
        rclpy.shutdown()
