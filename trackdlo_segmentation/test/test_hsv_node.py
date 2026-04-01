"""Tests for HSV segmentation node."""
import numpy as np
import pytest
import rclpy
from trackdlo_segmentation.hsv_node import HsvSegmentationNode


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_hsv_node_instantiates():
    node = HsvSegmentationNode()
    assert node.get_name() == 'hsv_segmentation'
    node.destroy_node()


def test_segment_returns_binary_mask():
    node = HsvSegmentationNode()
    # Create a blue-ish image (H~100 in HSV, within default 85-135 range)
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:, :] = [180, 100, 50]  # BGR: blue-dominant
    mask = node.segment(img)
    assert mask.shape == (480, 640)
    assert mask.dtype == np.uint8
    assert set(np.unique(mask)).issubset({0, 255})
    node.destroy_node()


def test_segment_detects_blue_object():
    node = HsvSegmentationNode()
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    # Paint a blue rectangle (H~100 in HSV)
    img[100:200, 100:300] = [200, 80, 30]  # BGR: strong blue
    mask = node.segment(img)
    # The blue region should be detected
    assert mask[150, 200] == 255
    # Non-blue region should be zero
    assert mask[0, 0] == 0
    node.destroy_node()


def test_has_hsv_parameters():
    node = HsvSegmentationNode()
    lower = node.get_parameter('hsv_threshold_lower_limit').value
    upper = node.get_parameter('hsv_threshold_upper_limit').value
    assert lower == '85 50 20'
    assert upper == '135 255 255'
    node.destroy_node()
