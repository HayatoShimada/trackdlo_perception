"""Tests for SegmentationNodeBase."""
import numpy as np
import pytest
import rclpy
from trackdlo_segmentation.base import SegmentationNodeBase


class DummySegNode(SegmentationNodeBase):
    """Concrete subclass for testing."""

    def __init__(self):
        super().__init__('dummy_seg')
        self.last_input = None

    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        self.last_input = cv_image
        mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        mask[cv_image[:, :, 0] > 128] = 255
        return mask


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_dummy_seg_instantiates():
    node = DummySegNode()
    assert node.get_name() == 'dummy_seg'
    node.destroy_node()


def test_segment_returns_correct_shape():
    node = DummySegNode()
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    mask = node.segment(img)
    assert mask.shape == (480, 640)
    assert mask.dtype == np.uint8
    node.destroy_node()


def test_segment_returns_binary_values():
    node = DummySegNode()
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[100:200, 100:200, 0] = 200  # Blue channel > 128
    mask = node.segment(img)
    assert set(np.unique(mask)).issubset({0, 255})
    assert mask[150, 150] == 255
    assert mask[0, 0] == 0
    node.destroy_node()


def test_has_publisher_and_subscriber():
    node = DummySegNode()
    pub_topics = [t[0] for t in node.get_publisher_names_and_types_by_node(
        'dummy_seg', '/')]
    sub_topics = [t[0] for t in node.get_subscriber_names_and_types_by_node(
        'dummy_seg', '/')]
    assert '/trackdlo/segmentation_mask' in pub_topics
    assert '/camera/color/image_raw' in sub_topics
    node.destroy_node()
