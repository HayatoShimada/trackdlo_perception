#!/usr/bin/env python3
# Copyright 2026 Hayato Shimada
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""SAM2 segmentation node using Image Predictor with mask propagation."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from trackdlo_msgs.srv import SetPrompt
from sam2.sam2_image_predictor import SAM2ImagePredictor


class Sam2SegmentationNode(Node):
    """SAM2-based segmentation with click-to-prompt interface."""

    def __init__(self):
        super().__init__('sam2_segmentation')
        self.bridge = CvBridge()

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('sam2_model_id', 'facebook/sam2.1-hiera-small')

        rgb_topic = self.get_parameter('rgb_topic').value
        model_id = self.get_parameter('sam2_model_id').value

        # State
        self.state = 'LOADING_MODEL'
        self.latest_image = None
        self.prev_mask_logits = None
        self.prompt_point = None

        # Load SAM2
        self.get_logger().info(f'Loading SAM2 model: {model_id}')
        self.predictor = SAM2ImagePredictor.from_pretrained(model_id)
        self.get_logger().info('SAM2 model loaded')
        self.state = 'WAITING_PROMPT'

        # Publishers
        self.mask_pub = self.create_publisher(
            Image, '/trackdlo/segmentation_mask', 1)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, rgb_topic, self._on_image, 1)

        # Service
        self.prompt_srv = self.create_service(
            SetPrompt, '/trackdlo/sam2/set_prompt', self._on_set_prompt)

        self.get_logger().info(
            'SAM2 node ready. Click on composite_view to set prompt.')

    def _on_set_prompt(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = 'No image received yet'
            return response

        self.prompt_point = np.array([[request.x, request.y]], dtype=np.float32)
        self.prev_mask_logits = None

        # Run initial prediction with point prompt
        self.predictor.set_image(self.latest_image)
        masks, scores, logits = self.predictor.predict(
            point_coords=self.prompt_point,
            point_labels=np.array([1], dtype=np.int32),
            multimask_output=True,
        )

        # Select best mask
        best_idx = np.argmax(scores)
        self.prev_mask_logits = logits[best_idx:best_idx + 1]

        # Publish mask
        mask = (masks[best_idx] * 255).astype(np.uint8)
        self._publish_mask(mask)

        self.state = 'TRACKING'
        self.get_logger().info(
            f'Prompt set at ({request.x}, {request.y}), '
            f'score={scores[best_idx]:.3f}')
        response.success = True
        response.message = f'Tracking started (score={scores[best_idx]:.3f})'
        return response

    def _on_image(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

        if self.state != 'TRACKING' or self.prev_mask_logits is None:
            return

        # Propagate mask using previous logits
        self.predictor.set_image(self.latest_image)
        masks, scores, logits = self.predictor.predict(
            mask_input=self.prev_mask_logits,
            multimask_output=False,
        )

        self.prev_mask_logits = logits[0:1]
        mask = (masks[0] * 255).astype(np.uint8)
        self._publish_mask(mask)

    def _publish_mask(self, mask):
        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        self.mask_pub.publish(mask_msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Sam2SegmentationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
