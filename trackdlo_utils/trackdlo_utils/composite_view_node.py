#!/usr/bin/env python3
# Copyright 2026 Hayato Shimada
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge

import cv2
import numpy as np
from trackdlo_msgs.srv import SetPrompt


class CompositeViewNode(Node):
    def __init__(self):
        super().__init__('composite_view')
        self.bridge = CvBridge()

        self.panels = {
            'Camera': None,
            'Segmentation Mask': None,
            'Segmentation Overlay': None,
            'TrackDLO Results': None,
        }

        self.create_subscription(
            Image, '/camera/color/image_raw', self._cb_camera, 1)
        self.create_subscription(
            Image, '/trackdlo/segmentation_mask_img', self._cb_mask, 1)
        self.create_subscription(
            Image, '/trackdlo/segmentation_overlay', self._cb_overlay, 1)
        self.create_subscription(
            Image, '/trackdlo/results_img', self._cb_results, 1)
        self.create_subscription(
            PointCloud2, '/trackdlo/results_pc', self._cb_pc, 1)

        camera_info_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            CameraInfo, '/camera/aligned_depth_to_color/camera_info', self._cb_camera_info, camera_info_qos)

        self.points_3d = []
        self.camera_info = None

        self.timer = self.create_timer(1.0 / 30.0, self._timer_cb)
        self.prompt_cli = self.create_client(
            SetPrompt, '/trackdlo/sam2/set_prompt')
        self.orig_size = None  # (h, w) of original camera image
        self.panel_size = None  # (ph, pw) of display panel
        cv2.setMouseCallback('TrackDLO Composite View', self._on_mouse)
        self.get_logger().info('Composite view node started')

    def _cb_camera(self, msg):
        self.panels['Camera'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _cb_mask(self, msg):
        mono = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        self.panels['Segmentation Mask'] = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    def _cb_overlay(self, msg):
        self.panels['Segmentation Overlay'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _cb_results(self, msg):
        self.panels['TrackDLO Results'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _cb_pc(self, msg):
        self.points_3d = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))

    def _cb_camera_info(self, msg):
        self.camera_info = msg

    def _timer_cb(self):
        # Determine panel size from first available image
        ref = next((img for img in self.panels.values() if img is not None), None)
        if ref is None:
            return

        h, w = ref.shape[:2]
        ph, pw = h, w
        self.orig_size = (h, w)
        self.panel_size = (ph, pw)

        grid = []
        for label, img in self.panels.items():
            if img is not None:
                display_img = img.copy()
                if label == 'TrackDLO Results' and self.points_3d and self.camera_info:
                    fx = self.camera_info.k[0]
                    fy = self.camera_info.k[4]
                    cx = self.camera_info.k[2]
                    cy = self.camera_info.k[5]
                    if len(self.points_3d) > 0:
                        # Display only one point (the first node) to avoid screen clutter
                        pt = self.points_3d[0]
                        x, y, z = pt[0], pt[1], pt[2]
                        if z > 0:
                            u = int(fx * x / z + cx)
                            v = int(fy * y / z + cy)
                            text = f"XYZ: {x:.2f}, {y:.2f}, {z:.2f}"
                            cv2.putText(display_img, text, (u, v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                panel = cv2.resize(display_img, (pw, ph))
            else:
                panel = np.zeros((ph, pw, 3), dtype=np.uint8)
            # cv2.putText(panel, label, (10, 25),
            #            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            grid.append(panel)

        top = np.hstack((grid[0], grid[1]))
        bottom = np.hstack((grid[2], grid[3]))
        composite = np.vstack((top, bottom))

        cv2.imshow('TrackDLO Composite View', composite)
        cv2.waitKey(1)

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.orig_size is None or self.panel_size is None:
            return

        ph, pw = self.panel_size
        # Camera panel is top-left (0,0) to (pw, ph)
        if x >= pw or y >= ph:
            return

        orig_h, orig_w = self.orig_size
        orig_x = int(x * orig_w / pw)
        orig_y = int(y * orig_h / ph)

        if self.prompt_cli.service_is_ready():
            req = SetPrompt.Request()
            req.x = orig_x
            req.y = orig_y
            self.prompt_cli.call_async(req)
            self.get_logger().info(
                f'SAM2 prompt sent: ({orig_x}, {orig_y})')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CompositeViewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
