#!/usr/bin/env python3
# Copyright 2026 Hayato Shimada
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""CPD-LLE Parameter Tuner GUI Node.

Opens an OpenCV window with trackbars to dynamically adjust
TrackDLO tracking parameters at runtime via ROS2 parameter services.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger
from trackdlo_msgs.msg import TrackingStatus
import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont


# (name, type, slider_max, scale_divisor, offset, description)
# real_value = (slider_value + offset) / scale_divisor
PARAM_DEFS = [
    ('beta',       'double', 200,    100,   0,
     u'\u5f62\u72b6\u525b\u6027 (\u5c0f=\u67d4\u8edf, \u5927=\u76f4\u7dda\u7684)'),
    ('lambda',     'double', 100000, 1,     0,
     u'\u5927\u57df\u7684\u6ed1\u3089\u304b\u3055\u306e\u5f37\u5ea6'),
    ('alpha',      'double', 100,    10,    0,
     u'LLE\u6b63\u5247\u5316 (\u521d\u671f\u5f62\u72b6\u3078\u306e\u6574\u5408\u6027)'),
    ('mu',         'double', 50,     100,   1,
     u'\u30ce\u30a4\u30ba/\u5916\u308c\u5024\u6bd4\u7387 (0.01-0.5)'),
    ('max_iter',   'int',    100,    1,     1,
     u'EM\u6700\u5927\u53cd\u5fa9\u56de\u6570 (\u901f\u5ea6 vs \u7cbe\u5ea6)'),
    ('tol',        'double', 100,    10000, 1,
     u'\u53ce\u675f\u5224\u5b9a\u306e\u3057\u304d\u3044\u5024'),
    ('k_vis',      'double', 500,    1,     0,
     u'\u53ef\u8996\u6027\u9805\u306e\u91cd\u307f (\u96a0\u308c\u30ce\u30fc\u30c9\u306e\u4fe1\u983c\u5ea6)'),
    ('d_vis',      'double', 200,    1000,  1,
     u'\u30ae\u30e3\u30c3\u30d7\u88dc\u9593\u306e\u6700\u5927\u6e2c\u5730\u7dda\u8ddd\u96e2 [m]'),
    ('visibility_threshold', 'double', 50, 1000, 1,
     u'\u53ef\u8996\u5224\u5b9a\u306e\u8ddd\u96e2\u3057\u304d\u3044\u5024 [m]'),
    ('dlo_pixel_width',      'int',  100,   1,  5,
     u'DLO\u306e\u592a\u3055 (\u91cd\u306a\u308a\u5224\u5b9a\u7528) [px]'),
    ('downsample_leaf_size', 'double', 100, 1000, 1,
     u'\u30dc\u30af\u30bb\u30eb\u30b5\u30a4\u30ba [m] (\u5c0f=\u9ad8\u5bc6\u5ea6)'),
    ('lle_weight', 'double', 100,    1,     1,
     u'\u5c40\u6240\u7684\u306a\u5f62\u72b6\u4fdd\u6301\u306e\u5f37\u3055'),
]

WINDOW_NAME = 'CPD-LLE Param Tuner'
TARGET_NODE = '/trackdlo'


def real_to_slider(value, ptype, scale_divisor, offset):
    if ptype == 'int':
        return int(value)
    return int(round(value * scale_divisor)) - offset


def slider_to_real(slider_val, ptype, scale_divisor, offset):
    if ptype == 'int':
        return slider_val + offset
    return (slider_val + offset) / scale_divisor


class ParamTunerNode(Node):
    def __init__(self):
        super().__init__('param_tuner')

        self.set_cli = self.create_client(
            SetParameters, f'{TARGET_NODE}/set_parameters')
        self.get_cli = self.create_client(
            GetParameters, f'{TARGET_NODE}/get_parameters')

        self.prev_slider = {}

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

        # Create trackbars with default positions (will be updated once we
        # fetch current values from the target node)
        for name, ptype, slider_max, scale_div, offset, _desc in PARAM_DEFS:
            # start at minimum
            cv2.createTrackbar(name, WINDOW_NAME, 0, slider_max, lambda x: None)
            self.prev_slider[name] = 0

        # Service clients for control
        self.start_cli = self.create_client(Trigger, '/trackdlo/start')
        self.stop_cli = self.create_client(Trigger, '/trackdlo/stop')
        self.reset_cli = self.create_client(Trigger, '/trackdlo/reset')

        # Status subscription
        self.tracking_state = TrackingStatus.IDLE
        self.tracking_message = 'Idle'
        self.tracking_fps = 0.0
        self.tracking_nodes = 0
        self.create_subscription(
            TrackingStatus, '/trackdlo/status', self._on_status, 1)

        # Button regions (populated by _draw_info)
        self.buttons = {}
        cv2.setMouseCallback(WINDOW_NAME, self._on_mouse)

        # Try to fetch current parameter values from the target node
        self.initial_fetch_done = False
        self.fetch_timer = self.create_timer(1.0, self._try_fetch_initial)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info(
            f'Param Tuner started. Waiting for {TARGET_NODE} node...')

    def _try_fetch_initial(self):
        if self.initial_fetch_done:
            self.fetch_timer.cancel()
            return

        if not self.get_cli.service_is_ready():
            self.get_logger().info(
                f'Waiting for {TARGET_NODE}/get_parameters service...',
                throttle_duration_sec=5.0)
            return

        req = GetParameters.Request()
        req.names = [d[0] for d in PARAM_DEFS]
        future = self.get_cli.call_async(req)
        future.add_done_callback(self._on_initial_params)

    def _on_initial_params(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f'Failed to get parameters: {e}')
            return

        for i, (name, ptype, slider_max, scale_div, offset, _desc) in enumerate(PARAM_DEFS):
            pval = resp.values[i]
            if pval.type == ParameterType.PARAMETER_DOUBLE:
                real_val = pval.double_value
            elif pval.type == ParameterType.PARAMETER_INTEGER:
                real_val = pval.integer_value
            else:
                continue

            sv = real_to_slider(real_val, ptype, scale_div, offset)
            sv = max(0, min(sv, slider_max))
            cv2.setTrackbarPos(name, WINDOW_NAME, sv)
            self.prev_slider[name] = sv
            self.get_logger().info(f'  {name} = {real_val} (slider={sv})')

        self.initial_fetch_done = True
        self.fetch_timer.cancel()
        self.get_logger().info('Initial parameter values loaded.')

    def _on_status(self, msg):
        self.tracking_state = msg.state
        self.tracking_message = msg.message
        self.tracking_fps = msg.fps
        self.tracking_nodes = msg.tracked_nodes

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for name, (bx, by, bw, bh) in self.buttons.items():
            if bx <= x <= bx + bw and by <= y <= by + bh:
                if name == 'start':
                    self.start_cli.call_async(Trigger.Request())
                    self.get_logger().info('Start requested')
                elif name == 'stop':
                    self.stop_cli.call_async(Trigger.Request())
                    self.get_logger().info('Stop requested')
                elif name == 'reset':
                    self.reset_cli.call_async(Trigger.Request())
                    self.get_logger().info('Reset requested')

    def timer_callback(self):
        # Draw a blank info image
        info_img = self._draw_info()
        cv2.imshow(WINDOW_NAME, info_img)

        changed = []
        for name, ptype, slider_max, scale_div, offset, _desc in PARAM_DEFS:
            sv = cv2.getTrackbarPos(name, WINDOW_NAME)
            if sv != self.prev_slider.get(name):
                real_val = slider_to_real(sv, ptype, scale_div, offset)
                changed.append((name, ptype, real_val))
                self.prev_slider[name] = sv

        if changed and self.set_cli.service_is_ready():
            req = SetParameters.Request()
            for name, ptype, real_val in changed:
                p = Parameter()
                p.name = name
                pv = ParameterValue()
                if ptype == 'int':
                    pv.type = ParameterType.PARAMETER_INTEGER
                    pv.integer_value = int(real_val)
                else:
                    pv.type = ParameterType.PARAMETER_DOUBLE
                    pv.double_value = float(real_val)
                p.value = pv
                req.parameters.append(p)
                self.get_logger().info(f'Setting {name} = {real_val}')
            self.set_cli.call_async(req)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Param Tuner closing.')
            rclpy.shutdown()

    def _draw_info(self):
        """Draw info image with control buttons, status, and parameter values."""
        row_h = 32
        button_area_h = 80
        status_area_h = 80
        param_area_h = len(PARAM_DEFS) * row_h + 16
        w = 720
        h = button_area_h + status_area_h + param_area_h

        pil_img = PILImage.new('RGB', (w, h), (40, 40, 40))
        draw = ImageDraw.Draw(pil_img)

        try:
            font_btn = ImageFont.truetype(
                '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 18)
            font_val = ImageFont.truetype(
                '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 16)
            font_desc = ImageFont.truetype(
                '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 14)
        except (IOError, OSError):
            font_btn = ImageFont.load_default()
            font_val = font_btn
            font_desc = font_btn

        # --- Buttons ---
        btn_y = 15
        btn_h = 40
        btn_w = 100
        self.buttons = {}

        buttons_def = [
            ('start', 20, (0, 160, 0), '> Start'),
            ('stop', 140, (160, 0, 0), 'Stop'),
            ('reset', 260, (0, 80, 160), 'Reset'),
        ]
        for name, bx, color, label in buttons_def:
            draw.rounded_rectangle(
                [bx, btn_y, bx + btn_w, btn_y + btn_h],
                radius=5, fill=color)
            draw.text((bx + 15, btn_y + 8), label,
                      fill=(255, 255, 255), font=font_btn)
            self.buttons[name] = (bx, btn_y, btn_w, btn_h)

        # --- Status ---
        status_y = button_area_h
        state_names = {
            TrackingStatus.IDLE: ('IDLE', (128, 128, 128)),
            TrackingStatus.INITIALIZING: ('INITIALIZING', (220, 200, 0)),
            TrackingStatus.TRACKING: ('TRACKING', (0, 220, 0)),
            TrackingStatus.ERROR: ('ERROR', (220, 0, 0)),
        }
        state_name, state_color = state_names.get(
            self.tracking_state, ('UNKNOWN', (128, 128, 128)))

        draw.ellipse([20, status_y + 8, 36, status_y + 24], fill=state_color)
        draw.text((45, status_y + 5),
                  f'State: {state_name}', fill=(220, 220, 220), font=font_val)
        draw.text((20, status_y + 30),
                  f'{self.tracking_message}', fill=(180, 180, 180), font=font_desc)
        draw.text((20, status_y + 52),
                  f'FPS: {self.tracking_fps:.1f}  Nodes: {self.tracking_nodes}',
                  fill=(180, 180, 180), font=font_desc)

        # --- Parameters ---
        y = button_area_h + status_area_h
        draw.line([(10, y), (w - 10, y)], fill=(80, 80, 80), width=1)
        y += 8
        for name, ptype, slider_max, scale_div, offset, desc in PARAM_DEFS:
            sv = cv2.getTrackbarPos(name, WINDOW_NAME)
            real_val = slider_to_real(sv, ptype, scale_div, offset)
            if ptype == 'int':
                val_text = f'{name}: {int(real_val)}'
            else:
                val_text = f'{name}: {real_val:.4f}'
            draw.text((10, y), val_text, fill=(220, 220, 220), font=font_val)
            draw.text((320, y + 1), desc, fill=(140, 180, 140), font=font_desc)
            y += row_h

        return np.array(pil_img)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ParamTunerNode()
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
