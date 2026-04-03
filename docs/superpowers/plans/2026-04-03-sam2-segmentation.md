# SAM2 セグメンテーション統合 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** SAM2 Image Predictor でDLOをセグメンテーションし、composite_viewのクリックでプロンプトを指定する。

**Architecture:** SAM2 Image Predictor をフレームごとに実行。初回はクリック座標をpoint promptとして使い、以降は前フレームのマスクを `mask_input` として渡してフレーム間一貫性を維持。`/trackdlo/segmentation_mask` に配信し、trackdlo_nodeの external モードで受信。

**Tech Stack:** SAM2 (facebook/sam2.1-hiera-small), PyTorch, ROS2 Humble, Python

**Spec:** `docs/superpowers/specs/2026-04-03-sam2-segmentation-design.md`

**Note:** SAM2 Video Predictor はビデオファイルパスを要求するため、リアルタイムストリームには SAM2 Image Predictor を使用。前フレームマスクの `mask_input` 渡しでフレーム間追跡を実現。

---

## File Structure

### 新規作成
- `trackdlo_msgs/srv/SetPrompt.srv` — クリック座標サービス
- `trackdlo_segmentation/trackdlo_segmentation/sam2_node.py` — SAM2ノード

### 変更
- `trackdlo_msgs/CMakeLists.txt` — SetPrompt.srv 追加
- `trackdlo_segmentation/setup.py` — エントリポイント追加
- `trackdlo_segmentation/package.xml` — trackdlo_msgs 依存追加
- `trackdlo_utils/trackdlo_utils/composite_view_node.py` — クリックUI追加
- `trackdlo_bringup/launch/trackdlo.launch.py` — segmentation_mode launch引数復活、SAM2条件起動
- `trackdlo_bringup/config/realsense_params.yaml` — SAM2パラメータ追加

---

### Task 1: SetPrompt サービス定義

**Files:**
- Create: `trackdlo_msgs/srv/SetPrompt.srv`
- Modify: `trackdlo_msgs/CMakeLists.txt`

- [ ] **Step 1: サービスファイル作成**

Create `trackdlo_msgs/srv/SetPrompt.srv`:

```
int32 x
int32 y
---
bool success
string message
```

- [ ] **Step 2: CMakeLists.txt にサービス追加**

`trackdlo_msgs/CMakeLists.txt` の `rosidl_generate_interfaces` にサービスを追加:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackingStatus.msg"
  "srv/SetPrompt.srv"
  DEPENDENCIES std_msgs
)
```

- [ ] **Step 3: ビルド確認**

Run: `cd ~/repos && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-select trackdlo_msgs`
Expected: ビルド成功

Run: `source install/setup.bash && ros2 interface show trackdlo_msgs/srv/SetPrompt`
Expected: サービス定義が表示

- [ ] **Step 4: コミット**

```bash
git add trackdlo_msgs/srv/SetPrompt.srv trackdlo_msgs/CMakeLists.txt
git commit -m "feat(msgs): add SetPrompt service for SAM2 click prompt"
```

---

### Task 2: SAM2 セグメンテーションノード

**Files:**
- Create: `trackdlo_segmentation/trackdlo_segmentation/sam2_node.py`
- Modify: `trackdlo_segmentation/setup.py`
- Modify: `trackdlo_segmentation/package.xml`

- [ ] **Step 1: package.xml に依存追加**

`trackdlo_segmentation/package.xml` に追加:

```xml
<exec_depend>trackdlo_msgs</exec_depend>
```

- [ ] **Step 2: setup.py にエントリポイント追加**

`trackdlo_segmentation/setup.py` の `console_scripts` に追加:

```python
'sam2_segmentation = trackdlo_segmentation.sam2_node:main',
```

- [ ] **Step 3: sam2_node.py 作成**

Create `trackdlo_segmentation/trackdlo_segmentation/sam2_node.py`:

```python
#!/usr/bin/env python3
# Copyright 2026 Hayato Shimada
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""SAM2 segmentation node using Image Predictor with mask propagation."""

import numpy as np
import cv2
import torch
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
        self.prev_mask_logits = None  # Reset mask propagation

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
```

- [ ] **Step 4: ビルド確認**

Run: `cd ~/repos && colcon build --packages-select trackdlo_msgs trackdlo_segmentation`
Expected: ビルド成功

- [ ] **Step 5: コミット**

```bash
git add trackdlo_segmentation/trackdlo_segmentation/sam2_node.py \
        trackdlo_segmentation/setup.py \
        trackdlo_segmentation/package.xml
git commit -m "feat(segmentation): add SAM2 Image Predictor segmentation node"
```

---

### Task 3: composite_view クリックUI

**Files:**
- Modify: `trackdlo_utils/trackdlo_utils/composite_view_node.py`

- [ ] **Step 1: import と SetPrompt クライアント追加**

`composite_view_node.py` に追加:

```python
from trackdlo_msgs.srv import SetPrompt
```

`__init__` に追加:

```python
self.prompt_cli = self.create_client(
    SetPrompt, '/trackdlo/sam2/set_prompt')
self.orig_size = None  # (h, w) of original camera image
self.panel_size = None  # (ph, pw) of display panel
cv2.setMouseCallback('TrackDLO Composite View', self._on_mouse)
```

- [ ] **Step 2: _on_mouse コールバック**

```python
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
```

- [ ] **Step 3: _timer_cb でサイズ追跡**

`_timer_cb` の先頭で `self.orig_size` と `self.panel_size` を更新するコードを追加。
`ref` から元画像サイズ取得後:

```python
h, w = ref.shape[:2]
ph, pw = h // 2, w // 2
self.orig_size = (h, w)
self.panel_size = (ph, pw)
```

- [ ] **Step 4: ビルド確認**

Run: `cd ~/repos && colcon build --packages-select trackdlo_utils`
Expected: ビルド成功

- [ ] **Step 5: コミット**

```bash
git add trackdlo_utils/trackdlo_utils/composite_view_node.py
git commit -m "feat(utils): add click-to-prompt for SAM2 on composite_view Camera panel"
```

---

### Task 4: launch + yaml 更新

**Files:**
- Modify: `trackdlo_bringup/launch/trackdlo.launch.py`
- Modify: `trackdlo_bringup/config/realsense_params.yaml`

- [ ] **Step 1: realsense_params.yaml に SAM2 パラメータ追加**

`realsense_params.yaml` の末尾に追加:

```yaml
sam2_segmentation:
  ros__parameters:
    rgb_topic: "/camera/color/image_raw"
    sam2_model_id: "facebook/sam2.1-hiera-small"
```

- [ ] **Step 2: launch ファイルに segmentation_mode 引数と SAM2 ノード追加**

`trackdlo.launch.py` を修正:

imports に追加:
```python
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
```

`generate_launch_description` に `segmentation_mode` 引数追加:
```python
DeclareLaunchArgument(
    'segmentation_mode', default_value='internal',
    description='Segmentation: internal (HSV), external, sam2',
    choices=['internal', 'external', 'sam2'],
),
```

`_launch_setup` で `segmentation_mode` を取得し、trackdlo_node のパラメータにオーバーライドを追加:

```python
segmentation_mode = LaunchConfiguration('segmentation_mode').perform(context)
# trackdlo_node: sam2 の場合は external モードにする
trackdlo_seg_mode = 'external' if segmentation_mode == 'sam2' else segmentation_mode
```

trackdlo ノードの parameters に追加:
```python
{'segmentation_mode': trackdlo_seg_mode},
```

SAM2 ノードを条件起動で追加:
```python
# --- SAM2 Segmentation (when segmentation_mode=sam2) ---
Node(
    package='trackdlo_segmentation',
    executable='sam2_segmentation',
    name='sam2_segmentation',
    output='screen',
    parameters=[params_file],
    condition=LaunchConfigurationEquals('segmentation_mode', 'sam2'),
),
```

- [ ] **Step 3: ビルド確認**

Run: `cd ~/repos && colcon build --packages-select trackdlo_bringup`
Expected: ビルド成功

- [ ] **Step 4: コミット**

```bash
git add trackdlo_bringup/launch/trackdlo.launch.py \
        trackdlo_bringup/config/realsense_params.yaml
git commit -m "feat(bringup): add segmentation_mode launch arg with SAM2 support"
```

---

### Task 5: 統合テスト

- [ ] **Step 1: フルビルド**

Run:
```bash
cd ~/repos && source /opt/ros/${ROS_DISTRO}/setup.bash && \
colcon build --packages-select trackdlo_msgs trackdlo_segmentation trackdlo_core \
  trackdlo_utils trackdlo_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Expected: 全パッケージビルド成功

- [ ] **Step 2: SAM2モードで起動**

Run:
```bash
source install/setup.bash && \
ros2 launch trackdlo_bringup trackdlo.launch.py segmentation_mode:=sam2
```
Expected:
- trackdlo_node が IDLE + external モードで起動
- sam2_segmentation が起動、モデルロード完了
- composite_view に Camera 画像が表示

- [ ] **Step 3: クリックでプロンプト送信**

composite_view の Camera パネル上でDLOをクリック。
Expected:
- SAM2 がマスク生成開始
- Segmentation Mask パネルにマスク表示

- [ ] **Step 4: 追跡開始**

param_tuner の Start ボタンまたは:
```bash
ros2 service call /trackdlo/start std_srvs/srv/Trigger
```
Expected:
- init_tracker が初期化
- trackdlo_node が TRACKING 状態に遷移
- TrackDLO Results にDLO追跡結果が表示

- [ ] **Step 5: コミット**

最終調整があればコミット:
```bash
git add -A && git commit -m "feat: SAM2 segmentation integration with click-to-prompt"
```
