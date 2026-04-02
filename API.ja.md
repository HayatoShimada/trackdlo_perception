# trackdlo_perception APIリファレンス

[English version](API.md)

## ROS2 ノード

### trackdlo (C++ 追跡ノード)

CPD-LLEアルゴリズムによるDLOリアルタイム追跡ノード。

**実行ファイル:** `trackdlo` (パッケージ: `trackdlo_core`)

#### 購読トピック

| トピック | 型 | 説明 |
|---|---|---|
| `/camera/color/image_raw` | sensor_msgs/Image | RGB画像 (`rgb_topic`で変更可能) |
| `/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | 深度画像 (`depth_topic`で変更可能) |
| `/camera/aligned_depth_to_color/camera_info` | sensor_msgs/CameraInfo | カメラ内部パラメータ |
| `/trackdlo/init_nodes` | sensor_msgs/PointCloud2 | 初期DLOノード位置 (init_trackerから) |
| `/trackdlo/segmentation_mask` | sensor_msgs/Image | 外部セグメンテーションマスク (mono8, `use_external_mask=true`時のみ) |
| `/mask_with_occlusion` | sensor_msgs/Image | オクルージョンマスク |

#### 公開トピック

| トピック | 型 | 説明 |
|---|---|---|
| `/trackdlo/results_pc` | sensor_msgs/PointCloud2 | **メイン出力:** 追跡済みDLOノード座標 |
| `/trackdlo/results_img` | sensor_msgs/Image | 追跡結果の可視化画像 |
| `/trackdlo/results_marker` | visualization_msgs/MarkerArray | 追跡結果3Dマーカー (RViz用) |
| `/trackdlo/guide_nodes` | visualization_msgs/MarkerArray | ガイドノードマーカー |
| `/trackdlo/filtered_pointcloud` | sensor_msgs/PointCloud2 | ダウンサンプリング済み入力点群 |
| `/trackdlo/segmentation_mask_img` | sensor_msgs/Image | セグメンテーションマスク |
| `/trackdlo/segmentation_overlay` | sensor_msgs/Image | マスクオーバーレイ画像 |
| `/trackdlo/self_occluded_pc` | sensor_msgs/PointCloud2 | 自己遮蔽ノード |
| `/trackdlo/corr_priors` | visualization_msgs/MarkerArray | 対応事前情報 |

#### パラメータ

**アルゴリズム (CPD-LLE):**

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `beta` | double | 0.35 | 形状剛性。小さいほど柔軟 |
| `lambda` | double | 50000.0 | 大域的滑らかさの強度 |
| `alpha` | double | 3.0 | 初期形状への整合性 |
| `mu` | double | 0.1 | ノイズ比率 (0-1) |
| `max_iter` | int | 50 | EM最大反復数 |
| `tol` | double | 0.0002 | EM収束判定しきい値 |
| `lle_weight` | double | 10.0 | LLE正則化重み |

**可視性/オクルージョン:**

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `k_vis` | double | 50.0 | 可視性項の重み |
| `d_vis` | double | 0.06 | ギャップ補間の最大測地線距離 (m) |
| `visibility_threshold` | double | 0.008 | 可視判定の距離しきい値 (m) |
| `dlo_pixel_width` | int | 40 | DLOの画素幅 |

**前処理:**

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `beta_pre_proc` | double | 3.0 | 前処理用レジストレーションbeta |
| `lambda_pre_proc` | double | 1.0 | 前処理用レジストレーションlambda |
| `downsample_leaf_size` | double | 0.008 | ボクセルグリッドサイズ (m) |
| `multi_color_dlo` | bool | false | 多色DLO検出 |

**セグメンテーション:**

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `use_external_mask` | bool | false | 外部マスクを使用 (HSV内蔵の代わり) |
| `hsv_threshold_lower_limit` | string | "90 90 30" | HSV下限 "H S V" |
| `hsv_threshold_upper_limit` | string | "130 255 255" | HSV上限 "H S V" |

**トピック設定:**

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `camera_info_topic` | string | "/camera/aligned_depth_to_color/camera_info" | カメラ情報トピック |
| `rgb_topic` | string | "/camera/color/image_raw" | RGB画像トピック |
| `depth_topic` | string | "/camera/aligned_depth_to_color/image_raw" | 深度画像トピック |
| `result_frame_id` | string | "camera_color_optical_frame" | 結果のTFフレーム |

---

### init_tracker (Python 初期化ノード)

最初のフレームからDLOスケルトンを抽出し、初期ノード位置を公開。

**実行ファイル:** `init_tracker` (パッケージ: `trackdlo_core`)

#### 公開トピック

| トピック | 型 | 説明 |
|---|---|---|
| `/trackdlo/init_nodes` | sensor_msgs/PointCloud2 | 初期ノード位置 (1回のみ公開) |
| `/trackdlo/init_nodes_markers` | visualization_msgs/MarkerArray | 初期化マーカー (RViz用) |

#### パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `num_of_nodes` | int | 30 | 追跡ノード数 |
| `multi_color_dlo` | bool | false | 多色DLO検出 |
| `visualize_initialization_process` | bool | false | デバッグ可視化 |
| `use_external_mask` | bool | false | 外部マスク使用 |

---

### hsv_segmentation (Python HSVセグメンテーションノード)

HSV色空間によるセグメンテーション。オプションでインタラクティブGUIチューナー付き。

**実行ファイル:** `hsv_segmentation` (パッケージ: `trackdlo_segmentation`)

#### パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `rgb_topic` | string | "/camera/color/image_raw" | RGB画像トピック |
| `hsv_threshold_lower_limit` | string | "85 50 20" | HSV下限 "H S V" |
| `hsv_threshold_upper_limit` | string | "135 255 255" | HSV上限 "H S V" |
| `enable_gui` | bool | false | トラックバーGUIを有効化 |

---

## Launch 引数

### trackdlo.launch.py

| 引数 | デフォルト | 選択肢 | 説明 |
|---|---|---|---|
| `segmentation` | `hsv` | `hsv`, `hsv_tuner` | セグメンテーション方式。`hsv_tuner`でGUIノード起動 |
| `rviz` | `true` | — | RViz2を起動 |

```bash
ros2 launch trackdlo_bringup trackdlo.launch.py segmentation:=hsv_tuner rviz:=false
```

---

## C++ ライブラリ API

### trackdlo (CPD-LLEアルゴリズム)

```cpp
#include "trackdlo_core/trackdlo.hpp"

// パラメータ付きで構築
trackdlo tracker(30, 0.008, 0.35, 50000.0, 3.0, 50.0, 0.1, 50, 0.0002, 3.0, 1.0, 10.0);

// 初期化
tracker.initialize_nodes(Y_init);  // Eigen::MatrixXd (N x 3)
tracker.initialize_geodesic_coord(geodesic_coord);

// フレームごとの追跡
tracker.tracking_step(X, visible_nodes, visible_nodes_extended, proj_matrix, rows, cols);

// 結果取得
Eigen::MatrixXd result = tracker.get_tracking_result();  // (N x 3)
```

### ImagePreprocessor

```cpp
#include "trackdlo_core/image_preprocessor.hpp"

trackdlo_core::ImagePreprocessor preprocessor(false, false, {85, 50, 20}, {135, 255, 255});
cv::Mat mask, cur_image;
bool ok = preprocessor.process(rgb_image, mask, cur_image);
```

### VisibilityChecker

```cpp
#include "trackdlo_core/visibility_checker.hpp"

trackdlo_core::VisibilityChecker checker;
trackdlo_core::VisibilityResult result = checker.check_visibility(
  Y, X, proj_matrix, mask, 0.008, 20);
// result.visible_nodes — 可視ノードのインデックス
// result.not_self_occluded_nodes — 自己遮蔽されていないノード
```

### PipelineManager

```cpp
#include "trackdlo_core/pipeline_manager.hpp"

trackdlo_core::PipelineManager pipeline(false, false, lower, upper);
pipeline.initialize_tracker(init_nodes, geodesic_coord);

trackdlo_core::PipelineResult result = pipeline.process(rgb, depth, proj_matrix);
if (result.success) {
  // result.Y — 追跡ノード (Eigen::MatrixXd, N x 3)
}
```

---

## Python セグメンテーション API

### カスタムセグメンテーションの作成

```python
from trackdlo_segmentation import SegmentationNodeBase
import numpy as np

class MySegmentationNode(SegmentationNodeBase):
    def __init__(self):
        super().__init__('my_segmentation')

    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        """BGR画像をセグメンテーションする。

        Args:
            cv_image: 入力BGR画像, shape (H, W, 3), dtype uint8

        Returns:
            バイナリマスク, shape (H, W), dtype uint8, 値は0または255
        """
        mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        return mask
```

全セグメンテーションノードは自動的に:
- RGBトピックを購読（`rgb_topic`パラメータで設定可能）
- `/trackdlo/segmentation_mask` (mono8) にマスクを公開

### SegmentationNodeBase リファレンス

| メソッド | 説明 |
|---|---|
| `__init__(node_name: str)` | ROS2ノード名で初期化 |
| `segment(cv_image: np.ndarray) -> np.ndarray` | **抽象メソッド。** セグメンテーションロジックを実装 |
| `_on_image(msg: Image)` | 内部コールバック。`segment()`を呼びマスクを公開 |

| 属性 | 型 | 説明 |
|---|---|---|
| `bridge` | CvBridge | ROS ↔ OpenCV 変換器 |
| `mask_pub` | Publisher | マスク公開用パブリッシャー |
| `image_sub` | Subscription | RGB画像サブスクリプション |
