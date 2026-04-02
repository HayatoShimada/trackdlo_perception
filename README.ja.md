# trackdlo_perception

[English version](README.md)

ROS2 (Humble/Jazzy) ベースの変形線状物体 (DLO: Deformable Linear Object) リアルタイム追跡システム。
Intel RealSense RGB-D カメラ (D415/D435/D455) で DLO を認識・追跡し、プレビューウィンドウで結果を確認できる。

[TrackDLO](https://github.com/RMDLO/trackdlo) (RMDLO) をベースに開発。

## 特徴

- **CPD-LLE アルゴリズム**によるリアルタイム DLO 追跡
- **プラグイン可能なセグメンテーション**: HSV / YOLO / DeepLab (拡張可能)
- **4パネルプレビューウィンドウ**: カメラ映像、マスク、オーバーレイ、追跡結果を同時表示
- **Docker ベース**: ワンコマンドで起動、GPU 自動検出
- **Humble / Jazzy 対応**: `ROS_DISTRO` ビルド引数で切替

## パッケージ構成

```
trackdlo_perception/
├── trackdlo_core/           CPD-LLE 追跡アルゴリズム (C++17 + Python)
├── trackdlo_segmentation/   セグメンテーション基底クラス + HSV 実装
├── trackdlo_utils/          Composite View, パラメータチューナー
├── trackdlo_bringup/        Launch ファイル・パラメータ・RViz 設定
├── trackdlo_msgs/           カスタムメッセージ (将来用)
└── docker/                  Docker Compose + GPU 設定
```

## クイックスタート

### ビルド (Docker)

```bash
cd docker/

# 全イメージをビルド
bash build.sh

# Core のみ
bash build.sh core

# Jazzy でビルド
ROS_DISTRO=jazzy bash build.sh
```

### 起動

```bash
xhost +local:docker
cd docker/

# HSV セグメンテーション (デフォルト)
./run.sh

# HSV チューナー GUI
./run.sh hsv_tuner

# バックグラウンド起動
./run.sh hsv -d
```

### ネイティブ起動

```bash
# ビルド
colcon build --packages-select trackdlo_msgs trackdlo_segmentation trackdlo_core trackdlo_utils trackdlo_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 起動
ros2 launch trackdlo_bringup trackdlo.launch.py
ros2 launch trackdlo_bringup trackdlo.launch.py segmentation:=hsv_tuner
ros2 launch trackdlo_bringup trackdlo.launch.py rviz:=false
```

## セグメンテーションモード

| モード | 説明 | ユースケース |
|--------|------|-------------|
| `hsv` (デフォルト) | HSV しきい値処理 | DLO の色が既知、パラメータ調整済み |
| `hsv_tuner` | スライダー GUI でリアルタイム調整 | 新しい DLO の HSV 値を探す |

### セグメンテーションの拡張

`SegmentationNodeBase` を継承するだけで新しいセグメンテーション手法を追加できる:

```python
from trackdlo_segmentation import SegmentationNodeBase

class MySegmentationNode(SegmentationNodeBase):
    def __init__(self):
        super().__init__('my_segmentation')

    def segment(self, cv_image):
        # cv_image: BGR (H, W, 3)
        # return: binary mask (H, W), values 0 or 255
        ...
```

全セグメンテーションノードは `/trackdlo/segmentation_mask` (mono8) にパブリッシュする。

## アーキテクチャ

### Docker 構成

```
trackdlo-core コンテナ
┌─────────────────────┐
│ RealSense driver    │
│ trackdlo_node (C++) │
│ init_tracker (Py)   │
│ HSV segmentation    │
│ composite_view      │
│ param_tuner         │
│ RViz2               │
└─────────────────────┘
    host network (CycloneDDS, ROS_DOMAIN_ID=42)
```

### 処理パイプライン

```
[RealSense D435/D415]
    |
    +-- /camera/color/image_raw
    +-- /camera/aligned_depth_to_color/image_raw
    |
    v
[セグメンテーション]  <- HSV / YOLO / DeepLab (差し替え可能)
    |
    v /trackdlo/segmentation_mask
    |
[trackdlo_node (CPD-LLE)]
    |
    +-- /trackdlo/results_pc      (追跡済み DLO ノード座標)
    +-- /trackdlo/results_img     (追跡結果の可視化)
    |
    v
[composite_view]  <- 4パネルプレビューウィンドウ
```

## 他の ROS2 プロジェクトとの連携

trackdlo_perception は標準 ROS2 メッセージ型のみを使用。同じ `ROS_DOMAIN_ID` を設定するだけで他のコンテナから購読可能:

```python
# 他のプロジェクトから追跡結果を購読する例
self.create_subscription(PointCloud2, '/trackdlo/results_pc', self.callback, 10)
```

## 主要パラメータ

`trackdlo_bringup/config/realsense_params.yaml` で設定:

| パラメータ | デフォルト値 | 説明 |
|---|---|---|
| `beta` | 0.35 | 形状剛性 (小さいほど柔軟) |
| `lambda` | 50000.0 | 大域的滑らかさの強度 |
| `alpha` | 3.0 | 初期形状への整合性 |
| `mu` | 0.1 | ノイズ比率 |
| `max_iter` | 20 | EM 最大反復数 |
| `k_vis` | 50.0 | 可視性項の重み |
| `d_vis` | 0.06 | ギャップ補間の最大測地線距離 (m) |
| `visibility_threshold` | 0.008 | 可視判定の距離しきい値 (m) |
| `downsample_leaf_size` | 0.02 | ボクセルサイズ (m) |
| `num_of_nodes` | 30 | 追跡ノード数 |

## 依存関係

- ROS2 Humble or Jazzy
- Intel RealSense SDK 2.0 (realsense2_camera)
- OpenCV, PCL, Eigen3
- scikit-image, scipy, Open3D

## ライセンス

BSD-3-Clause
