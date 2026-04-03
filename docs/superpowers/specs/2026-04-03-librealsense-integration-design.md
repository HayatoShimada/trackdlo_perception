# librealsense2 直接統合 + 外部制御インターフェース設計

## 概要

trackdlo_nodeにlibrealsense2 SDKを直接統合し、ROS2トピック経由のカメラ入力を廃止する。
HSVセグメンテーションもC++に内蔵し、外部制御用サービスとステータス配信を追加する。

## 目的

- カメラフレーム取得からDLO追跡までのレイテンシを最小化
- 外部ノード/GUIからの初期化・停止・リセット制御を可能にする
- 追跡状態（セグメンテーション完了、DLO生成完了、エラー等）をリアルタイムで配信

## アーキテクチャ

### Before (現在)

```
[realsense2_camera_node] --ROS topic--> [hsv_segmentation (Py)] --ROS topic-->
[trackdlo_node (C++)] --ROS topic--> [composite_view (Py)]
                                       [init_tracker (Py)] --ROS topic-->
```

5プロセス、4回のROSトピック経由シリアライズ/デシリアライズ。

### After (新設計)

```
[trackdlo_node (C++)]
  |- librealsense2で直接RGB+Depth取得
  |- HSVセグメンテーション内蔵
  |- CPD-LLE追跡
  |- ROS2 Service: start / stop / reset
  |- ROS2 Topic: /trackdlo/status (状態配信)
  +- ROS2 Topic: results_pc, results_img, segmentation_mask等 (既存トピック維持)

[init_tracker (Py)]     <- そのまま残す（1回きり、scipy/skimage活用）
[composite_view (Py)]   <- そのまま残す（既存トピックを購読）
[param_tuner (Py)]      <- 拡張: Start/Stop/Resetボタン + ステータス表示
```

### 排除されるもの

- `realsense2_camera_node` (ROSラッパー) → librealsense2直接使用に置換
- `hsv_segmentation` (Python) → C++内蔵HSVに置換
- `realsense2_camera` パッケージへの実行時必須依存

## librealsense2 カメラ統合

trackdlo_node内でカメラフレームを直接取得する。

### フレーム取得

```cpp
rs2::pipeline pipe;
rs2::config cfg;
cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
// serial_noが指定されていれば特定カメラを使用
if (!serial_no.empty()) cfg.enable_device(serial_no);

rs2::align align(RS2_STREAM_COLOR);

auto frames = pipe.wait_for_frames();
auto aligned = align.process(frames);
auto color_frame = aligned.get_color_frame();
auto depth_frame = aligned.get_depth_frame();
// rs2::frame -> cv::Mat (メモリコピー1回)
```

### カメラ情報取得

CameraInfoトピック購読を廃止し、SDKから直接取得:

```cpp
auto profile = pipe.get_active_profile();
auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
auto intrinsics = stream.get_intrinsics();
// intrinsics.fx, fy, ppx, ppy -> proj_matrix_
```

### パラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `camera_width` | 424 | カメラ解像度 幅 |
| `camera_height` | 240 | カメラ解像度 高さ |
| `camera_fps` | 30 | フレームレート |
| `camera_serial_no` | "" | カメラシリアル番号（空なら自動検出） |

## HSVセグメンテーション統合

### 動作モード (`segmentation_mode` パラメータ)

| 値 | 動作 |
|---|---|
| `"internal"` (デフォルト) | 内蔵HSVで処理。フレーム取得→マスク生成→追跡が同一プロセス |
| `"external"` | `/trackdlo/segmentation_mask` トピックを購読。YOLO等の外部ノード用 |

### 内蔵HSV処理

```cpp
cv::Mat segment_hsv(const cv::Mat& bgr_image) {
  cv::Mat hsv, mask;
  cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, lower_bound_, upper_bound_, mask);
  return mask;
}
```

- HSV閾値は既存パラメータ `hsv_threshold_upper_limit` / `hsv_threshold_lower_limit` をそのまま使用
- 動的パラメータ変更に対応（param_tunerからリアルタイム調整可能）

## 外部制御サービス

### サービス

| サービス | 型 | 動作 |
|---|---|---|
| `/trackdlo/start` | `std_srvs/srv/Trigger` | カメラ起動→セグメンテーション→初期化→追跡開始 |
| `/trackdlo/stop` | `std_srvs/srv/Trigger` | 追跡停止、カメラ停止 |
| `/trackdlo/reset` | `std_srvs/srv/Trigger` | 追跡リセット→再初期化→追跡再開 |

レスポンスは `Trigger.Response` の `success` (bool) と `message` (string) で結果を返す。

### ステータストピック

`/trackdlo/status` をカスタムメッセージで配信（1Hz程度）:

```
# trackdlo_msgs/msg/TrackingStatus.msg
uint8 state
uint8 IDLE=0
uint8 INITIALIZING=1
uint8 TRACKING=2
uint8 ERROR=3

string message          # "DLO initialized (30 nodes)" / エラー内容
int32 tracked_nodes     # 現在の追跡ノード数
float64 fps             # 処理FPS
```

### 状態遷移

```
IDLE --start--> INITIALIZING --初期化成功--> TRACKING
                     |                         |
                     +-初期化失敗--> ERROR      +-reset--> INITIALIZING
                                     |         |
                                     +-start--> INITIALIZING
TRACKING --stop--> IDLE
    *    --reset--> INITIALIZING
```

- ノード起動時は `IDLE` で待機（`start` サービスを待つ）
- `init_tracker` (Python) の初期化ロジックはそのまま活用

## param_tuner GUI拡張

既存の param_tuner に制御ボタンとステータス表示を統合:

```
+--- CPD-LLE Control Panel -----------------+
|                                            |
|  [> Start]  [Stop]  [Reset]               |
|                                            |
|  State: * TRACKING                         |
|  Message: DLO initialized (30 nodes)       |
|  FPS: 28.5                                 |
|                                            |
|  --- Parameters -------------------------- |
|  beta     [====|============] 0.35         |
|  lambda   [====|============] 50000.0      |
|  ...                                       |
+--------------------------------------------+
```

- ボタン: OpenCVの `cv2.setMouseCallback` でクリック検出 → 対応するサービス呼び出し
- ステータス: `/trackdlo/status` 購読で常時更新
- 状態インジケータ: IDLE=灰、INITIALIZING=黄、TRACKING=緑、ERROR=赤

## 変更対象ファイル

### 変更

| ファイル | 変更内容 |
|---|---|
| `trackdlo_core/CMakeLists.txt` | `librealsense2` のfind_package追加 |
| `trackdlo_core/package.xml` | `librealsense2` 依存追加、`trackdlo_msgs` 依存追加 |
| `trackdlo_core/src/trackdlo_node.cpp` | librealsense2統合、HSV内蔵、サービス/ステータス追加 |
| `trackdlo_msgs/msg/TrackingStatus.msg` | 新規作成 |
| `trackdlo_msgs/CMakeLists.txt` | TrackingStatus.msg ビルド追加 |
| `trackdlo_msgs/package.xml` | メッセージ生成依存追加 |
| `trackdlo_bringup/launch/trackdlo.launch.py` | realsense2_cameraノード廃止、hsv_segmentation廃止 |
| `trackdlo_bringup/config/realsense_params.yaml` | カメラパラメータ追加 |
| `trackdlo_utils/trackdlo_utils/param_tuner_node.py` | Start/Stop/Resetボタン + ステータス表示 |

### 変更なし

- `trackdlo_core/src/trackdlo.cpp` — CPD-LLEアルゴリズム本体
- `trackdlo_core/src/pipeline_manager.cpp` — 処理パイプライン
- `trackdlo_core/trackdlo_core/initialize.py` — 初期化ロジック
- `trackdlo_utils/trackdlo_utils/composite_view_node.py` — 既存トピック維持で変更不要
- `trackdlo_segmentation/` — 外部マスクモード用に残す

### 新規作成

- `trackdlo_msgs/msg/TrackingStatus.msg`
