# SAM2 セグメンテーション統合設計

## 概要

SAM2 (Segment Anything Model 2) の Video Predictor を使い、ユーザーがcomposite_view上でDLOをクリックして指定した後、フレーム間自動追跡でセグメンテーションマスクを生成する。既存の `segmentation_mode: "external"` を活用し、内蔵HSVとの切り替えはパラメータのみで行う。

## 目的

- 黒いケーブルなどHSVでは困難なDLOを高精度にセグメンテーション
- 初回クリックで指定、以降は自動追跡
- 既存のtrackdlo_nodeとの統合は`/trackdlo/segmentation_mask`トピック経由で変更不要

## アーキテクチャ

```
[trackdlo_node (C++)]
  |- segmentation_mode: "external"
  |- /trackdlo/segmentation_mask を購読（既存機能）
  +- カメラ取得 + RGB/Depth 配信（既存）

[sam2_segmentation (Python, 新規)]
  |- /camera/color/image_raw を購読
  |- SAM2 Video Predictor でフレームごとにマスク生成
  |- /trackdlo/segmentation_mask に配信
  +- /trackdlo/sam2/set_prompt サービスでクリック座標受付

[composite_view (Python, 拡張)]
  +- Cameraパネルクリック → /trackdlo/sam2/set_prompt 呼び出し
```

## SAM2ノード

### 状態遷移

```
LOADING_MODEL → WAITING_PROMPT → TRACKING
                    ^                |
                    +--- reset ------+
```

### 処理フロー

1. **起動時 (LOADING_MODEL)**: `sam2.1_hiera_small` モデルをGPUにロード
2. **WAITING_PROMPT**: RGBフレームを受信するが、マスクは配信しない。最新フレームのみ保持
3. **プロンプト受信**: クリック座標 (x, y) を受け取り、Video Predictor を初期化。最新フレーム + 座標でセグメンテーション開始
4. **TRACKING**: 毎フレーム Video Predictor でマスクを伝搬し `/trackdlo/segmentation_mask` に配信

### モデル選択

`sam2.1_hiera_small` を使用:
- RTX 3090 (24GB VRAM) で十分動作
- 推論速度と精度のバランスが良い
- パラメータ `sam2_model_size` で `tiny` / `small` / `base_plus` / `large` に切替可能

### パラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `sam2_model_size` | `"small"` | SAM2モデルサイズ |
| `sam2_checkpoint_dir` | `""` | チェックポイントディレクトリ（空なら自動ダウンロード） |
| `rgb_topic` | `/camera/color/image_raw` | RGB画像トピック |

## SetPrompt サービス定義

```
# trackdlo_msgs/srv/SetPrompt.srv
int32 x
int32 y
---
bool success
string message
```

1点クリックのみ。複数点やボックスは将来拡張。

## composite_view クリックUI

- composite_view の4パネル構成: Camera / Mask / Overlay / Results
- Camera パネル上でマウスクリック
- パネル内の相対座標を元画像座標に変換して `/trackdlo/sam2/set_prompt` を呼び出し
- SAM2 が TRACKING に遷移後、Camera パネル上に「SAM2 Tracking」とステータス表示

### 座標変換

composite_view は元画像を `(pw, ph)` にリサイズして表示。Camera パネルは左上 (0, 0) に配置。クリック座標 `(cx, cy)` を元画像座標に変換:

```python
orig_x = int(cx * orig_w / pw)
orig_y = int(cy * orig_h / ph)
```

## Launch 統合

`segmentation_mode` パラメータで制御:

| 値 | 動作 |
|---|---|
| `"internal"` (デフォルト) | 内蔵HSV。SAM2ノード起動しない |
| `"external"` | 外部ノード（手動起動）。SAM2ノード起動しない |
| `"sam2"` | SAM2ノードを自動起動。trackdlo_nodeは external モードで動作 |

launch ファイルで `segmentation_mode` パラメータを受け取り、`"sam2"` の場合のみ `sam2_segmentation` ノードを起動。trackdlo_node には `segmentation_mode: "external"` を渡す。

## 変更対象ファイル

### 新規作成

- `trackdlo_segmentation/trackdlo_segmentation/sam2_node.py` — SAM2 Video Predictor ノード
- `trackdlo_msgs/srv/SetPrompt.srv` — クリック座標サービス定義

### 変更

- `trackdlo_msgs/CMakeLists.txt` — SetPrompt.srv 追加
- `trackdlo_segmentation/setup.py` — `sam2_segmentation` エントリポイント追加
- `trackdlo_segmentation/package.xml` — `trackdlo_msgs` 依存追加
- `trackdlo_utils/trackdlo_utils/composite_view_node.py` — Camera パネルクリック → SetPrompt 呼び出し
- `trackdlo_utils/package.xml` — `trackdlo_msgs` 依存追加
- `trackdlo_bringup/launch/trackdlo.launch.py` — `segmentation_mode` launch引数追加、SAM2 ノード条件起動
- `trackdlo_bringup/config/realsense_params.yaml` — SAM2 パラメータ追加

### 変更なし

- `trackdlo_core/` — `segmentation_mode: "external"` の既存機能で対応
- `trackdlo_segmentation/trackdlo_segmentation/base.py` — SAM2 は Base を継承しない（Video Predictor はフレーム間状態を持つため）
