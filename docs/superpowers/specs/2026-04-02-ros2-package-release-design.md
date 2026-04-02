# trackdlo_perception ROS2パッケージリリース 設計ドキュメント

## 概要

trackdlo_ros2をROS2公式パッケージとしてリリース可能にする。段階的に GitHub Release → bloom release (PPA) → rosdistro登録 → Docker Hub配布を実現する。

## 命名規則 (REP-144準拠)

- リポジトリ名: `trackdlo_perception`（`ros`を含まない）
- 元のC++パッケージ`trackdlo_perception`は`trackdlo_core`にリネーム
- TrackDLOの出自: https://github.com/RMDLO/trackdlo
- maintainer: Hayato Shimada <info@85-store.com>

## リネーム対応表

| 変更前 | 変更後 |
|---|---|
| リポジトリ `trackdlo_ros2` | `trackdlo_perception` |
| パッケージ `trackdlo_perception` | `trackdlo_core` |
| ディレクトリ `trackdlo_perception/` | `trackdlo_core/` |
| Pythonパッケージ `trackdlo_perception` | `trackdlo_core` |
| CMakeプロジェクト名 `trackdlo_perception` | `trackdlo_core` |
| Dockerイメージ `trackdlo_ros2-*` | `trackdlo_perception-*` |

変更しないもの:
- ノード実行ファイル名: `trackdlo`（そのまま）
- ライブラリ名: `trackdlo_core`（既にこの名前）
- トピック名: `/trackdlo/*`（そのまま）
- 他のパッケージ名: `trackdlo_segmentation`, `trackdlo_msgs`, `trackdlo_bringup`, `trackdlo_utils`

## パッケージ構成とリリース対象

```
trackdlo_perception/ (リポジトリ)
├── trackdlo_core/             # CPD-LLEアルゴリズム + 初期化
├── trackdlo_segmentation/     # セグメンテーション基底クラス + HSV
├── trackdlo_msgs/             # カスタムメッセージ（将来用）
├── trackdlo_bringup/          # Launch, config, RViz
├── trackdlo_utils/            # composite_view, param_tuner
└── docker/
```

### リリース対象

| パッケージ | bloom (PPA) | GitHub Release | Docker Hub |
|---|---|---|---|
| `trackdlo_core` | Yes | Yes | Yes |
| `trackdlo_segmentation` | Yes | Yes | Yes |
| `trackdlo_msgs` | Yes | Yes | Yes |
| `trackdlo_bringup` | Yes | Yes | Yes |
| `trackdlo_utils` | Yes | Yes | Yes |

注: rosdistro登録は対象外。bloom PPAで十分な配布を実現する。

### SAM2の扱い

SAM2セグメンテーションノードはtorch/CUDA依存が重いため、trackdlo_utilsから削除する。将来的に別リポジトリ`trackdlo_segmentation_sam2`として分離可能。Dockerfile.sam2とdocker-compose.ymlのsam2 profileも削除する。

## Phase 1: GitHubリリース準備

### 1. リネーム

影響範囲:
- `trackdlo_core/package.xml` — パッケージ名、説明
- `trackdlo_core/CMakeLists.txt` — project名、ament_python_install_packageのパッケージ名
- `trackdlo_core/trackdlo_core/` — Pythonパッケージディレクトリ名
- `trackdlo_core/trackdlo_core/__init__.py`
- `trackdlo_core/trackdlo_core/initialize.py`
- `trackdlo_core/trackdlo_core/utils.py`
- `trackdlo_core/scripts/init_tracker` — import文
- `trackdlo_bringup/launch/trackdlo.launch.py` — `package='trackdlo_core'`
- `trackdlo_bringup/launch/evaluation.launch.py` — パッケージ参照
- `trackdlo_bringup/package.xml` — exec_depend
- `docker/Dockerfile.core` — COPYパス
- `docker/docker-compose.yml` — colcon buildパッケージ名
- `CLAUDE.md`, `README.md` — ドキュメント内の参照

### 2. メタデータ修正

全package.xml:
```xml
<maintainer email="info@85-store.com">Hayato Shimada</maintainer>
```

全setup.py:
```python
maintainer='Hayato Shimada',
maintainer_email='info@85-store.com',
```

バージョン: 全パッケージ `2.0.0` に統一。

### 3. LICENSEファイル

リポジトリルートにBSD-3-Clauseライセンスファイルを追加。

### 4. CUDAオプション化

`trackdlo_core/CMakeLists.txt`:
```cmake
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  set(USE_CUDA ON)
  message(STATUS "CUDA found: GPU acceleration enabled")
else()
  set(USE_CUDA OFF)
  message(STATUS "CUDA not found: using CPU-only mode")
endif()

# pointcloud_cuda.cuの条件付きコンパイル
if(USE_CUDA)
  list(APPEND CORE_SOURCES src/pointcloud_cuda.cu)
  target_compile_definitions(trackdlo_core PUBLIC USE_CUDA)
endif()
```

CUDAなしの場合、pointcloud_cuda.cuを除外し、C++側でCPUフォールバックを使用。

### 5. SAM2の分離

削除対象:
- `trackdlo_utils/trackdlo_utils/sam2_segmentation_node.py`
- `trackdlo_utils/setup.py`のsam2_segmentationエントリポイント
- `docker/Dockerfile.sam2`
- `docker/docker-compose.yml`のtrackdlo-sam2サービス
- `docker/docker-compose.nvidia.yml`のtrackdlo-sam2セクション
- `trackdlo_bringup/launch/trackdlo.launch.py`のSAM2ノード定義
- `trackdlo_bringup/config/realsense_params.yaml`のsam2_segmentationセクション
- launch引数`segmentation`のchoicesから`sam2`を削除

### 6. その他の修正

- `trackdlo_bringup/CMakeLists.txt`: worlds/ディレクトリ参照の削除
- `trackdlo_utils`内のGazebo専用ツール（`depth_format_converter`, `simulate_occlusion`, `simulate_occlusion_eval`）はエントリポイントから削除。ファイルは残しても害はないが、setup.pyのconsole_scriptsからは除外する

### 7. GitHub Release

- タグ `v2.0.0` を作成
- リリースノートを記載

## Phase 2: bloom release (PPA配布)

### bloom設定

```bash
bloom-release --rosdistro humble --track humble trackdlo_perception
```

- リリースリポジトリ `trackdlo_perception-release` がGitHubに自動作成される
- 全5パッケージが個別のdebianパッケージとしてビルドされる

### CI/CD (GitHub Actions)

ワークフロー `.github/workflows/build.yml`:
- マトリクス: Humble × amd64, Jazzy × amd64
- ステップ:
  1. `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`（CUDA OFFでビルド）
  2. `colcon test --return-code-on-test-failure`
  3. `flake8` lint
- トリガー: push to master, PR

### rosdepキー

全package.xmlの依存がrosdepキーとして解決可能であることを確認。カスタムrosdepキーが必要な場合は`rosdep/sources.list.d`に追加。

## Phase 3: Docker Hub

- イメージ名: `hayatoshimada/trackdlo_perception`
- タグ: `humble`, `jazzy`, `humble-cuda`, `latest`
- GitHub Actions: タグpush時に自動ビルド+push

### README英語化

README.md（英語版）とREADME.ja.md（日本語版）を作成。README.mdがデフォルトで表示される英語版。
