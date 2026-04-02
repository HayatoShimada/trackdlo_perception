# Phase 1: GitHub Release 準備 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** trackdlo_ros2をREP-144準拠にリネームし、メタデータ修正・LICENSE追加・CUDAオプション化・SAM2分離を行い、GitHub Release v2.0.0を作成可能な状態にする。

**Architecture:** パッケージ`trackdlo_perception`→`trackdlo_core`にリネーム、リポジトリ名`trackdlo_ros2`→`trackdlo_perception`に変更、全103箇所のC++/Python/CMake/Launch/Docker参照を更新。CUDAはCheckLanguageで自動検出しオプション化。SAM2関連ファイルを削除。

**Tech Stack:** ROS2 (Humble/Jazzy), C++17, Python 3, CMake, ament_cmake, ament_python, Docker

---

## File Structure

### Files to Rename (directory)
- `trackdlo_perception/` → `trackdlo_core/`
- `trackdlo_perception/trackdlo_perception/` → `trackdlo_core/trackdlo_core/`
- `trackdlo_perception/include/trackdlo_perception/` → `trackdlo_core/include/trackdlo_core/`

### Files to Create
- `LICENSE` (BSD-3-Clause)

### Files to Delete
- `trackdlo_utils/trackdlo_utils/sam2_segmentation_node.py`
- `docker/Dockerfile.sam2`

### Files to Modify (major changes)
- `trackdlo_core/CMakeLists.txt` — project名リネーム + CUDAオプション化
- `trackdlo_core/package.xml` — パッケージ名 + maintainer
- All C++ source/header files in `trackdlo_core/` — namespace + include path
- All Python files in `trackdlo_core/trackdlo_core/` — import paths
- `trackdlo_core/scripts/init_tracker` — import path
- `trackdlo_bringup/launch/trackdlo.launch.py` — package references + SAM2削除
- `trackdlo_bringup/launch/evaluation.launch.py` — package reference
- `trackdlo_bringup/package.xml` — dependency name
- `trackdlo_bringup/CMakeLists.txt` — worlds/削除
- `trackdlo_bringup/config/realsense_params.yaml` — sam2セクション削除
- `trackdlo_utils/setup.py` — maintainer + SAM2/Gazeboエントリ削除
- `trackdlo_utils/package.xml` — maintainer
- `trackdlo_segmentation/setup.py` — maintainer + version
- `trackdlo_segmentation/package.xml` — maintainer + version + description
- `trackdlo_msgs/package.xml` — maintainer
- `docker/Dockerfile.core` — COPY path
- `docker/docker-compose.yml` — image names + SAM2削除
- `docker/docker-compose.nvidia.yml` — SAM2削除
- `docker/build.sh` — SAM2ターゲット削除
- `docker/run.sh` — SAM2モード削除
- `CLAUDE.md` — 全参照更新
- `README.md` — 英語版に書き直し
- `README.ja.md` — 日本語版（現README.mdベース）

---

## Task 1: Rename Package Directory and Update C++ Headers/Sources

**Files:**
- Rename: `trackdlo_perception/` → `trackdlo_core/`
- Rename: `trackdlo_core/trackdlo_perception/` → `trackdlo_core/trackdlo_core/`
- Rename: `trackdlo_core/include/trackdlo_perception/` → `trackdlo_core/include/trackdlo_core/`
- Modify: All `.hpp` and `.cpp` and `.cu` files — namespace + include paths

- [ ] **Step 1: Rename directories**

```bash
cd /home/user/repos/src/trackdlo_ros2
git mv trackdlo_perception trackdlo_core
git mv trackdlo_core/trackdlo_perception trackdlo_core/trackdlo_core
git mv trackdlo_core/include/trackdlo_perception trackdlo_core/include/trackdlo_core
```

- [ ] **Step 2: Update all C++ include directives**

In all `.cpp`, `.cu`, and `.hpp` files under `trackdlo_core/`, replace:
```
#include "trackdlo_perception/  →  #include "trackdlo_core/
```

Files affected (use `sed` or equivalent):
- `src/utils.cpp` (2 includes)
- `src/visualizer.cpp` (1 include)
- `src/image_preprocessor.cpp` (1 include)
- `src/trackdlo_node.cpp` (3 includes)
- `src/visibility_checker.cpp` (1 include)
- `src/pointcloud_cuda.cu` (1 include)
- `src/run_evaluation.cpp` (3 includes)
- `src/pipeline_manager.cpp` (2 includes)
- `src/trackdlo.cpp` (2 includes)
- `src/evaluator.cpp` (3 includes)
- `include/trackdlo_core/utils.hpp` (1 include)
- `include/trackdlo_core/evaluator.hpp` (1 include)
- `include/trackdlo_core/pipeline_manager.hpp` (4 includes)

- [ ] **Step 3: Update all C++ namespace declarations**

In all `.cpp`, `.cu`, and `.hpp` files, replace:
```
namespace trackdlo_perception  →  namespace trackdlo_core
trackdlo_perception::          →  trackdlo_core::
```

Files affected:
- `src/visualizer.cpp` — namespace open/close
- `src/image_preprocessor.cpp` — namespace open/close
- `src/trackdlo_node.cpp` — 3 qualified references (`trackdlo_perception::PipelineManager`, `trackdlo_perception::PipelineResult`)
- `src/visibility_checker.cpp` — namespace open/close
- `src/pointcloud_cuda.cu` — namespace open/close
- `src/pipeline_manager.cpp` — namespace open/close + 5 qualified references
- `include/trackdlo_core/visibility_checker.hpp` — namespace open/close
- `include/trackdlo_core/pointcloud_cuda.cuh` — namespace open/close
- `include/trackdlo_core/image_preprocessor.hpp` — namespace open/close
- `include/trackdlo_core/pipeline_manager.hpp` — namespace open/close + 3 qualified references
- `include/trackdlo_core/visualizer.hpp` — namespace open/close

- [ ] **Step 4: Update Python imports**

Edit `trackdlo_core/trackdlo_core/initialize.py`:
```python
# Change line 21 from:
from trackdlo_perception.utils import extract_connected_skeleton, ndarray2MarkerArray
# To:
from trackdlo_core.utils import extract_connected_skeleton, ndarray2MarkerArray
```

Edit `trackdlo_core/scripts/init_tracker`:
```python
# Change line 3 from:
from trackdlo_perception.initialize import main
# To:
from trackdlo_core.initialize import main
```

- [ ] **Step 5: Update CMakeLists.txt project name**

Edit `trackdlo_core/CMakeLists.txt` line 2:
```cmake
# Change from:
project(trackdlo_perception)
# To:
project(trackdlo_core)
```

- [ ] **Step 6: Update package.xml**

Edit `trackdlo_core/package.xml`:
```xml
<!-- Change line 4 from: -->
<name>trackdlo_perception</name>
<!-- To: -->
<name>trackdlo_core</name>
```

- [ ] **Step 7: Update launch file references**

Edit `trackdlo_bringup/launch/trackdlo.launch.py`:
```python
# Lines 59 and 73: change package='trackdlo_perception' to:
package='trackdlo_core',
```

Edit `trackdlo_bringup/launch/evaluation.launch.py`:
```python
# Line 24: change package='trackdlo_perception' to:
package='trackdlo_core',
```

Edit `trackdlo_bringup/package.xml`:
```xml
<!-- Change line 12 from: -->
<exec_depend>trackdlo_perception</exec_depend>
<!-- To: -->
<exec_depend>trackdlo_core</exec_depend>
```

- [ ] **Step 8: Update Docker references**

Edit `docker/Dockerfile.core` line 21:
```dockerfile
# Change from:
COPY trackdlo_perception/ src/trackdlo_perception/
# To:
COPY trackdlo_core/ src/trackdlo_core/
```

- [ ] **Step 9: Commit**

```bash
git add -A
git commit -m "refactor: rename trackdlo_perception package to trackdlo_core

REP-144 compliance: free the name trackdlo_perception for use
as the repository name. Update all C++ namespaces, includes,
Python imports, launch files, and Docker references."
```

---

## Task 2: Update Metadata Across All Packages

**Files:**
- Modify: `trackdlo_core/package.xml`
- Modify: `trackdlo_segmentation/package.xml`
- Modify: `trackdlo_segmentation/setup.py`
- Modify: `trackdlo_msgs/package.xml`
- Modify: `trackdlo_bringup/package.xml`
- Modify: `trackdlo_utils/package.xml`
- Modify: `trackdlo_utils/setup.py`

- [ ] **Step 1: Update all package.xml maintainer fields**

In each of the 5 package.xml files, change:
```xml
<maintainer email="todo@todo.com">TODO</maintainer>
```
To:
```xml
<maintainer email="info@85-store.com">Hayato Shimada</maintainer>
```

Files:
- `trackdlo_core/package.xml`
- `trackdlo_segmentation/package.xml`
- `trackdlo_msgs/package.xml`
- `trackdlo_bringup/package.xml`
- `trackdlo_utils/package.xml`

- [ ] **Step 2: Update all setup.py maintainer fields**

In `trackdlo_segmentation/setup.py` and `trackdlo_utils/setup.py`, change:
```python
maintainer='TODO',
maintainer_email='todo@todo.com',
```
To:
```python
maintainer='Hayato Shimada',
maintainer_email='info@85-store.com',
```

- [ ] **Step 3: Unify versions**

In `trackdlo_segmentation/package.xml`, change version from `1.0.0` to `2.0.0`:
```xml
<version>2.0.0</version>
```

In `trackdlo_segmentation/setup.py`, change version from `1.0.0` to `2.0.0`:
```python
version='2.0.0',
```

- [ ] **Step 4: Update descriptions to remove trackdlo_ros2 references**

In `trackdlo_segmentation/package.xml`:
```xml
<description>Pluggable segmentation interface for trackdlo_perception</description>
```

In `trackdlo_segmentation/setup.py`:
```python
description='Pluggable segmentation interface for trackdlo_perception',
```

In `trackdlo_segmentation/trackdlo_segmentation/base.py` docstring (line 1):
```python
"""Base class for all segmentation nodes in trackdlo_perception.
```

In `trackdlo_bringup/package.xml`:
```xml
<description>trackdlo_perception: Launch files, configuration, and bringup</description>
```

In `trackdlo_utils/setup.py`:
```python
description='trackdlo_perception: Visualization and parameter tuning tools',
```

In `trackdlo_utils/package.xml`:
```xml
<description>trackdlo_perception: Visualization and parameter tuning tools</description>
```

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "chore: update maintainer info and unify versions to 2.0.0

Set maintainer to Hayato Shimada <info@85-store.com> across
all package.xml and setup.py files. Unify trackdlo_segmentation
version from 1.0.0 to 2.0.0."
```

---

## Task 3: Add LICENSE File

**Files:**
- Create: `LICENSE`

- [ ] **Step 1: Create LICENSE file**

Create `LICENSE` at repository root with BSD-3-Clause text:

```
BSD 3-Clause License

Copyright (c) 2026, Hayato Shimada
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

- [ ] **Step 2: Commit**

```bash
git add LICENSE
git commit -m "chore: add BSD-3-Clause LICENSE file"
```

---

## Task 4: Make CUDA Optional in CMakeLists.txt

**Files:**
- Modify: `trackdlo_core/CMakeLists.txt`

- [ ] **Step 1: Replace CUDA configuration in CMakeLists.txt**

In `trackdlo_core/CMakeLists.txt`, replace line 11:
```cmake
enable_language(CUDA)
```

With:
```cmake
# Optional CUDA support
include(CheckLanguage)
check_language(CUDA)
option(USE_CUDA "Enable CUDA GPU acceleration" ${CMAKE_CUDA_COMPILER})
if(USE_CUDA AND CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  message(STATUS "CUDA found: GPU acceleration enabled")
else()
  set(USE_CUDA OFF)
  message(STATUS "CUDA not found: using CPU-only mode")
endif()
```

- [ ] **Step 2: Conditionally compile pointcloud_cuda.cu**

In `trackdlo_core/CMakeLists.txt`, replace the library source list (lines 37-46):
```cmake
# Core algorithm library
set(TRACKDLO_CORE_SOURCES
  src/trackdlo.cpp
  src/utils.cpp
  src/evaluator.cpp
  src/image_preprocessor.cpp
  src/visibility_checker.cpp
  src/visualizer.cpp
  src/pipeline_manager.cpp
)

if(USE_CUDA)
  list(APPEND TRACKDLO_CORE_SOURCES src/pointcloud_cuda.cu)
endif()

add_library(trackdlo_core SHARED ${TRACKDLO_CORE_SOURCES})

if(USE_CUDA)
  target_compile_definitions(trackdlo_core PUBLIC USE_CUDA)
endif()
```

- [ ] **Step 3: Commit**

```bash
git add trackdlo_core/CMakeLists.txt
git commit -m "feat: make CUDA optional with automatic detection

Use CheckLanguage to detect CUDA at configure time. When CUDA
is unavailable, pointcloud_cuda.cu is excluded and USE_CUDA
preprocessor define is not set, enabling CPU fallback."
```

---

## Task 5: Remove SAM2 and Gazebo-only Tools

**Files:**
- Delete: `trackdlo_utils/trackdlo_utils/sam2_segmentation_node.py`
- Delete: `docker/Dockerfile.sam2`
- Modify: `trackdlo_utils/setup.py` — remove sam2 + Gazebo entry points
- Modify: `trackdlo_bringup/launch/trackdlo.launch.py` — remove SAM2 node + choice
- Modify: `trackdlo_bringup/config/realsense_params.yaml` — remove sam2 section
- Modify: `docker/docker-compose.yml` — remove sam2 service
- Modify: `docker/docker-compose.nvidia.yml` — remove sam2 section
- Modify: `docker/build.sh` — remove sam2 targets
- Modify: `docker/run.sh` — remove sam2 mode

- [ ] **Step 1: Delete SAM2 files**

```bash
cd /home/user/repos/src/trackdlo_ros2
git rm trackdlo_utils/trackdlo_utils/sam2_segmentation_node.py
git rm docker/Dockerfile.sam2
```

- [ ] **Step 2: Clean up trackdlo_utils/setup.py entry points**

Remove these entry points from `trackdlo_utils/setup.py` console_scripts:
```python
            'simulate_occlusion = trackdlo_utils.simulate_occlusion:main',
            'simulate_occlusion_eval = trackdlo_utils.simulate_occlusion_eval:main',
            'depth_format_converter = trackdlo_utils.depth_format_converter:main',
            'sam2_segmentation = trackdlo_utils.sam2_segmentation_node:main',
```

Remaining entry points should be:
```python
    entry_points={
        'console_scripts': [
            'tracking_test = trackdlo_utils.tracking_test:main',
            'collect_pointcloud = trackdlo_utils.collect_pointcloud:main',
            'mask_node = trackdlo_utils.mask:main',
            'tracking_result_img = trackdlo_utils.tracking_result_img_from_pointcloud_topic:main',
            'composite_view = trackdlo_utils.composite_view_node:main',
            'param_tuner = trackdlo_utils.param_tuner_node:main',
        ],
    },
```

- [ ] **Step 3: Remove SAM2 from launch file**

In `trackdlo_bringup/launch/trackdlo.launch.py`:

Remove the SAM2 node block (lines 93-100):
```python
        # --- SAM2 Segmentation (only when segmentation:=sam2) ---
        Node(
            package='trackdlo_utils',
            executable='sam2_segmentation',
            name='sam2_segmentation',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('segmentation', 'sam2'),
        ),
```

Update the segmentation choices (remove 'sam2'):
```python
        DeclareLaunchArgument(
            'segmentation', default_value='hsv',
            description='Segmentation method: hsv (built-in), hsv_tuner (GUI)',
            choices=['hsv', 'hsv_tuner'],
        ),
```

- [ ] **Step 4: Remove SAM2 from realsense_params.yaml**

In `trackdlo_bringup/config/realsense_params.yaml`, delete the entire `sam2_segmentation` section:
```yaml
sam2_segmentation:
  ros__parameters:
    rgb_topic: "/camera/color/image_raw"
    sam2_checkpoint: "facebook/sam2.1-hiera-small"
    force_cpu: false
```

- [ ] **Step 5: Remove SAM2 from Docker files**

In `docker/docker-compose.yml`, delete the entire `trackdlo-sam2` service block (lines 50-68).

In `docker/docker-compose.nvidia.yml`, delete the `trackdlo-sam2` section:
```yaml
  trackdlo-sam2:
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
```

In `docker/build.sh`, remove the sam2 and sam2-cuda targets:
```bash
if [[ "$TARGET" == "all" || "$TARGET" == "sam2" ]]; then
  echo "=== Building SAM2 image (CPU) ==="
  docker compose --profile sam2 build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --build-arg SAM2_DEVICE=cpu trackdlo-sam2
fi

if [[ "$TARGET" == "sam2-cuda" ]]; then
  echo "=== Building SAM2 image (CUDA) ==="
  docker compose --profile sam2 build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --build-arg SAM2_DEVICE=cuda trackdlo-sam2
fi
```

In `docker/run.sh`, remove the sam2 mode. Replace the mode handling:
```bash
MODE="${1:-hsv}"

if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  echo "Usage: $0 [hsv|hsv_tuner] [docker-compose options...]"
  echo ""
  echo "  Modes:"
  echo "    hsv           - HSV segmentation, no GUI (default)"
  echo "    hsv_tuner     - HSV segmentation with tuner GUI"
  echo ""
  echo "  Examples:"
  echo "    $0                            # HSV segmentation"
  echo "    $0 hsv_tuner                  # HSV with tuner GUI"
  echo "    $0 hsv -d                     # Detached mode"
  echo ""
  echo "  Environment variables:"
  echo "    ROS_DISTRO=jazzy $0           # Use Jazzy"
  exit 0
fi

COMPOSE_FILES="-f docker-compose.yml"

# Add NVIDIA GPU support if available
if command -v nvidia-smi &> /dev/null; then
  COMPOSE_FILES="$COMPOSE_FILES -f docker-compose.nvidia.yml"
  echo "=== NVIDIA GPU detected ==="
fi

echo "=== Starting core (segmentation=${MODE}) ==="
SEGMENTATION="$MODE" docker compose $COMPOSE_FILES up trackdlo-core "${@:2}"
```

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "refactor: remove SAM2 and Gazebo-only tools

SAM2 will be provided as a separate package. Remove
sam2_segmentation_node, Dockerfile.sam2, and all SAM2
references from launch/compose/scripts. Remove Gazebo-only
entry points (simulate_occlusion, depth_format_converter)."
```

---

## Task 6: Fix trackdlo_bringup CMakeLists.txt

**Files:**
- Modify: `trackdlo_bringup/CMakeLists.txt`

- [ ] **Step 1: Remove worlds/ install directive**

In `trackdlo_bringup/CMakeLists.txt`, delete lines 21-24:
```cmake
# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
```

- [ ] **Step 2: Commit**

```bash
git add trackdlo_bringup/CMakeLists.txt
git commit -m "fix: remove non-existent worlds/ directory from CMakeLists.txt"
```

---

## Task 7: Update Docker Image Names

**Files:**
- Modify: `docker/Dockerfile.core`
- Modify: `docker/docker-compose.yml`
- Modify: `docker/.env`

- [ ] **Step 1: Update Dockerfile.core**

In `docker/Dockerfile.core`, change line 2:
```dockerfile
ARG BASE_IMAGE=trackdlo_ros2-base:latest
```
To:
```dockerfile
ARG BASE_IMAGE=trackdlo_perception-base:latest
```

- [ ] **Step 2: Update docker-compose.yml image names**

In `docker/docker-compose.yml`, replace all `trackdlo_ros2-` with `trackdlo_perception-`:
- Line 9: `image: trackdlo_perception-base:latest`
- Line 19: `BASE_IMAGE: trackdlo_perception-base:latest`
- Line 20: `image: trackdlo_perception-core:latest`

Also update the colcon build command to use `trackdlo_core` instead of any `trackdlo_perception` references in the command block.

- [ ] **Step 3: Commit**

```bash
git add docker/
git commit -m "chore: rename Docker images from trackdlo_ros2 to trackdlo_perception"
```

---

## Task 8: Update Documentation (README English + Japanese)

**Files:**
- Create: `README.ja.md` (move current Japanese README)
- Modify: `README.md` (rewrite in English)
- Modify: `CLAUDE.md` (update all references)

- [ ] **Step 1: Move current README to README.ja.md**

```bash
cd /home/user/repos/src/trackdlo_ros2
git mv README.md README.ja.md
```

- [ ] **Step 2: Update README.ja.md references**

In `README.ja.md`, replace all `trackdlo_perception` package references with `trackdlo_core`, and `trackdlo_ros2` with `trackdlo_perception`. Update the package structure tree, build commands, and architecture diagrams.

- [ ] **Step 3: Create English README.md**

Create a new `README.md` in English covering:
- Project description (DLO tracking with RealSense RGB-D)
- Credit to RMDLO/trackdlo
- Package structure (trackdlo_core, trackdlo_segmentation, trackdlo_msgs, trackdlo_bringup, trackdlo_utils)
- Quick start (Docker + native build)
- Segmentation modes (hsv, hsv_tuner)
- Pluggable segmentation architecture (SegmentationNodeBase)
- Docker architecture diagram
- Processing pipeline
- Integration with other ROS2 projects
- Key parameters table
- Dependencies
- License (BSD-3-Clause)

Add link at top: `[日本語版はこちら](README.ja.md)`

- [ ] **Step 4: Update CLAUDE.md**

Replace all `trackdlo_perception` package references with `trackdlo_core` in CLAUDE.md. Update build commands, file paths, package table, and architecture descriptions.

- [ ] **Step 5: Commit**

```bash
git add README.md README.ja.md CLAUDE.md
git commit -m "docs: add English README, update all documentation

Create English README.md as default. Move Japanese version to
README.ja.md. Update all trackdlo_perception references to
trackdlo_core throughout documentation."
```

---

## Task 9: Rename GitHub Repository

**Files:** None (GitHub operation)

- [ ] **Step 1: Rename repository on GitHub**

```bash
gh repo rename trackdlo_perception --repo HayatoShimada/trackdlo_ros2
```

- [ ] **Step 2: Update local remote**

```bash
cd /home/user/repos/src/trackdlo_ros2
git remote set-url origin git@github.com:HayatoShimada/trackdlo_perception.git
```

- [ ] **Step 3: Push all changes**

```bash
git push origin master
```

- [ ] **Step 4: Create GitHub Release**

```bash
git tag -a v2.0.0 -m "v2.0.0: First release of trackdlo_perception

- ROS2 Humble/Jazzy support
- Pluggable segmentation architecture (SegmentationNodeBase)
- HSV segmentation with GUI tuner
- Optional CUDA GPU acceleration
- Docker support with auto GPU detection
- 4-panel composite preview window
- CPD-LLE parameter tuner GUI

Based on TrackDLO (https://github.com/RMDLO/trackdlo)"

git push origin v2.0.0
```

```bash
gh release create v2.0.0 --title "v2.0.0" --notes "$(cat <<'EOF'
## trackdlo_perception v2.0.0

First release of trackdlo_perception — a ROS2 package for real-time tracking of Deformable Linear Objects (DLO) using RGB-D cameras.

### Features
- CPD-LLE algorithm for real-time DLO tracking
- Pluggable segmentation architecture (`SegmentationNodeBase`)
- HSV segmentation with interactive GUI tuner
- Optional CUDA GPU acceleration (auto-detected)
- Docker support with NVIDIA GPU auto-detection
- 4-panel composite preview window
- ROS2 Humble and Jazzy support

### Packages
- `trackdlo_core` — Core CPD-LLE tracking algorithm
- `trackdlo_segmentation` — Pluggable segmentation interface + HSV
- `trackdlo_msgs` — Custom messages (reserved)
- `trackdlo_bringup` — Launch files and configuration
- `trackdlo_utils` — Visualization and parameter tuning tools

### Based on
[TrackDLO](https://github.com/RMDLO/trackdlo) by RMDLO
EOF
)"
```
