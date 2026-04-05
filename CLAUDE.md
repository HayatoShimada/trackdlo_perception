# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

trackdlo_perception is a ROS2 (Humble/Jazzy) system for real-time tracking of Deformable Linear Objects (DLO). It uses RGB-D camera input (Intel RealSense D415/D435/D455) to detect and track DLO via the CPD-LLE algorithm, with a pluggable segmentation architecture and a 4-panel preview window.

## Build & Development Commands

### Native Build (ROS2 workspace)
```bash
# Build all packages (order matters: bringup must be last)
colcon build --packages-select trackdlo_msgs trackdlo_segmentation trackdlo_core trackdlo_utils trackdlo_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build single package
colcon build --packages-select trackdlo_core --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source after build
source install/setup.bash
```

### Docker Build & Run
```bash
cd docker/

# Build all images
bash build.sh

# Build core only
bash build.sh core

# Run
./run.sh                  # HSV segmentation (default)
./run.sh hsv_tuner        # HSV with tuner GUI
./run.sh hsv -d           # Detached mode

# Build for Jazzy
ROS_DISTRO=jazzy bash build.sh
```

Note: The core container rebuilds `trackdlo_utils` and `trackdlo_segmentation` on startup, so Python changes in those packages take effect without a full Docker rebuild.

### Tests & Linting
```bash
# Run all tests (includes ament_lint: copyright, flake8, pep257)
colcon test --packages-select trackdlo_segmentation trackdlo_core trackdlo_utils trackdlo_bringup --return-code-on-test-failure

# View test results
colcon test-result --verbose

# Run a single C++ test (after build)
./build/trackdlo_core/test_trackdlo
./build/trackdlo_core/test_image_preprocessor
./build/trackdlo_core/test_visibility_checker

# Run a single Python test
python3 -m pytest trackdlo_segmentation/test/test_base.py -v
python3 -m pytest trackdlo_segmentation/test/test_hsv_node.py -v

# Quick Python lint (without colcon test)
flake8 trackdlo_core/trackdlo_core/ trackdlo_utils/trackdlo_utils/ trackdlo_segmentation/trackdlo_segmentation/ --max-line-length=150 --ignore=E501,W503,E741
```

### Launch Commands
```bash
# Default (HSV segmentation)
ros2 launch trackdlo_bringup trackdlo.launch.py

# HSV tuner GUI
ros2 launch trackdlo_bringup trackdlo.launch.py segmentation:=hsv_tuner

# Without RViz
ros2 launch trackdlo_bringup trackdlo.launch.py rviz:=false
```

## Architecture

### Build Order & Dependencies
```
trackdlo_msgs ─────────┐
trackdlo_core ─────────┤ (these 3 are independent, can build in parallel)
trackdlo_segmentation ─┤
trackdlo_utils ────────┤
                       └──→ trackdlo_bringup (exec_depends on core, segmentation, utils)
```

### Package Structure

| Package | Language | Build | Role |
|---------|----------|-------|------|
| `trackdlo_core` | C++17 + Python | ament_cmake | Core CPD-LLE tracking algorithm + initialization |
| `trackdlo_segmentation` | Python | ament_python | Pluggable segmentation interface (base class + HSV) |
| `trackdlo_utils` | Python | ament_python | Composite view, param tuner, test tools |
| `trackdlo_bringup` | Launch/Config | ament_cmake | Launch files, YAML params, RViz config |
| `trackdlo_msgs` | ROS IDL | ament_cmake | Custom messages (reserved for future use) |

### Processing Pipeline
1. **Initialization** (`trackdlo_core/trackdlo_core/initialize.py`): HSV threshold -> skeleton extraction -> spline fitting -> equally-spaced 3D nodes -> publishes once to `/trackdlo/init_nodes`
2. **Per-frame tracking** (`trackdlo_core/src/trackdlo_node.cpp` + `trackdlo.cpp`): RGB-D sync -> segmentation mask -> point cloud -> voxel downsample -> visibility estimation -> CPD-LLE EM iterations -> updated node positions

### Key Source Files
- `trackdlo_core/src/trackdlo.cpp` — CPD-LLE algorithm core (cpd_lle method)
- `trackdlo_core/src/trackdlo_node.cpp` — ROS2 node: image sync, preprocessing, visibility detection
- `trackdlo_core/src/utils.cpp` — Projection, depth conversion, occlusion helpers
- `trackdlo_core/trackdlo_core/initialize.py` — Initialization pipeline (InitTrackerNode)
- `trackdlo_segmentation/trackdlo_segmentation/base.py` — SegmentationNodeBase (pluggable interface)
- `trackdlo_segmentation/trackdlo_segmentation/hsv_node.py` — HSV segmentation with GUI tuner
- `trackdlo_bringup/config/realsense_params.yaml` — Tracking parameters (CPD-LLE tuning, HSV thresholds, topic names)
- `trackdlo_bringup/config/evaluation_params.yaml` — Evaluation/benchmarking parameters

### Key ROS2 Topics
- `/trackdlo/init_nodes` (PointCloud2) — Initial nodes, published once
- `/trackdlo/results_pc` (PointCloud2) — Per-frame tracked node positions
- `/trackdlo/segmentation_mask` (Image) — Segmentation mask (from HSV or external)
- `/trackdlo/results_img` (Image) — Tracking result visualization

### Segmentation Architecture
All segmentation nodes inherit from `SegmentationNodeBase` and publish to `/trackdlo/segmentation_mask` (mono8). New backends (YOLO, DeepLab, etc.) can be added by subclassing `SegmentationNodeBase`.

### CI/CD
- `.github/workflows/build.yml` — Build and test on push/PR for both Humble and Jazzy distros (CUDA disabled in CI: `-DUSE_CUDA=OFF`)
- `.github/workflows/release-deb.yml` — Triggered on version tags (`v*`), builds Debian packages via bloom, publishes to gh-pages apt repository

## Code Conventions

- C++ standard: C++17 with `-O3` optimization for perception; optional CUDA support (`-DUSE_CUDA=ON`)
- Commit messages: conventional commits (`feat:`, `fix:`, `docs:`, etc.), both Japanese and English
- Python linting: flake8 with max-line-length=150 (ament_lint runs flake8+pep257+copyright in `colcon test`)
- ROS2 nodes: C++ nodes inherit `rclcpp::Node`, Python from `rclpy.node.Node`
- All nodes declare parameters with `declare_parameter()`
- RGB-D synchronization uses `message_filters::ApproximateTimeSynchronizer`
- Perception nodes use `respawn=True, respawn_delay=3.0` in launch files
- Thread safety via `std::mutex` and `std::atomic` in C++ nodes
- Segmentation nodes inherit from `trackdlo_segmentation.SegmentationNodeBase`
