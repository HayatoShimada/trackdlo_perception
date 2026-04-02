# trackdlo_perception

Real-time tracking of Deformable Linear Objects (DLO) using ROS2 and Intel RealSense RGB-D cameras.

[日本語版はこちら](README.ja.md)

Based on [TrackDLO](https://github.com/RMDLO/trackdlo) by RMDLO.

## Features

- **Real-time DLO tracking** via CPD-LLE algorithm
- **Pluggable segmentation**: HSV / YOLO / DeepLab (extensible via `SegmentationNodeBase`)
- **4-panel preview window**: camera feed, mask, overlay, and tracking results
- **Docker-based**: single command launch with automatic GPU detection
- **ROS2 Humble / Jazzy**: switch via `ROS_DISTRO` build argument

## Package Structure

```
trackdlo_perception/
├── trackdlo_core/           CPD-LLE tracking algorithm (C++17 + Python)
├── trackdlo_segmentation/   Segmentation base class + HSV implementation
├── trackdlo_utils/          Composite view, parameter tuner
├── trackdlo_bringup/        Launch files, YAML params, RViz config
├── trackdlo_msgs/           Custom messages (reserved for future use)
└── docker/                  Docker Compose + GPU configuration
```

## Installation

### From apt (ROS2 Humble)

```bash
echo "deb [trusted=yes] https://hayatoshimada.github.io/trackdlo_perception/humble /" \
  | sudo tee /etc/apt/sources.list.d/trackdlo.list
sudo apt update
sudo apt install ros-humble-trackdlo-core ros-humble-trackdlo-segmentation
```

### From Source

```bash
cd ~/ros2_ws/src
git clone https://github.com/HayatoShimada/trackdlo_perception.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Quick Start

### Docker Build

```bash
cd docker/

# Build all images
bash build.sh

# Build core only
bash build.sh core

# Build for Jazzy
ROS_DISTRO=jazzy bash build.sh
```

### Docker Run

```bash
xhost +local:docker
cd docker/

# HSV segmentation (default)
./run.sh

# HSV tuner GUI
./run.sh hsv_tuner

# Background mode
./run.sh hsv -d
```

### Native Build

```bash
# Build
colcon build --packages-select trackdlo_msgs trackdlo_segmentation trackdlo_core trackdlo_utils trackdlo_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Launch
ros2 launch trackdlo_bringup trackdlo.launch.py
ros2 launch trackdlo_bringup trackdlo.launch.py segmentation:=hsv_tuner
ros2 launch trackdlo_bringup trackdlo.launch.py rviz:=false
```

## Segmentation Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| `hsv` (default) | HSV thresholding | DLO color is known, parameters tuned |
| `hsv_tuner` | Real-time slider GUI | Finding HSV values for a new DLO |

## Architecture

### Docker

```
trackdlo-core container
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

### Processing Pipeline

```
[RealSense D435/D415]
    |
    +-- /camera/color/image_raw
    +-- /camera/aligned_depth_to_color/image_raw
    |
    v
[Segmentation]  <- HSV / YOLO / DeepLab (swappable)
    |
    v /trackdlo/segmentation_mask
    |
[trackdlo_node (CPD-LLE)]
    |
    +-- /trackdlo/results_pc      (tracked DLO node positions)
    +-- /trackdlo/results_img     (tracking result visualization)
    |
    v
[composite_view]  <- 4-panel preview window
```

## Usage

### 1. Launch with RealSense Camera

Connect an Intel RealSense D415/D435/D455 and run:

```bash
# Default HSV segmentation
ros2 launch trackdlo_bringup trackdlo.launch.py

# With HSV tuner GUI (adjust thresholds interactively)
ros2 launch trackdlo_bringup trackdlo.launch.py segmentation:=hsv_tuner

# Without RViz
ros2 launch trackdlo_bringup trackdlo.launch.py rviz:=false
```

The 4-panel preview window opens automatically showing camera feed, segmentation mask, overlay, and tracking results.

### 2. Subscribe to Tracking Results

From your own ROS2 node, subscribe to the tracked DLO positions:

```python
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.create_subscription(
            PointCloud2, '/trackdlo/results_pc', self.on_tracking, 10)

    def on_tracking(self, msg):
        # Each point is a tracked DLO node (x, y, z in camera frame)
        points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z')))
        # points[0] = first endpoint, points[-1] = last endpoint
```

### 3. Use as a Dependency in Your Package

Add to your `package.xml`:

```xml
<exec_depend>trackdlo_core</exec_depend>
```

### Available Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/trackdlo/results_pc` | PointCloud2 | Tracked DLO node positions (main output) |
| `/trackdlo/results_img` | Image | Tracking result visualization |
| `/trackdlo/segmentation_mask` | Image | Segmentation mask (mono8) |
| `/trackdlo/init_nodes` | PointCloud2 | Initial nodes (published once) |

### 4. Add Custom Segmentation

Create a new segmentation backend by subclassing `SegmentationNodeBase`:

```python
from trackdlo_segmentation import SegmentationNodeBase
import numpy as np

class YoloSegmentationNode(SegmentationNodeBase):
    def __init__(self):
        super().__init__('yolo_segmentation')
        # Initialize your model here

    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        # Input: BGR image (H, W, 3), dtype uint8
        # Output: binary mask (H, W), dtype uint8, values 0 or 255
        mask = your_model.predict(cv_image)
        return mask
```

Run your node alongside trackdlo with external mask mode:

```bash
# Terminal 1: launch trackdlo with external mask
ros2 launch trackdlo_bringup trackdlo.launch.py segmentation:=hsv_tuner

# Terminal 2: run your segmentation node (replaces HSV)
ros2 run your_package yolo_segmentation
```

All segmentation nodes publish to `/trackdlo/segmentation_mask` (mono8).

## Integration with Other ROS2 Projects

trackdlo_perception uses only standard ROS2 message types. Any ROS2 node sharing the same `ROS_DOMAIN_ID` can subscribe to tracking results — no custom messages needed.

## Key Parameters

Configured in `trackdlo_bringup/config/realsense_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `beta` | 0.35 | Shape rigidity (smaller = more flexible) |
| `lambda` | 50000.0 | Global smoothness strength |
| `alpha` | 3.0 | Conformity to initial shape |
| `mu` | 0.1 | Noise ratio |
| `max_iter` | 20 | Maximum EM iterations |
| `k_vis` | 50.0 | Visibility term weight |
| `d_vis` | 0.06 | Max geodesic distance for gap interpolation (m) |
| `visibility_threshold` | 0.008 | Visibility distance threshold (m) |
| `downsample_leaf_size` | 0.02 | Voxel size (m) |
| `num_of_nodes` | 30 | Number of tracking nodes |

## Dependencies

- ROS2 Humble or Jazzy
- Intel RealSense SDK 2.0 (realsense2_camera)
- OpenCV, PCL, Eigen3
- scikit-image, scipy, Open3D

## License

BSD-3-Clause
