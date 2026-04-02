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

### Pluggable Segmentation

Add new segmentation backends by subclassing `SegmentationNodeBase`:

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

All segmentation nodes publish to `/trackdlo/segmentation_mask` (mono8).

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

## Integration with Other ROS2 Projects

trackdlo_perception uses only standard ROS2 message types. Subscribe from any container sharing the same `ROS_DOMAIN_ID`:

```python
# Subscribe to tracking results from another project
self.create_subscription(PointCloud2, '/trackdlo/results_pc', self.callback, 10)
```

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
