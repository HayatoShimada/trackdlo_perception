# trackdlo_perception API Reference

[日本語版はこちら](API.ja.md)

## ROS2 Nodes

### trackdlo (C++ Tracking Node)

Main tracking node that performs CPD-LLE deformable object tracking.

**Executable:** `trackdlo` (package: `trackdlo_core`)

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | RGB image (configurable via `rgb_topic`) |
| `/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | Depth image (configurable via `depth_topic`) |
| `/camera/aligned_depth_to_color/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics (configurable via `camera_info_topic`) |
| `/trackdlo/init_nodes` | sensor_msgs/PointCloud2 | Initial DLO node positions (from init_tracker) |
| `/trackdlo/segmentation_mask` | sensor_msgs/Image | External segmentation mask (mono8, only if `use_external_mask=true`) |
| `/mask_with_occlusion` | sensor_msgs/Image | Occlusion mask |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/trackdlo/results_pc` | sensor_msgs/PointCloud2 | **Main output:** tracked DLO node positions |
| `/trackdlo/results_img` | sensor_msgs/Image | Tracking visualization image |
| `/trackdlo/results_marker` | visualization_msgs/MarkerArray | Tracking results as 3D markers (for RViz) |
| `/trackdlo/guide_nodes` | visualization_msgs/MarkerArray | Guide nodes markers |
| `/trackdlo/filtered_pointcloud` | sensor_msgs/PointCloud2 | Downsampled input point cloud |
| `/trackdlo/segmentation_mask_img` | sensor_msgs/Image | Segmentation mask visualization |
| `/trackdlo/segmentation_overlay` | sensor_msgs/Image | Mask overlay on RGB image |
| `/trackdlo/self_occluded_pc` | sensor_msgs/PointCloud2 | Self-occluded nodes |
| `/trackdlo/corr_priors` | visualization_msgs/MarkerArray | Correspondence priors |

#### Parameters

**Algorithm (CPD-LLE):**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `beta` | double | 0.35 | Shape rigidity. Smaller = more flexible |
| `lambda` | double | 50000.0 | Global smoothness strength |
| `alpha` | double | 3.0 | Conformity to initial shape |
| `mu` | double | 0.1 | Noise ratio (0-1). Larger = noisier point cloud expected |
| `max_iter` | int | 50 | Maximum EM iterations per frame |
| `tol` | double | 0.0002 | EM convergence tolerance |
| `lle_weight` | double | 10.0 | LLE regularization weight |

**Visibility/Occlusion:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `k_vis` | double | 50.0 | Visibility term weight |
| `d_vis` | double | 0.06 | Max geodesic distance for gap interpolation (m) |
| `visibility_threshold` | double | 0.008 | Distance threshold for visible node (m) |
| `dlo_pixel_width` | int | 40 | Approximate DLO width in pixels |

**Preprocessing:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `beta_pre_proc` | double | 3.0 | Pre-processing registration beta |
| `lambda_pre_proc` | double | 1.0 | Pre-processing registration lambda |
| `downsample_leaf_size` | double | 0.008 | Voxel grid leaf size (m) |
| `multi_color_dlo` | bool | false | Enable multi-color DLO detection |

**Segmentation:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_external_mask` | bool | false | Use external mask instead of built-in HSV |
| `hsv_threshold_lower_limit` | string | "90 90 30" | HSV lower bounds "H S V" |
| `hsv_threshold_upper_limit` | string | "130 255 255" | HSV upper bounds "H S V" |

**Topics (configurable):**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_info_topic` | string | "/camera/aligned_depth_to_color/camera_info" | Camera info topic |
| `rgb_topic` | string | "/camera/color/image_raw" | RGB image topic |
| `depth_topic` | string | "/camera/aligned_depth_to_color/image_raw" | Depth image topic |
| `result_frame_id` | string | "camera_color_optical_frame" | TF frame for results |

---

### init_tracker (Python Initialization Node)

Extracts DLO skeleton from first frame and publishes initial node positions.

**Executable:** `init_tracker` (package: `trackdlo_core`)

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/trackdlo/init_nodes` | sensor_msgs/PointCloud2 | Initial node positions (published once) |
| `/trackdlo/init_nodes_markers` | visualization_msgs/MarkerArray | Initialization markers (for RViz) |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_of_nodes` | int | 30 | Number of tracking nodes |
| `multi_color_dlo` | bool | false | Multi-color DLO detection |
| `visualize_initialization_process` | bool | false | Show debug visualization |
| `use_external_mask` | bool | false | Use external segmentation mask |
| `camera_info_topic` | string | "/camera/color/camera_info" | Camera info topic |
| `rgb_topic` | string | "/camera/color/image_raw" | RGB image topic |
| `depth_topic` | string | "/camera/aligned_depth_to_color/image_raw" | Depth image topic |
| `result_frame_id` | string | "camera_color_optical_frame" | TF frame |
| `hsv_threshold_lower_limit` | string | "90 90 90" | HSV lower bounds |
| `hsv_threshold_upper_limit` | string | "130 255 255" | HSV upper bounds |

---

### hsv_segmentation (Python HSV Segmentation Node)

HSV color-based segmentation with optional interactive GUI tuner.

**Executable:** `hsv_segmentation` (package: `trackdlo_segmentation`)

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| RGB topic (configurable) | sensor_msgs/Image | RGB image input |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/trackdlo/segmentation_mask` | sensor_msgs/Image | Binary segmentation mask (mono8) |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rgb_topic` | string | "/camera/color/image_raw" | RGB image topic |
| `hsv_threshold_lower_limit` | string | "85 50 20" | HSV lower bounds "H S V" |
| `hsv_threshold_upper_limit` | string | "135 255 255" | HSV upper bounds "H S V" |
| `enable_gui` | bool | false | Enable interactive trackbar GUI |

---

## Launch Arguments

### trackdlo.launch.py

| Argument | Default | Choices | Description |
|----------|---------|---------|-------------|
| `segmentation` | `hsv` | `hsv`, `hsv_tuner` | Segmentation method. `hsv_tuner` launches HSV GUI node |
| `rviz` | `true` | — | Launch RViz2 |

```bash
ros2 launch trackdlo_bringup trackdlo.launch.py segmentation:=hsv_tuner rviz:=false
```

---

## C++ Library API

### trackdlo (CPD-LLE Algorithm)

```cpp
#include "trackdlo_core/trackdlo.hpp"

// Construct with parameters
trackdlo tracker(
  30,      // num_of_nodes
  0.008,   // visibility_threshold
  0.35,    // beta
  50000.0, // lambda
  3.0,     // alpha
  50.0,    // k_vis
  0.1,     // mu
  50,      // max_iter
  0.0002,  // tol
  3.0,     // beta_pre_proc
  1.0,     // lambda_pre_proc
  10.0);   // lle_weight

// Initialize
tracker.initialize_nodes(Y_init);  // Eigen::MatrixXd (N x 3)
tracker.initialize_geodesic_coord(geodesic_coord);  // std::vector<double>

// Track per frame
tracker.tracking_step(X, visible_nodes, visible_nodes_extended, proj_matrix, rows, cols);

// Get results
Eigen::MatrixXd result = tracker.get_tracking_result();  // (N x 3)
```

### ImagePreprocessor

```cpp
#include "trackdlo_core/image_preprocessor.hpp"

trackdlo_core::ImagePreprocessor preprocessor(
  false,                    // use_external_mask
  false,                    // multi_color_dlo
  {85, 50, 20},            // HSV lower
  {135, 255, 255});        // HSV upper

cv::Mat mask, cur_image;
bool ok = preprocessor.process(rgb_image, mask, cur_image);
```

### VisibilityChecker

```cpp
#include "trackdlo_core/visibility_checker.hpp"

trackdlo_core::VisibilityChecker checker;
trackdlo_core::VisibilityResult result = checker.check_visibility(
  Y, X, proj_matrix, mask, 0.008, 20);

// result.visible_nodes — indices of visible nodes
// result.not_self_occluded_nodes — indices not self-occluded
```

### PipelineManager

```cpp
#include "trackdlo_core/pipeline_manager.hpp"

trackdlo_core::PipelineManager pipeline(false, false, lower, upper);
pipeline.set_parameters(0.008, 20, 0.008, 0.06, 10);
pipeline.set_tracker_parameters(0.35, 3.0, 50000, 1.0, 3.0, 10.0, 0.1, 50, 0.0002, 50.0);
pipeline.initialize_tracker(init_nodes, geodesic_coord);

trackdlo_core::PipelineResult result = pipeline.process(rgb, depth, proj_matrix);
if (result.success) {
  // result.Y — tracked nodes (Eigen::MatrixXd, N x 3)
  // result.cur_pc_downsampled — filtered point cloud
}
```

---

## Python Segmentation API

### Creating a Custom Segmentation Node

```python
from trackdlo_segmentation import SegmentationNodeBase
import numpy as np

class MySegmentationNode(SegmentationNodeBase):
    """Custom segmentation backend."""

    def __init__(self):
        super().__init__('my_segmentation')  # ROS2 node name
        # Initialize your model here

    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        """Segment a BGR image.

        Args:
            cv_image: Input BGR image, shape (H, W, 3), dtype uint8.

        Returns:
            Binary mask, shape (H, W), dtype uint8, values 0 or 255.
        """
        # Your segmentation logic here
        mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        return mask

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MySegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

All segmentation nodes automatically:
- Subscribe to the RGB topic (configurable via `rgb_topic` parameter)
- Publish binary mask to `/trackdlo/segmentation_mask` (mono8)

### SegmentationNodeBase Reference

| Method | Description |
|--------|-------------|
| `__init__(node_name: str)` | Initialize with ROS2 node name |
| `segment(cv_image: np.ndarray) -> np.ndarray` | **Abstract.** Implement segmentation logic |
| `_on_image(msg: Image)` | Internal callback. Calls `segment()` and publishes mask |

| Attribute | Type | Description |
|-----------|------|-------------|
| `bridge` | CvBridge | ROS ↔ OpenCV converter |
| `mask_pub` | Publisher | Publisher for segmentation mask |
| `image_sub` | Subscription | Subscriber to RGB topic |
