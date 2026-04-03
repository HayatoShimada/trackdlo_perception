# librealsense2 直接統合 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** trackdlo_nodeにlibrealsense2 SDKを直接統合し、HSVセグメンテーションをC++に内蔵、外部制御サービスとステータス配信を追加する。

**Architecture:** realsense2_camera_nodeとPython HSVセグメンテーションを廃止し、trackdlo_node (C++)内でlibrealsense2から直接フレーム取得→HSVマスク生成→CPD-LLE追跡を行う。ROS2 Trigger Serviceで外部から start/stop/reset を制御し、カスタムメッセージで追跡状態を配信する。

**Tech Stack:** C++17, librealsense2, OpenCV, ROS2 (Humble/Jazzy), std_srvs, trackdlo_msgs

**Spec:** `docs/superpowers/specs/2026-04-03-librealsense-integration-design.md`

---

## File Structure

### 新規作成
- `trackdlo_msgs/msg/TrackingStatus.msg` — カスタムステータスメッセージ

### 変更
- `trackdlo_msgs/CMakeLists.txt` — メッセージ生成有効化
- `trackdlo_msgs/package.xml` — メッセージ生成依存追加（既存で対応済みか確認）
- `trackdlo_core/CMakeLists.txt` — librealsense2, std_srvs, trackdlo_msgs 依存追加
- `trackdlo_core/package.xml` — 同上の依存追加
- `trackdlo_core/src/trackdlo_node.cpp` — librealsense2統合、HSV内蔵、サービス、ステータス
- `trackdlo_bringup/launch/trackdlo.launch.py` — realsense/hsv_segmentationノード廃止
- `trackdlo_bringup/config/realsense_params.yaml` — カメラパラメータ追加、segmentation_mode追加
- `trackdlo_utils/trackdlo_utils/param_tuner_node.py` — Start/Stop/Resetボタン + ステータス表示

---

### Task 1: TrackingStatus メッセージ定義

**Files:**
- Create: `trackdlo_msgs/msg/TrackingStatus.msg`
- Modify: `trackdlo_msgs/CMakeLists.txt`

- [ ] **Step 1: メッセージファイル作成**

```
# trackdlo_msgs/msg/TrackingStatus.msg
uint8 state
uint8 IDLE=0
uint8 INITIALIZING=1
uint8 TRACKING=2
uint8 ERROR=3

string message
int32 tracked_nodes
float64 fps
```

- [ ] **Step 2: CMakeLists.txt でメッセージ生成を有効化**

`trackdlo_msgs/CMakeLists.txt` のコメントアウトされた `rosidl_generate_interfaces` を置き換える:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackingStatus.msg"
  DEPENDENCIES std_msgs
)
```

- [ ] **Step 3: ビルド確認**

Run: `cd ~/repos && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-select trackdlo_msgs`
Expected: ビルド成功、`TrackingStatus.msg` が生成される

- [ ] **Step 4: メッセージ確認**

Run: `source install/setup.bash && ros2 interface show trackdlo_msgs/msg/TrackingStatus`
Expected: メッセージ定義が表示される

- [ ] **Step 5: コミット**

```bash
git add trackdlo_msgs/msg/TrackingStatus.msg trackdlo_msgs/CMakeLists.txt
git commit -m "feat(msgs): add TrackingStatus message for external control"
```

---

### Task 2: trackdlo_core ビルド依存追加

**Files:**
- Modify: `trackdlo_core/CMakeLists.txt`
- Modify: `trackdlo_core/package.xml`

- [ ] **Step 1: package.xml に依存追加**

`trackdlo_core/package.xml` の `<depend>` セクションに以下を追加:

```xml
<depend>std_srvs</depend>
<depend>trackdlo_msgs</depend>
```

`<build_depend>` セクションに以下を追加:

```xml
<build_depend>librealsense2</build_depend>
```

- [ ] **Step 2: CMakeLists.txt に find_package 追加**

`trackdlo_core/CMakeLists.txt` の `find_package` セクションに追加:

```cmake
find_package(realsense2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trackdlo_msgs REQUIRED)
```

`trackdlo` 実行ファイルの `target_link_libraries` に追加:

```cmake
target_link_libraries(trackdlo trackdlo_core ${realsense2_LIBRARY})
```

`trackdlo` 実行ファイルの `ament_target_dependencies` に追加:

```cmake
ament_target_dependencies(trackdlo
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  cv_bridge
  image_transport
  pcl_conversions
  message_filters
  tf2_ros
  trackdlo_msgs
)
```

- [ ] **Step 3: ビルド確認**

Run: `cd ~/repos && colcon build --packages-select trackdlo_msgs trackdlo_core --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5`
Expected: ビルド成功（trackdlo_node.cpp の変更はまだなので既存コードのまま通る）

- [ ] **Step 4: コミット**

```bash
git add trackdlo_core/CMakeLists.txt trackdlo_core/package.xml
git commit -m "build(core): add librealsense2, std_srvs, trackdlo_msgs dependencies"
```

---

### Task 3: trackdlo_node.cpp — librealsense2 カメラ統合

**Files:**
- Modify: `trackdlo_core/src/trackdlo_node.cpp`

この Task では、librealsense2 でカメラフレームを直接取得し、カメラ情報もSDKから取得するように書き換える。message_filters による RGB+Depth 同期を廃止する。

- [ ] **Step 1: include とメンバ変数追加**

`trackdlo_node.cpp` 冒頭の include に追加:

```cpp
#include <librealsense2/rs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trackdlo_msgs/msg/tracking_status.hpp>
#include <thread>
#include <atomic>
```

不要になる include を削除:

```cpp
// 削除: #include <message_filters/subscriber.h>
// 削除: #include <message_filters/synchronizer.h>
// 削除: #include <message_filters/sync_policies/approximate_time.h>
```

- [ ] **Step 2: カメラパラメータ宣言追加**

コンストラクタのパラメータ宣言セクションに追加:

```cpp
this->declare_parameter<int>("camera_width", 424);
this->declare_parameter<int>("camera_height", 240);
this->declare_parameter<int>("camera_fps", 30);
this->declare_parameter<std::string>("camera_serial_no", "");
this->declare_parameter<std::string>("segmentation_mode", "internal");
```

パラメータ取得:

```cpp
camera_width_ = this->get_parameter("camera_width").as_int();
camera_height_ = this->get_parameter("camera_height").as_int();
camera_fps_ = this->get_parameter("camera_fps").as_int();
camera_serial_no_ = this->get_parameter("camera_serial_no").as_string();
segmentation_mode_ = this->get_parameter("segmentation_mode").as_string();
```

- [ ] **Step 3: メンバ変数の書き換え**

message_filters 関連のメンバを削除し、librealsense2 + 制御用メンバに置き換える:

```cpp
// 削除:
// using ApproxSyncPolicy = ...;
// std::shared_ptr<message_filters::Subscriber<...>> image_sub_;
// std::shared_ptr<message_filters::Subscriber<...>> depth_sub_;
// std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;
// rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
// std::string camera_info_topic_;

// 追加:
rs2::pipeline rs_pipe_;
rs2::align rs_align_{RS2_STREAM_COLOR};
std::thread camera_thread_;
std::atomic<bool> running_{false};

// カメラパラメータ
int camera_width_;
int camera_height_;
int camera_fps_;
std::string camera_serial_no_;
std::string segmentation_mode_;

// 状態管理
enum class State { IDLE, INITIALIZING, TRACKING, ERROR };
std::atomic<State> state_{State::IDLE};
std::string status_message_{"Idle"};
std::mutex status_mutex_;
int tracked_nodes_{0};
double current_fps_{0.0};

// サービス
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

// ステータスパブリッシャー
rclcpp::Publisher<trackdlo_msgs::msg::TrackingStatus>::SharedPtr status_pub_;
rclcpp::TimerBase::SharedPtr status_timer_;

// RGB配信用（composite_view, init_tracker向け）
image_transport::Publisher rgb_pub_;
image_transport::Publisher depth_pub_;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
```

- [ ] **Step 4: カメラ起動/停止メソッド実装**

```cpp
bool start_camera()
{
  try {
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, camera_width_, camera_height_,
                      RS2_FORMAT_BGR8, camera_fps_);
    cfg.enable_stream(RS2_STREAM_DEPTH, camera_width_, camera_height_,
                      RS2_FORMAT_Z16, camera_fps_);
    if (!camera_serial_no_.empty()) {
      cfg.enable_device(camera_serial_no_);
    }
    rs_pipe_.start(cfg);

    // カメラ情報取得 (intrinsics → proj_matrix_)
    auto profile = rs_pipe_.get_active_profile();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR)
                          .as<rs2::video_stream_profile>();
    auto intr = color_stream.get_intrinsics();
    proj_matrix_.setZero();
    proj_matrix_(0, 0) = intr.fx;
    proj_matrix_(0, 2) = intr.ppx;
    proj_matrix_(1, 1) = intr.fy;
    proj_matrix_(1, 2) = intr.ppy;
    proj_matrix_(2, 2) = 1.0;
    received_proj_matrix_ = true;

    // CameraInfo メッセージ作成して配信（init_tracker用）
    auto cam_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    cam_info->header.frame_id = result_frame_id_;
    cam_info->width = intr.width;
    cam_info->height = intr.height;
    cam_info->p = {intr.fx, 0.0, intr.ppx, 0.0,
                   0.0, intr.fy, intr.ppy, 0.0,
                   0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(*cam_info);

    RCLCPP_INFO(this->get_logger(), "RealSense started: %dx%d@%dfps",
                camera_width_, camera_height_, camera_fps_);
    return true;
  } catch (const rs2::error & e) {
    RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
    return false;
  }
}

void stop_camera()
{
  running_ = false;
  if (camera_thread_.joinable()) {
    camera_thread_.join();
  }
  try {
    rs_pipe_.stop();
  } catch (...) {}
  RCLCPP_INFO(this->get_logger(), "RealSense stopped.");
}
```

- [ ] **Step 5: カメラスレッド + HSVセグメンテーション内蔵**

```cpp
void camera_loop()
{
  while (running_) {
    try {
      auto frames = rs_pipe_.wait_for_frames(5000);
      auto aligned = rs_align_.process(frames);
      auto color_frame = aligned.get_color_frame();
      auto depth_frame = aligned.get_depth_frame();
      if (!color_frame || !depth_frame) continue;

      // rs2::frame → cv::Mat (ゼロコピーではないが最小限のコピー)
      cv::Mat color(cv::Size(camera_width_, camera_height_), CV_8UC3,
                    (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
      cv::Mat depth(cv::Size(camera_width_, camera_height_), CV_16UC1,
                    (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

      // RGB + Depth を配信（composite_view, init_tracker 向け）
      auto now = this->get_clock()->now();
      std_msgs::msg::Header header;
      header.stamp = now;
      header.frame_id = result_frame_id_;

      rgb_pub_.publish(
        *cv_bridge::CvImage(header, "bgr8", color).toImageMsg());
      depth_pub_.publish(
        *cv_bridge::CvImage(header, "16UC1", depth).toImageMsg());

      // 内蔵HSVセグメンテーション
      if (segmentation_mode_ == "internal") {
        cv::Mat hsv, mask;
        cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv,
                    cv::Scalar(hsv_lower_[0], hsv_lower_[1], hsv_lower_[2]),
                    cv::Scalar(hsv_upper_[0], hsv_upper_[1], hsv_upper_[2]),
                    mask);
        pipeline_manager_->set_external_mask(mask);
      }
      // external モードの場合は既存の external_mask_sub_ 経由で受信済み

      // 追跡処理を呼ぶ
      process_frame(color, depth);

    } catch (const rs2::error & e) {
      RCLCPP_WARN(this->get_logger(), "RealSense frame error: %s", e.what());
    }
  }
}
```

- [ ] **Step 6: process_frame メソッド（既存 Callback から抽出）**

既存の `Callback` メソッドの処理を `process_frame` に移す。`sensor_msgs::msg::Image` ではなく `cv::Mat` を直接受け取る:

```cpp
void process_frame(const cv::Mat & cur_image_orig, const cv::Mat & cur_depth)
{
  sensor_msgs::msg::Image::SharedPtr tracking_img_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cur_image_orig).toImageMsg();

  if (!pipeline_manager_->is_initialized()) {
    if (received_init_nodes_ && received_proj_matrix_) {
      std::vector<double> converted_node_coord;
      double cur_sum = 0;
      converted_node_coord.push_back(0.0);
      for (int i = 0; i < init_nodes_.rows() - 1; i++) {
        cur_sum += (init_nodes_.row(i + 1) - init_nodes_.row(i)).norm();
        converted_node_coord.push_back(cur_sum);
      }
      pipeline_manager_->initialize_tracker(init_nodes_, converted_node_coord);
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        state_ = State::TRACKING;
        tracked_nodes_ = init_nodes_.rows();
        status_message_ = "DLO initialized (" +
          std::to_string(init_nodes_.rows()) + " nodes)";
      }
    }
  } else {
    auto cur_time_cb = std::chrono::high_resolution_clock::now();

    trackdlo_core::PipelineResult result = pipeline_manager_->process(
      cur_image_orig, cur_depth, proj_matrix_);

    if (!result.success) {
      if (result.request_reinit && !reinit_requested_) {
        RCLCPP_WARN(this->get_logger(), "Requesting re-initialization.");
        reinit_requested_ = true;
        std::lock_guard<std::mutex> lock(status_mutex_);
        state_ = State::ERROR;
        status_message_ = "Tracking lost, requesting re-initialization";
      }
      if (!result.tracking_img.empty()) {
        tracking_img_msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result.tracking_img).toImageMsg();
      }
      tracking_img_pub_.publish(tracking_img_msg);
      return;
    }

    // マスク・オーバーレイの配信
    if (!result.mask.empty() && !result.cur_image.empty()) {
      seg_mask_pub_.publish(
        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", result.mask).toImageMsg());

      cv::Mat seg_overlay;
      cur_image_orig.copyTo(seg_overlay);
      cv::Mat color_layer(seg_overlay.size(), CV_8UC3, cv::Scalar(0, 255, 0));
      color_layer.copyTo(seg_overlay, result.mask);
      cv::addWeighted(cur_image_orig, 0.6, seg_overlay, 0.4, 0, seg_overlay);
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(result.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      cv::drawContours(seg_overlay, contours, -1, cv::Scalar(0, 255, 0), 2);
      seg_overlay_pub_.publish(
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", seg_overlay).toImageMsg());
    }

    double time_diff = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - cur_time_cb).count() / 1000.0;
    algo_total_ += time_diff;
    frames_ += 1;

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_fps_ = 1000.0 / (algo_total_ / frames_);
      tracked_nodes_ = result.Y.rows();
    }

    tracking_img_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result.tracking_img).toImageMsg();

    // PointCloud + Marker 配信（既存コードと同一）
    visualization_msgs::msg::MarkerArray results_msg = MatrixXd2MarkerArray(
      result.Y, result_frame_id_, "node_results",
      {1.0, 150.0 / 255.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}, 0.01, 0.005,
      result.not_self_occluded_nodes, {1.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 1.0});

    visualization_msgs::msg::MarkerArray guide_nodes_results = MatrixXd2MarkerArray(
      result.guide_nodes, result_frame_id_, "guide_node_results",
      {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 1.0, 0.5});

    visualization_msgs::msg::MarkerArray corr_priors_results = MatrixXd2MarkerArray(
      result.priors, result_frame_id_, "corr_prior_results",
      {0.0, 0.0, 0.0, 0.5}, {1.0, 0.0, 0.0, 0.5});

    pcl::PCLPointCloud2 cur_pc_pointcloud2, result_pc_pointcloud2, self_occluded_pc_pointcloud2;
    pcl::toPCLPointCloud2(result.cur_pc_downsampled, cur_pc_pointcloud2);
    pcl::toPCLPointCloud2(result.trackdlo_pc, result_pc_pointcloud2);
    pcl::toPCLPointCloud2(result.self_occluded_pc, self_occluded_pc_pointcloud2);

    sensor_msgs::msg::PointCloud2 cur_pc_msg, result_pc_msg, self_occluded_pc_msg;
    pcl_conversions::moveFromPCL(cur_pc_pointcloud2, cur_pc_msg);
    pcl_conversions::moveFromPCL(result_pc_pointcloud2, result_pc_msg);
    pcl_conversions::moveFromPCL(self_occluded_pc_pointcloud2, self_occluded_pc_msg);

    auto stamp = this->get_clock()->now();
    cur_pc_msg.header.frame_id = result_frame_id_;
    result_pc_msg.header.frame_id = result_frame_id_;
    result_pc_msg.header.stamp = stamp;
    self_occluded_pc_msg.header.frame_id = result_frame_id_;
    self_occluded_pc_msg.header.stamp = stamp;

    results_pub_->publish(results_msg);
    guide_nodes_pub_->publish(guide_nodes_results);
    corr_priors_pub_->publish(corr_priors_results);
    pc_pub_->publish(cur_pc_msg);
    result_pc_pub_->publish(result_pc_msg);
    self_occluded_pc_pub_->publish(self_occluded_pc_msg);
  }

  tracking_img_pub_.publish(tracking_img_msg);
}
```

- [ ] **Step 7: ビルド確認**

Run: `cd ~/repos && colcon build --packages-select trackdlo_msgs trackdlo_core --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -10`
Expected: ビルド成功

- [ ] **Step 8: コミット**

```bash
git add trackdlo_core/src/trackdlo_node.cpp
git commit -m "feat(core): integrate librealsense2 direct capture and internal HSV segmentation"
```

---

### Task 4: trackdlo_node.cpp — サービス + ステータス配信

**Files:**
- Modify: `trackdlo_core/src/trackdlo_node.cpp`

- [ ] **Step 1: init() メソッドにサービスとステータスを追加**

`init()` メソッドを書き換え。message_filters のサブスクライバ生成を削除し、以下を追加:

```cpp
void init()
{
  image_transport::ImageTransport it(shared_from_this());

  // RGB + Depth 配信（init_tracker, composite_view 向け）
  rgb_pub_ = it.advertise("/camera/color/image_raw", 1);
  depth_pub_ = it.advertise("/camera/aligned_depth_to_color/image_raw", 1);
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/aligned_depth_to_color/camera_info", 1);

  // 既存パブリッシャー
  int pub_queue_size = 1;
  tracking_img_pub_ = it.advertise("/trackdlo/results_img", pub_queue_size);
  seg_mask_pub_ = it.advertise("/trackdlo/segmentation_mask_img", pub_queue_size);
  seg_overlay_pub_ = it.advertise("/trackdlo/segmentation_overlay", pub_queue_size);

  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/trackdlo/filtered_pointcloud", pub_queue_size);
  results_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/trackdlo/results_marker", pub_queue_size);
  guide_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/trackdlo/guide_nodes", pub_queue_size);
  corr_priors_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/trackdlo/corr_priors", pub_queue_size);
  result_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/trackdlo/results_pc", pub_queue_size);
  self_occluded_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/trackdlo/self_occluded_pc", pub_queue_size);

  // 外部マスク（external モード用）
  if (segmentation_mode_ == "external") {
    external_mask_sub_ = it.subscribe(
      "/trackdlo/segmentation_mask", 1,
      std::bind(&TrackDLONode::update_external_mask, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
      "External segmentation mode. Subscribing to /trackdlo/segmentation_mask");
  }

  // オクルージョンマスク
  opencv_mask_sub_ = it.subscribe(
    "/mask_with_occlusion", 1,
    std::bind(&TrackDLONode::update_opencv_mask, this, std::placeholders::_1));

  // init_nodes 購読
  init_nodes_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/trackdlo/init_nodes", 1,
    std::bind(&TrackDLONode::update_init_nodes, this, std::placeholders::_1));

  // --- サービス ---
  start_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/trackdlo/start",
    std::bind(&TrackDLONode::on_start, this,
              std::placeholders::_1, std::placeholders::_2));

  stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/trackdlo/stop",
    std::bind(&TrackDLONode::on_stop, this,
              std::placeholders::_1, std::placeholders::_2));

  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/trackdlo/reset",
    std::bind(&TrackDLONode::on_reset, this,
              std::placeholders::_1, std::placeholders::_2));

  // --- ステータス配信 (1Hz) ---
  status_pub_ = this->create_publisher<trackdlo_msgs::msg::TrackingStatus>(
    "/trackdlo/status", 1);
  status_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TrackDLONode::publish_status, this));

  RCLCPP_INFO(this->get_logger(), "TrackDLO node initialized. State: IDLE. Call /trackdlo/start to begin.");
}
```

- [ ] **Step 2: サービスコールバック実装**

```cpp
void on_start(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (running_) {
    response->success = false;
    response->message = "Already running";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    state_ = State::INITIALIZING;
    status_message_ = "Starting camera...";
  }

  if (!start_camera()) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    state_ = State::ERROR;
    status_message_ = "Failed to start RealSense camera";
    response->success = false;
    response->message = status_message_;
    return;
  }

  running_ = true;
  camera_thread_ = std::thread(&TrackDLONode::camera_loop, this);

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_message_ = "Camera started, waiting for initialization...";
  }

  response->success = true;
  response->message = "Started successfully";
}

void on_stop(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (!running_) {
    response->success = false;
    response->message = "Not running";
    return;
  }

  stop_camera();

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    state_ = State::IDLE;
    status_message_ = "Stopped";
    current_fps_ = 0.0;
  }

  response->success = true;
  response->message = "Stopped successfully";
}

void on_reset(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  // 追跡状態をリセット
  received_init_nodes_ = false;
  reinit_requested_ = true;
  algo_total_ = 0.0;
  pub_data_total_ = 0.0;
  frames_ = 0;

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    state_ = State::INITIALIZING;
    status_message_ = "Reset, waiting for re-initialization...";
  }

  // カメラが動いていなければ起動
  if (!running_) {
    if (!start_camera()) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      state_ = State::ERROR;
      status_message_ = "Failed to start camera after reset";
      response->success = false;
      response->message = status_message_;
      return;
    }
    running_ = true;
    camera_thread_ = std::thread(&TrackDLONode::camera_loop, this);
  }

  response->success = true;
  response->message = "Reset successfully, re-initializing...";
}
```

- [ ] **Step 3: ステータス配信メソッド**

```cpp
void publish_status()
{
  trackdlo_msgs::msg::TrackingStatus msg;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    switch (state_) {
      case State::IDLE: msg.state = trackdlo_msgs::msg::TrackingStatus::IDLE; break;
      case State::INITIALIZING: msg.state = trackdlo_msgs::msg::TrackingStatus::INITIALIZING; break;
      case State::TRACKING: msg.state = trackdlo_msgs::msg::TrackingStatus::TRACKING; break;
      case State::ERROR: msg.state = trackdlo_msgs::msg::TrackingStatus::ERROR; break;
    }
    msg.message = status_message_;
    msg.tracked_nodes = tracked_nodes_;
    msg.fps = current_fps_;
  }
  status_pub_->publish(msg);
}
```

- [ ] **Step 4: デストラクタでカメラ停止**

```cpp
~TrackDLONode()
{
  stop_camera();
}
```

- [ ] **Step 5: ビルド確認**

Run: `cd ~/repos && colcon build --packages-select trackdlo_msgs trackdlo_core --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -10`
Expected: ビルド成功

- [ ] **Step 6: コミット**

```bash
git add trackdlo_core/src/trackdlo_node.cpp
git commit -m "feat(core): add start/stop/reset services and status topic"
```

---

### Task 5: realsense_params.yaml + launch ファイル更新

**Files:**
- Modify: `trackdlo_bringup/config/realsense_params.yaml`
- Modify: `trackdlo_bringup/launch/trackdlo.launch.py`

- [ ] **Step 1: realsense_params.yaml にカメラパラメータ追加**

`trackdlo:` セクションに以下を追加:

```yaml
    # Camera (librealsense2 direct)
    camera_width: 424
    camera_height: 240
    camera_fps: 30
    camera_serial_no: ""

    # Segmentation mode: "internal" (built-in HSV) or "external" (ROS topic)
    segmentation_mode: "internal"
```

- [ ] **Step 2: launch ファイル書き換え**

`trackdlo.launch.py` から以下を削除:
- `realsense2_camera` のインクルードlaunch（`realsense_launch`）
- `hsv_segmentation` ノード起動
- `segmentation` launch引数（もう不要、パラメータで制御）

`_launch_setup` を以下に書き換え:

```python
def _launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'realsense_params.yaml')

    rviz = LaunchConfiguration('rviz')

    return [
        # --- Static TF: base_link -> camera_link ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # --- TrackDLO C++ tracking node (librealsense2 integrated) ---
        Node(
            package='trackdlo_core',
            executable='trackdlo',
            name='trackdlo',
            output='screen',
            parameters=[params_file],
        ),

        # --- Python initialization node ---
        Node(
            package='trackdlo_core',
            executable='init_tracker',
            name='init_tracker',
            output='screen',
            parameters=[params_file],
        ),

        # --- Composite View (4-panel display) ---
        Node(
            package='trackdlo_utils',
            executable='composite_view',
            name='composite_view',
            output='screen',
            parameters=[params_file],
        ),

        # --- CPD-LLE Parameter Tuner + Control Panel ---
        Node(
            package='trackdlo_utils',
            executable='param_tuner',
            name='param_tuner',
            output='screen',
        ),

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.join(bringup_dir, 'rviz', 'tracking.rviz')
            ],
            condition=IfCondition(rviz),
        ),
    ]
```

`generate_launch_description` から `segmentation` 引数を削除:

```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 for visualization',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
```

不要な import を削除: `LaunchConfigurationEquals`, `PythonExpression`

- [ ] **Step 3: ビルド確認**

Run: `cd ~/repos && colcon build --packages-select trackdlo_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5`
Expected: ビルド成功

- [ ] **Step 4: コミット**

```bash
git add trackdlo_bringup/config/realsense_params.yaml trackdlo_bringup/launch/trackdlo.launch.py
git commit -m "feat(bringup): remove realsense2_camera dependency, add direct camera params"
```

---

### Task 6: param_tuner GUI拡張 — Start/Stop/Reset + ステータス

**Files:**
- Modify: `trackdlo_utils/trackdlo_utils/param_tuner_node.py`
- Modify: `trackdlo_utils/package.xml`

- [ ] **Step 1: package.xml に依存追加**

`trackdlo_utils/package.xml` に追加:

```xml
<depend>std_srvs</depend>
<depend>trackdlo_msgs</depend>
```

- [ ] **Step 2: import 追加**

`param_tuner_node.py` 冒頭に追加:

```python
from std_srvs.srv import Trigger
from trackdlo_msgs.msg import TrackingStatus
```

- [ ] **Step 3: ParamTunerNode にサービスクライアントとステータス購読を追加**

`__init__` に以下を追加:

```python
# Service clients
self.start_cli = self.create_client(Trigger, '/trackdlo/start')
self.stop_cli = self.create_client(Trigger, '/trackdlo/stop')
self.reset_cli = self.create_client(Trigger, '/trackdlo/reset')

# Status subscription
self.tracking_state = TrackingStatus.IDLE
self.tracking_message = 'Idle'
self.tracking_fps = 0.0
self.tracking_nodes = 0
self.create_subscription(
    TrackingStatus, '/trackdlo/status', self._on_status, 1)

# Button regions (will be set in _draw_info)
self.buttons = {}
cv2.setMouseCallback(WINDOW_NAME, self._on_mouse)
```

- [ ] **Step 4: ステータスコールバック**

```python
def _on_status(self, msg):
    self.tracking_state = msg.state
    self.tracking_message = msg.message
    self.tracking_fps = msg.fps
    self.tracking_nodes = msg.tracked_nodes
```

- [ ] **Step 5: マウスクリックハンドラ**

```python
def _on_mouse(self, event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    for name, (bx, by, bw, bh) in self.buttons.items():
        if bx <= x <= bx + bw and by <= y <= by + bh:
            if name == 'start':
                self.start_cli.call_async(Trigger.Request())
                self.get_logger().info('Start requested')
            elif name == 'stop':
                self.stop_cli.call_async(Trigger.Request())
                self.get_logger().info('Stop requested')
            elif name == 'reset':
                self.reset_cli.call_async(Trigger.Request())
                self.get_logger().info('Reset requested')
```

- [ ] **Step 6: _draw_info を拡張してボタンとステータスを描画**

`_draw_info` メソッドを以下のように書き換え:

```python
def _draw_info(self):
    """Draw info image with control buttons, status, and parameter values."""
    row_h = 32
    button_area_h = 80
    status_area_h = 80
    param_area_h = len(PARAM_DEFS) * row_h + 16
    w = 720
    h = button_area_h + status_area_h + param_area_h

    pil_img = PILImage.new('RGB', (w, h), (40, 40, 40))
    draw = ImageDraw.Draw(pil_img)

    try:
        font_btn = ImageFont.truetype(
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 18)
        font_val = ImageFont.truetype(
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 16)
        font_desc = ImageFont.truetype(
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 14)
    except (IOError, OSError):
        font_btn = ImageFont.load_default()
        font_val = font_btn
        font_desc = font_btn

    # --- Buttons ---
    btn_y = 15
    btn_h = 40
    btn_w = 100
    self.buttons = {}

    buttons_def = [
        ('start', 20, (0, 160, 0), '> Start'),
        ('stop', 140, (160, 0, 0), 'Stop'),
        ('reset', 260, (0, 80, 160), 'Reset'),
    ]
    for name, bx, color, label in buttons_def:
        draw.rounded_rectangle(
            [bx, btn_y, bx + btn_w, btn_y + btn_h],
            radius=5, fill=color)
        draw.text((bx + 15, btn_y + 8), label, fill=(255, 255, 255), font=font_btn)
        self.buttons[name] = (bx, btn_y, btn_w, btn_h)

    # --- Status ---
    status_y = button_area_h
    state_names = {
        TrackingStatus.IDLE: ('IDLE', (128, 128, 128)),
        TrackingStatus.INITIALIZING: ('INITIALIZING', (220, 200, 0)),
        TrackingStatus.TRACKING: ('TRACKING', (0, 220, 0)),
        TrackingStatus.ERROR: ('ERROR', (220, 0, 0)),
    }
    state_name, state_color = state_names.get(
        self.tracking_state, ('UNKNOWN', (128, 128, 128)))

    draw.ellipse([20, status_y + 8, 36, status_y + 24], fill=state_color)
    draw.text((45, status_y + 5),
              f'State: {state_name}', fill=(220, 220, 220), font=font_val)
    draw.text((20, status_y + 30),
              f'{self.tracking_message}', fill=(180, 180, 180), font=font_desc)
    draw.text((20, status_y + 52),
              f'FPS: {self.tracking_fps:.1f}  Nodes: {self.tracking_nodes}',
              fill=(180, 180, 180), font=font_desc)

    # --- Parameters ---
    y = button_area_h + status_area_h
    draw.line([(10, y), (w - 10, y)], fill=(80, 80, 80), width=1)
    y += 8
    for name, ptype, slider_max, scale_div, offset, desc in PARAM_DEFS:
        sv = cv2.getTrackbarPos(name, WINDOW_NAME)
        real_val = slider_to_real(sv, ptype, scale_div, offset)
        if ptype == 'int':
            val_text = f'{name}: {int(real_val)}'
        else:
            val_text = f'{name}: {real_val:.4f}'
        draw.text((10, y), val_text, fill=(220, 220, 220), font=font_val)
        draw.text((320, y + 1), desc, fill=(140, 180, 140), font=font_desc)
        y += row_h

    return np.array(pil_img)
```

- [ ] **Step 7: ビルド確認**

Run: `cd ~/repos && colcon build --packages-select trackdlo_utils --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5`
Expected: ビルド成功

- [ ] **Step 8: コミット**

```bash
git add trackdlo_utils/trackdlo_utils/param_tuner_node.py trackdlo_utils/package.xml
git commit -m "feat(utils): add Start/Stop/Reset buttons and status display to param_tuner"
```

---

### Task 7: 統合テスト

**Files:**
- (変更なし — 動作確認のみ)

- [ ] **Step 1: フルビルド**

Run:
```bash
cd ~/repos && source /opt/ros/${ROS_DISTRO}/setup.bash && \
colcon build --packages-select trackdlo_msgs trackdlo_core trackdlo_utils trackdlo_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Expected: 全パッケージビルド成功

- [ ] **Step 2: 既存テスト実行**

Run:
```bash
cd ~/repos && colcon test --packages-select trackdlo_core trackdlo_utils trackdlo_bringup \
  --return-code-on-test-failure
```
Expected: テスト通過（リンクエラーなし）

- [ ] **Step 3: ノード起動確認**

Run:
```bash
source install/setup.bash && ros2 launch trackdlo_bringup trackdlo.launch.py rviz:=false
```
Expected:
- ノードが IDLE 状態で起動
- `/trackdlo/status` トピックが配信される
- param_tuner に Start/Stop/Reset ボタンが表示される

- [ ] **Step 4: サービス呼び出しテスト**

別ターミナルから:
```bash
ros2 service call /trackdlo/start std_srvs/srv/Trigger
```
Expected:
- RealSense カメラが起動
- `/camera/color/image_raw` にRGB画像が配信される
- セグメンテーションマスクが生成される
- 初期化完了後、TRACKING 状態に遷移
- param_tuner のステータスが緑色 TRACKING に変化

- [ ] **Step 5: Stop/Reset テスト**

```bash
ros2 service call /trackdlo/stop std_srvs/srv/Trigger
```
Expected: カメラ停止、IDLE に遷移

```bash
ros2 service call /trackdlo/reset std_srvs/srv/Trigger
```
Expected: カメラ再起動、INITIALIZING → TRACKING に遷移

- [ ] **Step 6: コミット（全変更まとめ）**

最終的な微調整があればコミット:
```bash
git add -A && git commit -m "feat: integrate librealsense2 SDK with external control services"
```
