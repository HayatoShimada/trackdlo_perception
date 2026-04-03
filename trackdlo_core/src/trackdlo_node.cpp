// Copyright 2026 Hayato Shimada
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include <librealsense2/rs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trackdlo_msgs/msg/tracking_status.hpp>
#include <thread>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "trackdlo_core/trackdlo.hpp"
#include "trackdlo_core/utils.hpp"
#include "trackdlo_core/pipeline_manager.hpp"

using cv::Mat;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;

class TrackDLONode : public rclcpp::Node
{
public:
  TrackDLONode()
  : Node("tracker_node"),
    received_init_nodes_(false),
    received_proj_matrix_(false),
    proj_matrix_(3, 4),
    algo_total_(0.0),
    pub_data_total_(0.0),
    frames_(0)
  {
    // --- Tracking algorithm (CPD-LLE) parameters ---
    this->declare_parameter<double>("beta", 0.35);
    this->declare_parameter<double>("lambda", 50000.0);
    this->declare_parameter<double>("alpha", 3.0);
    this->declare_parameter<double>("mu", 0.1);
    this->declare_parameter<int>("max_iter", 50);
    this->declare_parameter<double>("tol", 0.0002);

    // --- Visibility / occlusion parameters ---
    this->declare_parameter<double>("k_vis", 50.0);
    this->declare_parameter<double>("d_vis", 0.06);
    this->declare_parameter<double>("visibility_threshold", 0.008);

    // --- Image drawing / preprocessing parameters ---
    this->declare_parameter<int>("dlo_pixel_width", 40);
    this->declare_parameter<double>("beta_pre_proc", 3.0);
    this->declare_parameter<double>("lambda_pre_proc", 1.0);
    this->declare_parameter<double>("lle_weight", 10.0);

    // --- Other settings ---
    this->declare_parameter<bool>("multi_color_dlo", false);
    this->declare_parameter<double>("downsample_leaf_size", 0.008);

    this->declare_parameter<std::string>("result_frame_id", "camera_color_optical_frame");
    this->declare_parameter<std::string>("hsv_threshold_upper_limit", "130 255 255");
    this->declare_parameter<std::string>("hsv_threshold_lower_limit", "90 90 30");
    this->declare_parameter<bool>("use_external_mask", false);

    // --- Camera parameters (librealsense2 direct capture) ---
    this->declare_parameter<int>("camera_width", 424);
    this->declare_parameter<int>("camera_height", 240);
    this->declare_parameter<int>("camera_fps", 30);
    this->declare_parameter<std::string>("camera_serial_no", "");
    this->declare_parameter<std::string>("segmentation_mode", "internal");

    beta_ = this->get_parameter("beta").as_double();
    lambda_ = this->get_parameter("lambda").as_double();
    alpha_ = this->get_parameter("alpha").as_double();
    mu_ = this->get_parameter("mu").as_double();
    max_iter_ = this->get_parameter("max_iter").as_int();
    tol_ = this->get_parameter("tol").as_double();
    k_vis_ = this->get_parameter("k_vis").as_double();
    d_vis_ = this->get_parameter("d_vis").as_double();
    visibility_threshold_ = this->get_parameter("visibility_threshold").as_double();
    dlo_pixel_width_ = this->get_parameter("dlo_pixel_width").as_int();
    beta_pre_proc_ = this->get_parameter("beta_pre_proc").as_double();
    lambda_pre_proc_ = this->get_parameter("lambda_pre_proc").as_double();
    lle_weight_ = this->get_parameter("lle_weight").as_double();
    multi_color_dlo_ = this->get_parameter("multi_color_dlo").as_bool();
    downsample_leaf_size_ = this->get_parameter("downsample_leaf_size").as_double();

    result_frame_id_ = this->get_parameter("result_frame_id").as_string();
    use_external_mask_ = this->get_parameter("use_external_mask").as_bool();

    camera_width_ = this->get_parameter("camera_width").as_int();
    camera_height_ = this->get_parameter("camera_height").as_int();
    camera_fps_ = this->get_parameter("camera_fps").as_int();
    camera_serial_no_ = this->get_parameter("camera_serial_no").as_string();
    segmentation_mode_ = this->get_parameter("segmentation_mode").as_string();

    // Parse HSV thresholds
    std::string hsv_threshold_upper_limit =
      this->get_parameter("hsv_threshold_upper_limit").as_string();
    std::string hsv_threshold_lower_limit =
      this->get_parameter("hsv_threshold_lower_limit").as_string();

    std::string rgb_val = "";
    for (size_t i = 0; i < hsv_threshold_upper_limit.length(); i++) {
      if (hsv_threshold_upper_limit.substr(i, 1) != " ") {
        rgb_val += hsv_threshold_upper_limit.substr(i, 1);
      } else {
        hsv_upper_.push_back(std::stoi(rgb_val));
        rgb_val = "";
      }
      if (i == hsv_threshold_upper_limit.length() - 1) {
        hsv_upper_.push_back(std::stoi(rgb_val));
      }
    }

    rgb_val = "";
    for (size_t i = 0; i < hsv_threshold_lower_limit.length(); i++) {
      if (hsv_threshold_lower_limit.substr(i, 1) != " ") {
        rgb_val += hsv_threshold_lower_limit.substr(i, 1);
      } else {
        hsv_lower_.push_back(std::stoi(rgb_val));
        rgb_val = "";
      }
      if (i == hsv_threshold_lower_limit.length() - 1) {
        hsv_lower_.push_back(std::stoi(rgb_val));
      }
    }

    // When segmentation_mode is "internal", we provide the mask externally
    // (from our own HSV computation), so use_external_mask should be true
    // for the pipeline manager.
    bool pipeline_use_external = use_external_mask_ || (segmentation_mode_ == "internal");

    pipeline_manager_ = std::make_unique<trackdlo_core::PipelineManager>(
      pipeline_use_external,
      multi_color_dlo_, hsv_lower_,
      hsv_upper_);
    update_pipeline_parameters();

    proj_matrix_.setZero();

    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() {
        this->init();
        this->init_timer_->cancel();
      }
    );

    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          const auto & name = p.get_name();
          if (name == "beta") {
            beta_ = p.as_double();
          } else if (name == "lambda") {
            lambda_ = p.as_double();
          } else if (name == "alpha") {
            alpha_ = p.as_double();
          } else if (name == "mu") {
            mu_ = p.as_double();
          } else if (name == "max_iter") {
            max_iter_ = p.as_int();
          } else if (name == "tol") {
            tol_ = p.as_double();
          } else if (name == "k_vis") {
            k_vis_ = p.as_double();
          } else if (name == "d_vis") {
            d_vis_ = p.as_double();
          } else if (name == "visibility_threshold") {
            visibility_threshold_ = p.as_double();
          } else if (name == "dlo_pixel_width") {
            dlo_pixel_width_ = p.as_int();
          } else if (name == "downsample_leaf_size") {
            downsample_leaf_size_ = p.as_double();
          } else if (name == "beta_pre_proc") {
            beta_pre_proc_ = p.as_double();
          } else if (name == "lambda_pre_proc") {
            lambda_pre_proc_ = p.as_double();
          } else if (name == "lle_weight") {
            lle_weight_ = p.as_double();
          } else {
            continue;
          }
        }
        update_pipeline_parameters();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
  }

  ~TrackDLONode()
  {
    stop_camera();
  }

  void init()
  {
    int pub_queue_size = 1;

    image_transport::ImageTransport it(shared_from_this());

    opencv_mask_sub_ = it.subscribe(
      "/mask_with_occlusion", 1,
      std::bind(&TrackDLONode::update_opencv_mask, this, std::placeholders::_1));

    if (use_external_mask_ && segmentation_mode_ != "internal") {
      external_mask_sub_ = it.subscribe(
        "/trackdlo/segmentation_mask", 1,
        std::bind(&TrackDLONode::update_external_mask, this, std::placeholders::_1));
      RCLCPP_INFO(
        this->get_logger(),
        "External mask mode enabled. Subscribing to /trackdlo/segmentation_mask");
    }

    init_nodes_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/trackdlo/init_nodes", 1,
      std::bind(&TrackDLONode::update_init_nodes, this, std::placeholders::_1));

    // RGB/Depth republish for init_tracker, composite_view, etc.
    rgb_pub_ = it.advertise("/camera/color/image_raw", pub_queue_size);
    depth_pub_ = it.advertise("/camera/aligned_depth_to_color/image_raw", pub_queue_size);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/aligned_depth_to_color/camera_info", pub_queue_size);

    // Result publishers
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

    // Services
    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/trackdlo/start",
      std::bind(&TrackDLONode::on_start, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/trackdlo/stop",
      std::bind(&TrackDLONode::on_stop, this, std::placeholders::_1, std::placeholders::_2));
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/trackdlo/reset",
      std::bind(&TrackDLONode::on_reset, this, std::placeholders::_1, std::placeholders::_2));

    // Status publisher at 1Hz
    status_pub_ = this->create_publisher<trackdlo_msgs::msg::TrackingStatus>(
      "/trackdlo/status", 1);
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TrackDLONode::publish_status, this));

    RCLCPP_INFO_STREAM(this->get_logger(), "TrackDLO node initialized. State: IDLE");
    RCLCPP_INFO_STREAM(this->get_logger(), "Call /trackdlo/start to begin camera capture.");
  }

private:
  // ---------- librealsense2 ----------
  rs2::pipeline rs_pipe_;
  rs2::align rs_align_{RS2_STREAM_COLOR};
  std::thread camera_thread_;
  std::atomic<bool> running_{false};

  // ---------- Camera params ----------
  int camera_width_;
  int camera_height_;
  int camera_fps_;
  std::string camera_serial_no_;
  std::string segmentation_mode_;

  // ---------- State management ----------
  enum class State { IDLE, INITIALIZING, TRACKING, ERROR };
  std::atomic<State> state_{State::IDLE};
  std::string status_message_{"Idle"};
  std::mutex status_mutex_;
  int tracked_nodes_{0};
  double current_fps_{0.0};

  // ---------- HSV thresholds ----------
  std::vector<int> hsv_upper_;
  std::vector<int> hsv_lower_;

  // ---------- Publishers ----------
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr results_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr guide_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corr_priors_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr self_occluded_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr result_pc_pub_;

  image_transport::Publisher tracking_img_pub_;
  image_transport::Publisher seg_mask_pub_;
  image_transport::Publisher seg_overlay_pub_;

  // RGB/Depth republish publishers
  image_transport::Publisher rgb_pub_;
  image_transport::Publisher depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // ---------- Services ----------
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  // ---------- Status publisher ----------
  rclcpp::Publisher<trackdlo_msgs::msg::TrackingStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // ---------- Subscribers ----------
  image_transport::Subscriber opencv_mask_sub_;
  image_transport::Subscriber external_mask_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr init_nodes_sub_;

  // ---------- Timer for deferred init ----------
  rclcpp::TimerBase::SharedPtr init_timer_;

  // ---------- Parameter callback ----------
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ---------- Pipeline Manager ----------
  std::unique_ptr<trackdlo_core::PipelineManager> pipeline_manager_;

  // ---------- Tracker state ----------
  bool received_init_nodes_;
  bool received_proj_matrix_;
  bool reinit_requested_{false};
  MatrixXd init_nodes_;
  MatrixXd proj_matrix_;

  // ---------- Parameters ----------
  bool multi_color_dlo_;
  double visibility_threshold_;
  int dlo_pixel_width_;
  double beta_;
  double beta_pre_proc_;
  double lambda_;
  double lambda_pre_proc_;
  double alpha_;
  double lle_weight_;
  double mu_;
  int max_iter_;
  double tol_;
  double k_vis_;
  double d_vis_;
  double downsample_leaf_size_;

  std::string result_frame_id_;
  bool use_external_mask_;

  // ---------- Timing ----------
  double algo_total_;
  double pub_data_total_;
  int frames_;

  void update_pipeline_parameters()
  {
    if (pipeline_manager_) {
      pipeline_manager_->set_parameters(
        visibility_threshold_, dlo_pixel_width_,
        downsample_leaf_size_, d_vis_, 30);
      pipeline_manager_->set_tracker_parameters(
        beta_, beta_pre_proc_, lambda_, lambda_pre_proc_,
        alpha_, lle_weight_, mu_, max_iter_, tol_, k_vis_);
    }
  }

  void update_opencv_mask(const sensor_msgs::msg::Image::ConstSharedPtr & opencv_mask_msg)
  {
    pipeline_manager_->set_occlusion_mask(cv_bridge::toCvShare(opencv_mask_msg, "bgr8")->image);
  }

  void update_external_mask(const sensor_msgs::msg::Image::ConstSharedPtr & mask_msg)
  {
    pipeline_manager_->set_external_mask(cv_bridge::toCvShare(mask_msg, "mono8")->image);
  }

  void update_init_nodes(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg)
  {
    pcl::PCLPointCloud2 * cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*pc_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_xyz;
    pcl::fromPCLPointCloud2(*cloud, cloud_xyz);
    delete cloud;

    init_nodes_ = cloud_xyz.getMatrixXfMap().topRows(3).transpose().cast<double>();

    if (!received_init_nodes_) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Received " << init_nodes_.rows() << " init nodes");
    }
    received_init_nodes_ = true;

    if (reinit_requested_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Re-initializing tracker with %d init nodes",
        static_cast<int>(init_nodes_.rows()));

      std::vector<double> converted_node_coord;
      double cur_sum = 0;
      converted_node_coord.push_back(0.0);
      for (int i = 0; i < init_nodes_.rows() - 1; i++) {
        cur_sum += (init_nodes_.row(i + 1) - init_nodes_.row(i)).norm();
        converted_node_coord.push_back(cur_sum);
      }
      pipeline_manager_->initialize_tracker(init_nodes_, converted_node_coord);
      reinit_requested_ = false;

      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        state_.store(State::TRACKING);
        status_message_ = "Re-initialized, tracking";
        tracked_nodes_ = static_cast<int>(init_nodes_.rows());
      }
    }
  }

  // ==================== Camera management ====================

  bool start_camera()
  {
    try {
      rs2::config cfg;
      if (!camera_serial_no_.empty()) {
        cfg.enable_device(camera_serial_no_);
      }
      cfg.enable_stream(
        RS2_STREAM_COLOR, camera_width_, camera_height_, RS2_FORMAT_BGR8, camera_fps_);
      cfg.enable_stream(
        RS2_STREAM_DEPTH, camera_width_, camera_height_, RS2_FORMAT_Z16, camera_fps_);

      auto profile = rs_pipe_.start(cfg);

      // Extract intrinsics from color stream and build projection matrix
      auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      auto intrinsics = color_stream.get_intrinsics();

      proj_matrix_.setZero();
      proj_matrix_(0, 0) = intrinsics.fx;
      proj_matrix_(0, 2) = intrinsics.ppx;
      proj_matrix_(1, 1) = intrinsics.fy;
      proj_matrix_(1, 2) = intrinsics.ppy;
      proj_matrix_(2, 2) = 1.0;
      received_proj_matrix_ = true;

      // Publish CameraInfo once
      sensor_msgs::msg::CameraInfo cam_info_msg;
      cam_info_msg.header.frame_id = result_frame_id_;
      cam_info_msg.header.stamp = this->now();
      cam_info_msg.width = static_cast<uint32_t>(intrinsics.width);
      cam_info_msg.height = static_cast<uint32_t>(intrinsics.height);
      cam_info_msg.distortion_model = "plumb_bob";
      cam_info_msg.d.resize(5, 0.0);
      for (int i = 0; i < 5; i++) {
        cam_info_msg.d[i] = intrinsics.coeffs[i];
      }
      // K matrix (3x3 intrinsic)
      cam_info_msg.k = {
        intrinsics.fx, 0.0, intrinsics.ppx,
        0.0, intrinsics.fy, intrinsics.ppy,
        0.0, 0.0, 1.0};
      // P matrix (3x4 projection)
      cam_info_msg.p = {
        intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
        0.0, intrinsics.fy, intrinsics.ppy, 0.0,
        0.0, 0.0, 1.0, 0.0};
      // R matrix (identity)
      cam_info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      camera_info_pub_->publish(cam_info_msg);

      RCLCPP_INFO(
        this->get_logger(),
        "Camera started: %dx%d @ %d fps (serial: %s)",
        camera_width_, camera_height_, camera_fps_,
        camera_serial_no_.empty() ? "auto" : camera_serial_no_.c_str());

      return true;
    } catch (const rs2::error & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to start camera: %s", e.what());
      return false;
    }
  }

  void stop_camera()
  {
    running_.store(false);
    if (camera_thread_.joinable()) {
      camera_thread_.join();
    }
    try {
      rs_pipe_.stop();
    } catch (...) {
      // Ignore errors when stopping (may not be running)
    }
  }

  void camera_loop()
  {
    auto last_fps_time = std::chrono::high_resolution_clock::now();
    int fps_frame_count = 0;

    while (running_.load()) {
      try {
        rs2::frameset frames = rs_pipe_.wait_for_frames(5000);
        rs2::frameset aligned = rs_align_.process(frames);

        auto color_frame = aligned.get_color_frame();
        auto depth_frame = aligned.get_depth_frame();

        if (!color_frame || !depth_frame) {
          continue;
        }

        // Convert to cv::Mat
        Mat color(
          cv::Size(color_frame.get_width(), color_frame.get_height()),
          CV_8UC3,
          const_cast<void *>(color_frame.get_data()),
          cv::Mat::AUTO_STEP);

        Mat depth(
          cv::Size(depth_frame.get_width(), depth_frame.get_height()),
          CV_16UC1,
          const_cast<void *>(depth_frame.get_data()),
          cv::Mat::AUTO_STEP);

        // Publish RGB and Depth for other nodes (init_tracker, composite_view)
        auto stamp = this->now();
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = result_frame_id_;

        rgb_pub_.publish(
          cv_bridge::CvImage(header, "bgr8", color).toImageMsg());
        depth_pub_.publish(
          cv_bridge::CvImage(header, "16UC1", depth).toImageMsg());

        // Internal HSV segmentation
        if (segmentation_mode_ == "internal") {
          Mat hsv;
          cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV);
          Mat mask;
          cv::inRange(
            hsv,
            cv::Scalar(hsv_lower_[0], hsv_lower_[1], hsv_lower_[2]),
            cv::Scalar(hsv_upper_[0], hsv_upper_[1], hsv_upper_[2]),
            mask);

          // Morphological cleanup
          int morph_size = 3;
          Mat element = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(2 * morph_size + 1, 2 * morph_size + 1));
          cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
          cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);

          pipeline_manager_->set_external_mask(mask);
        }

        // Process the frame
        process_frame(color, depth, stamp);

        // FPS calculation
        fps_frame_count++;
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_fps_time).count() / 1000.0;
        if (elapsed >= 1.0) {
          std::lock_guard<std::mutex> lock(status_mutex_);
          current_fps_ = fps_frame_count / elapsed;
          fps_frame_count = 0;
          last_fps_time = now;
        }

      } catch (const rs2::error & e) {
        RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
        {
          std::lock_guard<std::mutex> lock(status_mutex_);
          state_.store(State::ERROR);
          status_message_ = std::string("Camera error: ") + e.what();
        }
        break;
      }
    }
  }

  void process_frame(const Mat & cur_image_orig, const Mat & cur_depth,
    const rclcpp::Time & stamp)
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
          state_.store(State::TRACKING);
          status_message_ = "Tracking";
          tracked_nodes_ = static_cast<int>(init_nodes_.rows());
        }
      } else {
        // Waiting for init nodes
        State cur = state_.load();
        if (cur != State::INITIALIZING) {
          std::lock_guard<std::mutex> lock(status_mutex_);
          state_.store(State::INITIALIZING);
          status_message_ = "Waiting for init nodes";
        }
      }
    } else {
      std::chrono::high_resolution_clock::time_point cur_time_cb =
        std::chrono::high_resolution_clock::now();

      trackdlo_core::PipelineResult result = pipeline_manager_->process(
        cur_image_orig, cur_depth,
        proj_matrix_);

      if (!result.success) {
        if (result.request_reinit && !reinit_requested_) {
          RCLCPP_WARN(this->get_logger(), "Requesting re-initialization.");
          reinit_requested_ = true;
          {
            std::lock_guard<std::mutex> lock(status_mutex_);
            state_.store(State::INITIALIZING);
            status_message_ = "Tracking lost, re-initializing";
          }
        }
        if (!result.tracking_img.empty()) {
          tracking_img_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result.tracking_img).toImageMsg();
        }
        tracking_img_pub_.publish(tracking_img_msg);
        return;
      }

      // Publish masks and overlays
      if (!result.mask.empty() && !result.cur_image.empty()) {
        seg_mask_pub_.publish(
          cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", result.mask).toImageMsg());

        Mat seg_overlay;
        cur_image_orig.copyTo(seg_overlay);
        Mat color_layer(seg_overlay.size(), CV_8UC3, cv::Scalar(0, 255, 0));
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
      RCLCPP_INFO_STREAM(
        this->get_logger(), "Pipeline processing: " + std::to_string(
          time_diff) + " ms");
      algo_total_ += time_diff;

      std::chrono::high_resolution_clock::time_point cur_time_pub =
        std::chrono::high_resolution_clock::now();

      tracking_img_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result.tracking_img).toImageMsg();

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

      pcl::PCLPointCloud2 cur_pc_pointcloud2, result_pc_poincloud2, self_occluded_pc_poincloud2;
      pcl::toPCLPointCloud2(result.cur_pc_downsampled, cur_pc_pointcloud2);
      pcl::toPCLPointCloud2(result.trackdlo_pc, result_pc_poincloud2);
      pcl::toPCLPointCloud2(result.self_occluded_pc, self_occluded_pc_poincloud2);

      sensor_msgs::msg::PointCloud2 cur_pc_msg, result_pc_msg, self_occluded_pc_msg;
      pcl_conversions::moveFromPCL(cur_pc_pointcloud2, cur_pc_msg);
      pcl_conversions::moveFromPCL(result_pc_poincloud2, result_pc_msg);
      pcl_conversions::moveFromPCL(self_occluded_pc_poincloud2, self_occluded_pc_msg);

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

      for (size_t i = 0; i < guide_nodes_results.markers.size(); i++) {
        guide_nodes_results.markers[i].action = visualization_msgs::msg::Marker::DELETEALL;
      }
      for (size_t i = 0; i < corr_priors_results.markers.size(); i++) {
        corr_priors_results.markers[i].action = visualization_msgs::msg::Marker::DELETEALL;
      }

      time_diff = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - cur_time_pub).count() / 1000.0;
      RCLCPP_INFO_STREAM(this->get_logger(), "Pub data: " + std::to_string(time_diff) + " ms");
      pub_data_total_ += time_diff;

      frames_ += 1;
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Avg tracking step: " + std::to_string(algo_total_ / frames_) + " ms");
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Avg total: " + std::to_string((algo_total_ + pub_data_total_) / frames_) + " ms");

      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        tracked_nodes_ = static_cast<int>(result.Y.rows());
      }
    }

    tracking_img_pub_.publish(tracking_img_msg);
  }

  // ==================== Service callbacks ====================

  void on_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (running_.load()) {
      response->success = false;
      response->message = "Camera is already running";
      return;
    }

    if (!start_camera()) {
      response->success = false;
      response->message = "Failed to start camera";
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        state_.store(State::ERROR);
        status_message_ = "Camera start failed";
      }
      return;
    }

    running_.store(true);
    camera_thread_ = std::thread(&TrackDLONode::camera_loop, this);

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      state_.store(State::INITIALIZING);
      status_message_ = "Camera started, waiting for init nodes";
    }

    response->success = true;
    response->message = "Camera started";
    RCLCPP_INFO(this->get_logger(), "Camera started via service call");
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!running_.load()) {
      response->success = false;
      response->message = "Camera is not running";
      return;
    }

    stop_camera();

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      state_.store(State::IDLE);
      status_message_ = "Stopped";
      current_fps_ = 0.0;
    }

    response->success = true;
    response->message = "Camera stopped";
    RCLCPP_INFO(this->get_logger(), "Camera stopped via service call");
  }

  void on_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Stop camera if running
    bool was_running = running_.load();
    if (was_running) {
      stop_camera();
    }

    // Reset tracker state
    received_init_nodes_ = false;
    reinit_requested_ = false;
    algo_total_ = 0.0;
    pub_data_total_ = 0.0;
    frames_ = 0;

    // Recreate pipeline manager
    bool pipeline_use_external = use_external_mask_ || (segmentation_mode_ == "internal");
    pipeline_manager_ = std::make_unique<trackdlo_core::PipelineManager>(
      pipeline_use_external,
      multi_color_dlo_, hsv_lower_,
      hsv_upper_);
    update_pipeline_parameters();

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      state_.store(State::IDLE);
      status_message_ = "Reset complete";
      tracked_nodes_ = 0;
      current_fps_ = 0.0;
    }

    // Restart camera if it was running
    if (was_running) {
      if (start_camera()) {
        running_.store(true);
        camera_thread_ = std::thread(&TrackDLONode::camera_loop, this);
        {
          std::lock_guard<std::mutex> lock(status_mutex_);
          state_.store(State::INITIALIZING);
          status_message_ = "Reset complete, camera restarted";
        }
      }
    }

    response->success = true;
    response->message = "Tracker reset";
    RCLCPP_INFO(this->get_logger(), "Tracker reset via service call");
  }

  // ==================== Status publishing ====================

  void publish_status()
  {
    trackdlo_msgs::msg::TrackingStatus msg;

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      State s = state_.load();
      switch (s) {
        case State::IDLE:
          msg.state = trackdlo_msgs::msg::TrackingStatus::IDLE;
          break;
        case State::INITIALIZING:
          msg.state = trackdlo_msgs::msg::TrackingStatus::INITIALIZING;
          break;
        case State::TRACKING:
          msg.state = trackdlo_msgs::msg::TrackingStatus::TRACKING;
          break;
        case State::ERROR:
          msg.state = trackdlo_msgs::msg::TrackingStatus::ERROR;
          break;
      }
      msg.message = status_message_;
      msg.tracked_nodes = tracked_nodes_;
      msg.fps = current_fps_;
    }

    status_pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackDLONode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
