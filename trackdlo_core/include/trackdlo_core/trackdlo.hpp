// Copyright 2026 Hayato Shimada
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd
#pragma once

#ifndef TRACKDLO_CORE__TRACKDLO_HPP_
#define TRACKDLO_CORE__TRACKDLO_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <signal.h>
#include <unistd.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>

using Eigen::MatrixXd;
using cv::Mat;

class trackdlo
{
public:
  trackdlo();
  explicit trackdlo(int num_of_nodes);
  trackdlo(
    int num_of_nodes,
    double visibility_threshold,
    double beta,
    double lambda,
    double alpha,
    double k_vis,
    double mu,
    int max_iter,
    double tol,
    double beta_pre_proc,
    double lambda_pre_proc,
    double lle_weight);

  double get_sigma2();
  MatrixXd get_tracking_result();
  MatrixXd get_guide_nodes();
  std::vector<MatrixXd> get_correspondence_pairs();
  void initialize_geodesic_coord(std::vector<double> geodesic_coord);
  void initialize_nodes(MatrixXd Y_init);
  void set_sigma2(double sigma2);
  void set_beta(double v) {beta_ = v;}
  void set_lambda(double v) {lambda_ = v;}
  void set_alpha(double v) {alpha_ = v;}
  void set_mu(double v) {mu_ = v;}
  void set_max_iter(int v) {max_iter_ = v;}
  void set_tol(double v) {tol_ = v;}
  void set_k_vis(double v) {k_vis_ = v;}
  void set_beta_pre_proc(double v) {beta_pre_proc_ = v;}
  void set_lambda_pre_proc(double v) {lambda_pre_proc_ = v;}
  void set_lle_weight(double v) {lle_weight_ = v;}
  void set_visibility_threshold(double v) {visibility_threshold_ = v;}

  bool cpd_lle(
    MatrixXd X_orig,
    MatrixXd & Y,
    double & sigma2,
    double beta,
    double lambda,
    double lle_weight,
    double mu,
    int max_iter = 30,
    double tol = 0.0001,
    bool include_lle = true,
    std::vector<MatrixXd> correspondence_priors = {},
    double alpha = 0,
    std::vector<int> visible_nodes = {},
    double k_vis = 0,
    double visibility_threshold = 0.01);

  void tracking_step(
    MatrixXd X_orig,
    std::vector<int> visible_nodes,
    std::vector<int> visible_nodes_extended,
    MatrixXd proj_matrix,
    int img_rows,
    int img_cols);

private:
  MatrixXd Y_;
  MatrixXd guide_nodes_;
  double sigma2_;
  double beta_;
  double beta_pre_proc_;
  double lambda_;
  double lambda_pre_proc_;
  double alpha_;
  double k_vis_;
  double mu_;
  int max_iter_;
  double tol_;
  double lle_weight_;

  std::vector<double> geodesic_coord_;
  std::vector<MatrixXd> correspondence_priors_;
  double visibility_threshold_;

  std::vector<int> get_nearest_indices(int k, int M, int idx);
  MatrixXd calc_LLE_weights(int k, MatrixXd X);
  std::vector<MatrixXd> traverse_geodesic(
    std::vector<double> geodesic_coord, const MatrixXd guide_nodes,
    const std::vector<int> visible_nodes, int alignment);
  std::vector<MatrixXd> traverse_euclidean(
    std::vector<double> geodesic_coord, const MatrixXd guide_nodes,
    const std::vector<int> visible_nodes, int alignment, int alignment_node_idx = -1);
};

#endif  // TRACKDLO_CORE__TRACKDLO_HPP_
