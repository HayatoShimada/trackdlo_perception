// Copyright 2026 Hayato Shimada
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd
#ifndef TRACKDLO_CORE__POINTCLOUD_CUDA_CUH_
#define TRACKDLO_CORE__POINTCLOUD_CUDA_CUH_

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace trackdlo_core {
namespace cuda {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_pointcloud(
    const cv::Mat& mask,
    const cv::Mat& depth_image,
    const cv::Mat& color_image,
    const Eigen::MatrixXd& proj_matrix);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_downsampled_pointcloud(
    const cv::Mat& mask,
    const cv::Mat& depth_image,
    const cv::Mat& color_image,
    const Eigen::MatrixXd& proj_matrix,
    float leaf_size);

} // namespace cuda
} // namespace trackdlo_core

#endif // TRACKDLO_CORE__POINTCLOUD_CUDA_CUH_
