// Copyright 2026 Hayato Shimada
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd
#ifndef TRACKDLO_CORE__VISUALIZER_HPP_
#define TRACKDLO_CORE__VISUALIZER_HPP_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>

namespace trackdlo_core
{

class Visualizer
{
public:
  Visualizer() = default;
  ~Visualizer() = default;

  /**
   * @brief Generates a tracking image with nodes and connections drawn
   *
   * @param original_image Original RGB image
   * @param processed_image Processed/masked image
   * @param Y Current tracking results (nodes)
   * @param proj_matrix Camera projection matrix
   * @param visible_nodes Vector of nodes that are visible
   * @return cv::Mat containing the tracking visualization
   */
  cv::Mat draw_tracking_image(
    const cv::Mat & original_image,
    const cv::Mat & processed_image,
    const Eigen::MatrixXd & Y,
    const Eigen::MatrixXd & proj_matrix,
    const std::vector<int> & visible_nodes);
};

} // namespace trackdlo_core

#endif // TRACKDLO_CORE__VISUALIZER_HPP_
