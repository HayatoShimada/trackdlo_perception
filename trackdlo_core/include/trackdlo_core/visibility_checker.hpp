// Copyright 2026 Hayato Shimada
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd
#ifndef TRACKDLO_CORE__VISIBILITY_CHECKER_HPP_
#define TRACKDLO_CORE__VISIBILITY_CHECKER_HPP_

#include <Eigen/Dense>

#include <vector>

#include <opencv2/opencv.hpp>

namespace trackdlo_core
{

struct VisibilityResult
{
  std::vector<int> visible_nodes;            // Nodes within visibility threshold and not occluded
  std::vector<int> not_self_occluded_nodes;  // Nodes not self-occluded (for visualization)
};

class VisibilityChecker
{
public:
  VisibilityChecker() = default;

  // Checks properties like self-occlusion and visible nodes
  VisibilityResult check_visibility(
    const Eigen::MatrixXd & Y,
    const Eigen::MatrixXd & X,
    const Eigen::MatrixXd & proj_matrix,
    const cv::Mat & mask,
    double visibility_threshold,
    int dlo_pixel_width);
};

}  // namespace trackdlo_core

#endif  // TRACKDLO_CORE__VISIBILITY_CHECKER_HPP_
