// Copyright 2026 Hayato Shimada
// SPDX-License-Identifier: BSD-3-Clause
#ifndef TRACKDLO_CORE__VISIBILITY_CHECKER_HPP_
#define TRACKDLO_CORE__VISIBILITY_CHECKER_HPP_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>

namespace trackdlo_core {

struct VisibilityResult {
    std::vector<int> visible_nodes;           // Nodes well within visibility threshold and not occluded
    std::vector<int> not_self_occluded_nodes; // Nodes that are not self-occluded (used for visualization)
};

class VisibilityChecker {
public:
    VisibilityChecker() = default;

    // Checks properties like self-occlusion and visible nodes 
    VisibilityResult check_visibility(
        const Eigen::MatrixXd& Y,
        const Eigen::MatrixXd& X,
        const Eigen::MatrixXd& proj_matrix,
        const cv::Mat& mask,
        double visibility_threshold,
        int dlo_pixel_width);

};

} // namespace trackdlo_core

#endif // TRACKDLO_CORE__VISIBILITY_CHECKER_HPP_
