// Copyright 2026 Hayato Shimada
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <vector>

#include <opencv2/opencv.hpp>

#include "trackdlo_core/visibility_checker.hpp"

TEST(VisibilityCheckerTest, AllNodesVisible)
{
  trackdlo_core::VisibilityChecker checker;

  // 5 nodes in a straight line at z=0.5
  Eigen::MatrixXd Y(5, 3);
  Y << 0.0, 0.0, 0.5,
    0.05, 0.0, 0.5,
    0.10, 0.0, 0.5,
    0.15, 0.0, 0.5,
    0.20, 0.0, 0.5;

  // Point cloud near the nodes
  Eigen::MatrixXd X(50, 3);
  for (int i = 0; i < 50; i++) {
    X(i, 0) = 0.004 * i;
    X(i, 1) = 0.0;
    X(i, 2) = 0.5;
  }

  // Simple projection matrix (focal length 500, center 320,240)
  Eigen::MatrixXd proj(3, 4);
  proj << 500, 0, 320, 0,
    0, 500, 240, 0,
    0, 0, 1, 0;

  // White mask (everything visible)
  cv::Mat mask(480, 640, CV_8UC1, cv::Scalar(255));

  trackdlo_core::VisibilityResult result = checker.check_visibility(
    Y, X, proj, mask, 0.02, 10);

  // All or most nodes should be visible
  EXPECT_GE(static_cast<int>(result.visible_nodes.size()), 3);
}

TEST(VisibilityCheckerTest, ConstructsWithoutCrash)
{
  trackdlo_core::VisibilityChecker checker;
  // Should construct without issues
  SUCCEED();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
