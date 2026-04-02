// Copyright 2026 Hayato Shimada
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "trackdlo_core/trackdlo.hpp"

TEST(TrackDLOTest, DefaultConstructor)
{
  trackdlo tracker;
  EXPECT_EQ(tracker.get_tracking_result().rows(), 0);
}

TEST(TrackDLOTest, ConstructorWithNodeCount)
{
  trackdlo tracker(10);
  // After construction, Y_ should be zero-initialized with 10 rows
  Eigen::MatrixXd result = tracker.get_tracking_result();
  EXPECT_EQ(result.rows(), 10);
  EXPECT_EQ(result.cols(), 3);
}

TEST(TrackDLOTest, FullConstructor)
{
  trackdlo tracker(
    20,     // num_of_nodes
    0.008,  // visibility_threshold
    0.35,   // beta
    50000,  // lambda
    3.0,    // alpha
    50.0,   // k_vis
    0.1,    // mu
    30,     // max_iter
    0.001,  // tol
    3.0,    // beta_pre_proc
    1.0,    // lambda_pre_proc
    10.0);  // lle_weight
  EXPECT_EQ(tracker.get_tracking_result().rows(), 20);
}

TEST(TrackDLOTest, InitializeNodes)
{
  trackdlo tracker(5);

  Eigen::MatrixXd Y_init(5, 3);
  Y_init << 0, 0, 0.5,
    0.1, 0, 0.5,
    0.2, 0, 0.5,
    0.3, 0, 0.5,
    0.4, 0, 0.5;

  tracker.initialize_nodes(Y_init);
  Eigen::MatrixXd result = tracker.get_tracking_result();
  EXPECT_NEAR(result(0, 0), 0.0, 1e-6);
  EXPECT_NEAR(result(4, 0), 0.4, 1e-6);
}

TEST(TrackDLOTest, Sigma2GetSet)
{
  trackdlo tracker(5);
  tracker.set_sigma2(0.5);
  EXPECT_NEAR(tracker.get_sigma2(), 0.5, 1e-6);
}

TEST(TrackDLOTest, CpdLleConverges)
{
  trackdlo tracker(5);

  // Initialize nodes as a straight line
  Eigen::MatrixXd Y(5, 3);
  Y << 0, 0, 0.5,
    0.1, 0, 0.5,
    0.2, 0, 0.5,
    0.3, 0, 0.5,
    0.4, 0, 0.5;

  // Create a point cloud near the nodes (slightly shifted)
  Eigen::MatrixXd X(20, 3);
  for (int i = 0; i < 20; i++) {
    X(i, 0) = 0.02 * i;
    X(i, 1) = 0.01;  // small offset
    X(i, 2) = 0.5;
  }

  double sigma2 = 0.1;
  bool converged = tracker.cpd_lle(
    X, Y, sigma2,
    1.0,    // beta
    1.0,    // lambda
    1.0,    // lle_weight
    0.1,    // mu
    10,     // max_iter
    0.001,  // tol
    true);  // include_lle

  // Should not crash and sigma2 should decrease
  EXPECT_LT(sigma2, 0.1);
}

TEST(TrackDLOTest, CpdLleEmptyPointCloud)
{
  trackdlo tracker(5);

  Eigen::MatrixXd Y(5, 3);
  Y << 0, 0, 0.5,
    0.1, 0, 0.5,
    0.2, 0, 0.5,
    0.3, 0, 0.5,
    0.4, 0, 0.5;

  Eigen::MatrixXd X(0, 3);  // empty
  double sigma2 = 0.1;

  // Should handle gracefully without crashing
  bool result = tracker.cpd_lle(
    X, Y, sigma2, 1.0, 1.0, 1.0, 0.1, 5, 0.001, true);
  (void)result;  // Just check it doesn't crash
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
