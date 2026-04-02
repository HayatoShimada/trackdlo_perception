// Copyright 2026 Hayato Shimada
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include <gtest/gtest.h>

#include <vector>

#include <opencv2/opencv.hpp>

#include "trackdlo_core/image_preprocessor.hpp"

TEST(ImagePreprocessorTest, HsvMaskBlueObject)
{
  // Default HSV range: lower=[85,50,20], upper=[135,255,255]
  std::vector<int> lower = {85, 50, 20};
  std::vector<int> upper = {135, 255, 255};
  trackdlo_core::ImagePreprocessor preprocessor(false, false, lower, upper);

  // Create a blue image (BGR: [200, 80, 30] -> HSV H~105)
  cv::Mat blue_img(480, 640, CV_8UC3, cv::Scalar(200, 80, 30));

  cv::Mat mask, cur_image;
  bool result = preprocessor.process(blue_img, mask, cur_image);

  EXPECT_TRUE(result);
  EXPECT_EQ(mask.rows, 480);
  EXPECT_EQ(mask.cols, 640);
  // Blue should be detected — most pixels should be white
  int white_count = cv::countNonZero(mask);
  EXPECT_GT(white_count, 480 * 640 * 0.9);
}

TEST(ImagePreprocessorTest, HsvMaskBlackImage)
{
  std::vector<int> lower = {85, 50, 20};
  std::vector<int> upper = {135, 255, 255};
  trackdlo_core::ImagePreprocessor preprocessor(false, false, lower, upper);

  // Black image — nothing should be detected
  cv::Mat black_img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

  cv::Mat mask, cur_image;
  bool result = preprocessor.process(black_img, mask, cur_image);

  // May return false (no mask generated) or mask with all zeros
  if (result) {
    int white_count = cv::countNonZero(mask);
    EXPECT_EQ(white_count, 0);
  }
}

TEST(ImagePreprocessorTest, ExternalMaskMode)
{
  std::vector<int> lower = {85, 50, 20};
  std::vector<int> upper = {135, 255, 255};
  trackdlo_core::ImagePreprocessor preprocessor(true, false, lower, upper);

  EXPECT_FALSE(preprocessor.has_external_mask());

  // Set external mask
  cv::Mat ext_mask(480, 640, CV_8UC1, cv::Scalar(255));
  preprocessor.set_external_mask(ext_mask);

  EXPECT_TRUE(preprocessor.has_external_mask());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
