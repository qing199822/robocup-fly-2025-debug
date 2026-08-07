#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <darknet_ros_msgs/BoundingBox.h>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "local_mapping/semantic_depth_filter.h"

namespace {

darknet_ros_msgs::BoundingBox box(const std::string& class_id,
                                  std::int64_t xmin, std::int64_t ymin,
                                  std::int64_t xmax, std::int64_t ymax) {
  darknet_ros_msgs::BoundingBox result;
  result.Class = class_id;
  result.xmin = xmin;
  result.ymin = ymin;
  result.xmax = xmax;
  result.ymax = ymax;
  return result;
}

TEST(SemanticDepthFilter, KeepsPlannerObstacleButMasksPersistentDepth) {
  cv::Mat depth(6, 8, CV_16UC1, cv::Scalar(4000));
  const auto result = local_mapping::SemanticDepthFilter(1).apply(
      depth, {box("green0", 2, 2, 4, 3)});

  EXPECT_EQ(4000, result.planner_depth.at<std::uint16_t>(2, 3));
  EXPECT_EQ(0, result.static_depth.at<std::uint16_t>(2, 3));
  EXPECT_EQ(255, result.person_mask.at<std::uint8_t>(1, 1));
  EXPECT_EQ(4000, result.static_depth.at<std::uint16_t>(0, 7));
}

TEST(SemanticDepthFilter, MasksExactlyTheSixCompetitionPersonIds) {
  const std::vector<std::string> person_ids{
      "green0", "blue1", "brown2", "white3", "red4", "red5"};

  for (const auto& person_id : person_ids) {
    SCOPED_TRACE(person_id);
    cv::Mat depth(1, 1, CV_16UC1, cv::Scalar(17));
    const auto result = local_mapping::SemanticDepthFilter(0).apply(
        depth, {box(person_id, 0, 0, 0, 0)});
    EXPECT_EQ(0, result.static_depth.at<std::uint16_t>(0, 0));
    EXPECT_EQ(255, result.person_mask.at<std::uint8_t>(0, 0));
  }

  for (const auto& other_id : {"person", "green", "red6", "car", ""}) {
    SCOPED_TRACE(other_id);
    cv::Mat depth(1, 1, CV_16UC1, cv::Scalar(17));
    const auto result = local_mapping::SemanticDepthFilter(0).apply(
        depth, {box(other_id, 0, 0, 0, 0)});
    EXPECT_EQ(17, result.static_depth.at<std::uint16_t>(0, 0));
    EXPECT_EQ(0, result.person_mask.at<std::uint8_t>(0, 0));
  }
}

TEST(SemanticDepthFilter, ExpandsInclusiveBoxAndClipsToImageBoundary) {
  cv::Mat depth(4, 5, CV_16UC1, cv::Scalar(9));
  const auto result = local_mapping::SemanticDepthFilter(2).apply(
      depth, {box("blue1", 0, 1, 1, 2)});

  EXPECT_EQ(255, result.person_mask.at<std::uint8_t>(0, 0));
  EXPECT_EQ(255, result.person_mask.at<std::uint8_t>(3, 3));
  EXPECT_EQ(0, result.person_mask.at<std::uint8_t>(3, 4));
  EXPECT_EQ(0, result.static_depth.at<std::uint16_t>(3, 3));
  EXPECT_EQ(9, result.static_depth.at<std::uint16_t>(3, 4));
}

TEST(SemanticDepthFilter, IgnoresInvalidAndCompletelyOutOfImageBoxes) {
  cv::Mat depth(3, 4, CV_16UC1, cv::Scalar(12));
  const std::vector<darknet_ros_msgs::BoundingBox> boxes{
      box("green0", 3, 1, 2, 2),
      box("blue1", 1, 2, 2, 1),
      box("brown2", -20, -10, -2, -1),
      box("white3", 8, 5, 9, 6),
      box("red4", -1, 0, -1, 0),
      box("red5", 4, 2, 4, 2)};

  const auto result =
      local_mapping::SemanticDepthFilter(1).apply(depth, boxes);
  EXPECT_EQ(0, cv::countNonZero(result.person_mask));
  EXPECT_EQ(0, cv::countNonZero(result.static_depth != depth));
}

TEST(SemanticDepthFilter, HandlesExtremeCoordinatesWithoutOverflow) {
  cv::Mat depth(2, 2, CV_16UC1, cv::Scalar(31));
  const auto minimum = std::numeric_limits<std::int64_t>::min();
  const auto maximum = std::numeric_limits<std::int64_t>::max();
  const std::vector<darknet_ros_msgs::BoundingBox> boxes{
      box("red4", minimum, minimum, minimum, minimum),
      box("red5", maximum, maximum, maximum, maximum),
      box("green0", minimum, 0, maximum, 1)};

  const auto result =
      local_mapping::SemanticDepthFilter(7).apply(depth, boxes);
  EXPECT_EQ(4, cv::countNonZero(result.person_mask));
  EXPECT_EQ(0, cv::countNonZero(result.static_depth));
}

TEST(SemanticDepthFilter, SupportsFloatDepthAndWritesFloatZero) {
  cv::Mat depth(2, 3, CV_32FC1, cv::Scalar(2.5F));
  const auto result = local_mapping::SemanticDepthFilter(0).apply(
      depth, {box("white3", 1, 0, 2, 1)});

  EXPECT_FLOAT_EQ(2.5F, result.planner_depth.at<float>(0, 1));
  EXPECT_FLOAT_EQ(0.0F, result.static_depth.at<float>(0, 1));
  EXPECT_FLOAT_EQ(2.5F, result.static_depth.at<float>(0, 0));
}

TEST(SemanticDepthFilter, RejectsUnsupportedDepthTypes) {
  const local_mapping::SemanticDepthFilter filter(0);
  EXPECT_THROW(filter.apply(cv::Mat(2, 2, CV_8UC1), {}),
               std::invalid_argument);
  EXPECT_THROW(filter.apply(cv::Mat(2, 2, CV_16UC2), {}),
               std::invalid_argument);
  EXPECT_THROW(filter.apply(cv::Mat(2, 2, CV_32FC3), {}),
               std::invalid_argument);
}

TEST(SemanticDepthFilter, RejectsNegativeMargin) {
  EXPECT_THROW(local_mapping::SemanticDepthFilter(-1), std::invalid_argument);
}

TEST(SemanticDepthFilter,
     ReturnsIndependentDeepCopiesAndLeavesInputUnchanged) {
  cv::Mat depth(2, 2, CV_16UC1, cv::Scalar(101));
  const cv::Mat original = depth.clone();
  auto result = local_mapping::SemanticDepthFilter(0).apply(
      depth, {box("red4", 0, 0, 0, 0)});

  EXPECT_NE(depth.data, result.planner_depth.data);
  EXPECT_NE(depth.data, result.static_depth.data);
  EXPECT_NE(result.planner_depth.data, result.static_depth.data);
  EXPECT_EQ(0, cv::countNonZero(depth != original));

  result.planner_depth.at<std::uint16_t>(1, 1) = 202;
  result.static_depth.at<std::uint16_t>(1, 1) = 303;
  EXPECT_EQ(101, depth.at<std::uint16_t>(1, 1));
  EXPECT_EQ(202, result.planner_depth.at<std::uint16_t>(1, 1));
  EXPECT_EQ(303, result.static_depth.at<std::uint16_t>(1, 1));
}

TEST(SemanticDepthFilter, AcceptsTypedEmptyDepthSafely) {
  const cv::Mat empty_depth(0, 0, CV_32FC1);
  const auto result = local_mapping::SemanticDepthFilter(0).apply(
      empty_depth, {box("green0", 0, 0, 1, 1)});

  EXPECT_TRUE(result.planner_depth.empty());
  EXPECT_TRUE(result.static_depth.empty());
  EXPECT_TRUE(result.person_mask.empty());
  EXPECT_EQ(CV_32FC1, result.planner_depth.type());
  EXPECT_EQ(CV_32FC1, result.static_depth.type());
  EXPECT_EQ(CV_8UC1, result.person_mask.type());
}

TEST(SemanticDepthFilter, EmptyBoxesPreserveDepthAndProduceZeroMask) {
  cv::Mat depth(2, 3, CV_16UC1);
  depth.at<std::uint16_t>(0, 0) = 1;
  depth.at<std::uint16_t>(0, 1) = 2;
  depth.at<std::uint16_t>(0, 2) = 3;
  depth.at<std::uint16_t>(1, 0) = 4;
  depth.at<std::uint16_t>(1, 1) = 5;
  depth.at<std::uint16_t>(1, 2) = 6;

  const auto result = local_mapping::SemanticDepthFilter(3).apply(depth, {});
  EXPECT_EQ(0, cv::countNonZero(result.planner_depth != depth));
  EXPECT_EQ(0, cv::countNonZero(result.static_depth != depth));
  EXPECT_EQ(0, cv::countNonZero(result.person_mask));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
