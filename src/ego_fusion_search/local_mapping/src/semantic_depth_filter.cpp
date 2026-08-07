#include "local_mapping/semantic_depth_filter.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace local_mapping {
namespace {

bool isCompetitionPerson(const std::string& class_id) {
  static const std::array<std::string, 6> person_ids{
      {"green0", "blue1", "brown2", "white3", "red4", "red5"}};
  return std::find(person_ids.begin(), person_ids.end(), class_id) !=
         person_ids.end();
}

std::int64_t saturatingSubtract(std::int64_t value, std::int64_t amount) {
  if (value < std::numeric_limits<std::int64_t>::min() + amount) {
    return std::numeric_limits<std::int64_t>::min();
  }
  return value - amount;
}

std::int64_t saturatingAdd(std::int64_t value, std::int64_t amount) {
  if (value > std::numeric_limits<std::int64_t>::max() - amount) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return value + amount;
}

bool clippedInterval(std::int64_t minimum, std::int64_t maximum,
                     std::int64_t margin, int limit, int* clipped_minimum,
                     int* clipped_maximum) {
  if (minimum > maximum || maximum < 0 || minimum >= limit) {
    return false;
  }

  const std::int64_t expanded_minimum = saturatingSubtract(minimum, margin);
  const std::int64_t expanded_maximum = saturatingAdd(maximum, margin);
  if (expanded_maximum < 0 || expanded_minimum >= limit) {
    return false;
  }

  *clipped_minimum =
      static_cast<int>(std::max<std::int64_t>(0, expanded_minimum));
  *clipped_maximum = static_cast<int>(
      std::min<std::int64_t>(static_cast<std::int64_t>(limit) - 1,
                             expanded_maximum));
  return *clipped_minimum <= *clipped_maximum;
}

}  // namespace

SemanticDepthFilter::SemanticDepthFilter(int mask_margin_pixels)
    : mask_margin_pixels_(mask_margin_pixels) {
  if (mask_margin_pixels_ < 0) {
    throw std::invalid_argument("mask margin must not be negative");
  }
}

FilteredDepth SemanticDepthFilter::apply(
    const cv::Mat& depth,
    const std::vector<darknet_ros_msgs::BoundingBox>& boxes) const {
  if (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) {
    throw std::invalid_argument("depth must have type CV_16UC1 or CV_32FC1");
  }

  if (depth.empty()) {
    return FilteredDepth{cv::Mat(depth.rows, depth.cols, depth.type()),
                         cv::Mat(depth.rows, depth.cols, depth.type()),
                         cv::Mat(depth.rows, depth.cols, CV_8UC1)};
  }

  FilteredDepth result{depth.clone(), depth.clone(),
                       cv::Mat::zeros(depth.size(), CV_8UC1)};

  const std::int64_t margin = mask_margin_pixels_;
  for (const auto& detected_box : boxes) {
    if (!isCompetitionPerson(detected_box.Class)) {
      continue;
    }

    int xmin = 0;
    int xmax = 0;
    int ymin = 0;
    int ymax = 0;
    if (!clippedInterval(detected_box.xmin, detected_box.xmax, margin,
                         depth.cols, &xmin, &xmax) ||
        !clippedInterval(detected_box.ymin, detected_box.ymax, margin,
                         depth.rows, &ymin, &ymax)) {
      continue;
    }

    const cv::Rect region(xmin, ymin, xmax - xmin + 1, ymax - ymin + 1);
    result.person_mask(region).setTo(cv::Scalar(255));
  }

  result.static_depth.setTo(cv::Scalar(0), result.person_mask);
  return result;
}

}  // namespace local_mapping
