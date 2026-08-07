#ifndef LOCAL_MAPPING_SEMANTIC_DEPTH_FILTER_H_
#define LOCAL_MAPPING_SEMANTIC_DEPTH_FILTER_H_

#include <vector>

#include <darknet_ros_msgs/BoundingBox.h>
#include <opencv2/core.hpp>

namespace local_mapping {

struct FilteredDepth {
  cv::Mat planner_depth;
  cv::Mat static_depth;
  cv::Mat person_mask;
};

class SemanticDepthFilter {
 public:
  explicit SemanticDepthFilter(int mask_margin_pixels);

  FilteredDepth apply(
      const cv::Mat& depth,
      const std::vector<darknet_ros_msgs::BoundingBox>& boxes) const;

 private:
  int mask_margin_pixels_;
};

}  // namespace local_mapping

#endif  // LOCAL_MAPPING_SEMANTIC_DEPTH_FILTER_H_
