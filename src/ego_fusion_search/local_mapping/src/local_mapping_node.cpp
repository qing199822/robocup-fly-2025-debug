#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/bind/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <search_msgs/LocalClearance.h>
#include <search_msgs/PerceptionHealth.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "local_mapping/frontier_selector.h"
#include "local_mapping/health_monitor.h"
#include "local_mapping/semantic_depth_filter.h"
#include "local_mapping/voxel_map.h"

namespace local_mapping {
namespace {

constexpr char kMapFrame[] = "map";
constexpr std::size_t kRecentDepthDropCapacity = 64;

struct DepthMessageKey {
  std::uint32_t sequence;
  std::uint64_t stamp_nanoseconds;
};

struct NodeConfig {
  double max_sync_delta;
  double depth_timeout;
  double odom_timeout;
  double recovery_window;
  double min_valid_depth_ratio;
  double depth_min_m;
  double depth_max_m;
  int pixel_stride;
  int mask_margin_pixels;
  double voxel_resolution;
  int occupied_hits;
  int free_hits;
  double dynamic_ttl;
  double publish_rate;
  double search_altitude;
  double max_frontier_distance;
  double frontier_resolution;
  double vehicle_vertical_radius;
  int min_frontier_cluster_cells;
  double max_clearance_distance;
  std::string base_frame;
  std::string camera_frame;
  double tf_timeout;

  std::string depth_topic;
  std::string camera_info_topic;
  std::string odom_topic;
  std::string bounding_boxes_topic;
  std::string planner_depth_topic;
  std::string static_cloud_topic;
  std::string dynamic_cloud_topic;
  std::string health_topic;
  std::string clearance_topic;
  std::string frontier_goal_topic;
};

template <typename T>
T parameter(ros::NodeHandle* node, const std::string& name,
            const T& default_value) {
  T value = default_value;
  node->param(name, value, default_value);
  return value;
}

NodeConfig loadConfig(ros::NodeHandle* node) {
  NodeConfig config;
  config.max_sync_delta = parameter(node, "max_sync_delta", 0.15);
  config.depth_timeout = parameter(node, "depth_timeout", 0.50);
  config.odom_timeout = parameter(node, "odom_timeout", 0.50);
  config.recovery_window = parameter(node, "recovery_window", 1.00);
  config.min_valid_depth_ratio =
      parameter(node, "min_valid_depth_ratio", 0.20);
  config.depth_min_m = parameter(node, "depth_min_m", 0.20);
  config.depth_max_m = parameter(node, "depth_max_m", 8.00);
  config.pixel_stride = parameter(node, "pixel_stride", 4);
  config.mask_margin_pixels = parameter(node, "mask_margin_pixels", 4);
  config.voxel_resolution = parameter(node, "voxel_resolution", 0.20);
  config.occupied_hits = parameter(node, "occupied_hits", 2);
  config.free_hits = parameter(node, "free_hits", 2);
  config.dynamic_ttl = parameter(node, "dynamic_ttl", 1.00);
  config.publish_rate = parameter(node, "publish_rate", 5.0);
  config.search_altitude = parameter(node, "search_altitude", 3.0);
  config.max_frontier_distance =
      parameter(node, "max_frontier_distance", 8.0);
  config.frontier_resolution = parameter(node, "frontier_resolution", 0.25);
  config.vehicle_vertical_radius =
      parameter(node, "vehicle_vertical_radius", 0.20);
  config.min_frontier_cluster_cells =
      parameter(node, "min_frontier_cluster_cells", 2);
  config.max_clearance_distance =
      parameter(node, "max_clearance_distance", 8.0);
  config.base_frame = parameter<std::string>(node, "base_frame", "base_link");
  config.camera_frame =
      parameter<std::string>(node, "camera_frame", "depth_camera_base");
  config.tf_timeout = parameter(node, "tf_timeout", 0.05);

  config.depth_topic = parameter<std::string>(
      node, "depth_topic",
      "/typhoon_h480_0/realsense/depth_camera/depth/image_raw");
  config.camera_info_topic = parameter<std::string>(
      node, "camera_info_topic",
      "/typhoon_h480_0/realsense/depth_camera/depth/camera_info");
  config.odom_topic = parameter<std::string>(
      node, "odom_topic", "/typhoon_h480_0/global_odom");
  config.bounding_boxes_topic = parameter<std::string>(
      node, "bounding_boxes_topic",
      "/typhoon_h480_0/yolo11n/bounding_boxes");
  config.planner_depth_topic = parameter<std::string>(
      node, "planner_depth_topic",
      "/typhoon_h480_0/local_mapping/planner_depth");
  config.static_cloud_topic = parameter<std::string>(
      node, "static_cloud_topic",
      "/typhoon_h480_0/local_mapping/static_cloud");
  config.dynamic_cloud_topic = parameter<std::string>(
      node, "dynamic_cloud_topic",
      "/typhoon_h480_0/local_mapping/dynamic_cloud");
  config.health_topic = parameter<std::string>(
      node, "health_topic", "/typhoon_h480_0/local_mapping/health");
  config.clearance_topic = parameter<std::string>(
      node, "clearance_topic", "/typhoon_h480_0/local_mapping/clearance");
  config.frontier_goal_topic = parameter<std::string>(
      node, "frontier_goal_topic",
      "/typhoon_h480_0/local_mapping/frontier_goal");
  return config;
}

bool finite(double value) { return std::isfinite(value); }

void validateConfig(const NodeConfig& config) {
  const bool health_valid = finite(config.max_sync_delta) &&
                            config.max_sync_delta >= 0.0 &&
                            finite(config.depth_timeout) &&
                            config.depth_timeout > 0.0 &&
                            finite(config.odom_timeout) &&
                            config.odom_timeout > 0.0 &&
                            finite(config.recovery_window) &&
                            config.recovery_window > 0.0 &&
                            finite(config.min_valid_depth_ratio) &&
                            config.min_valid_depth_ratio >= 0.0 &&
                            config.min_valid_depth_ratio <= 1.0;
  const bool mapping_valid = finite(config.depth_min_m) &&
                             finite(config.depth_max_m) &&
                             config.depth_min_m >= 0.0 &&
                             config.depth_max_m > config.depth_min_m &&
                             config.pixel_stride > 0 &&
                             config.mask_margin_pixels >= 0 &&
                             finite(config.voxel_resolution) &&
                             config.voxel_resolution > 0.0 &&
                             config.occupied_hits > 0 &&
                             config.free_hits > 0 &&
                             finite(config.dynamic_ttl) &&
                             config.dynamic_ttl > 0.0;
  const bool publishing_valid =
      finite(config.publish_rate) && config.publish_rate > 0.0 &&
      finite(config.search_altitude) &&
      finite(config.max_frontier_distance) &&
      config.max_frontier_distance > 0.0 &&
      finite(config.frontier_resolution) && config.frontier_resolution > 0.0 &&
      finite(config.vehicle_vertical_radius) &&
      config.vehicle_vertical_radius >= 0.0 &&
      config.min_frontier_cluster_cells > 0 &&
      finite(config.max_clearance_distance) &&
      config.max_clearance_distance >= 0.0 && finite(config.tf_timeout) &&
      config.tf_timeout >= 0.0;
  const bool frames_and_topics_valid =
      !config.base_frame.empty() && !config.depth_topic.empty() &&
      !config.camera_info_topic.empty() && !config.odom_topic.empty() &&
      !config.bounding_boxes_topic.empty() &&
      !config.planner_depth_topic.empty() &&
      !config.static_cloud_topic.empty() &&
      !config.dynamic_cloud_topic.empty() && !config.health_topic.empty() &&
      !config.clearance_topic.empty() &&
      !config.frontier_goal_topic.empty();
  if (!health_valid || !mapping_valid || !publishing_valid ||
      !frames_and_topics_valid) {
    throw std::invalid_argument("invalid local mapping node configuration");
  }

  const long double side =
      2.0L * config.max_frontier_distance / config.frontier_resolution;
  if (!std::isfinite(side) ||
      side > static_cast<long double>(std::numeric_limits<std::uint32_t>::max())) {
    throw std::invalid_argument("frontier grid dimensions are too large");
  }
}

bool normalizedQuaternion(double x, double y, double z, double w,
                          tf2::Quaternion* result) {
  if (!finite(x) || !finite(y) || !finite(z) || !finite(w)) {
    return false;
  }
  const long double norm_squared = static_cast<long double>(x) * x +
                                   static_cast<long double>(y) * y +
                                   static_cast<long double>(z) * z +
                                   static_cast<long double>(w) * w;
  if (!std::isfinite(norm_squared) ||
      norm_squared <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  *result = tf2::Quaternion(x, y, z, w);
  result->normalize();
  return finite(result->x()) && finite(result->y()) && finite(result->z()) &&
         finite(result->w());
}

bool finitePoint(const Vec3& point) {
  return finite(point.x) && finite(point.y) && finite(point.z);
}

std::string normalizedFrame(const std::string& frame) {
  const std::size_t first_character = frame.find_first_not_of('/');
  return first_character == std::string::npos
             ? std::string()
             : frame.substr(first_character);
}

bool sameFrame(const std::string& first, const std::string& second) {
  const std::string normalized_first = normalizedFrame(first);
  return !normalized_first.empty() &&
         normalized_first == normalizedFrame(second);
}

bool concreteMappingFault(const std::string& fault) {
  return fault == "FRAME_ERROR" || fault == "TF_ERROR" ||
         fault == "CAMERA_INFO_ERROR" ||
         fault == "DEPTH_ENCODING_ERROR" || fault == "MAP_INPUT_ERROR" ||
         fault == "SYNC_ERROR";
}

}  // namespace

class LocalMappingNode {
 public:
  LocalMappingNode()
      : node_(),
        private_node_("~"),
        config_(loadConfig(&private_node_)),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        has_reliable_odom_(false),
        last_fusion_valid_(false),
        has_successful_fusion_(false),
        last_successful_fusion_time_(0.0),
        last_mapping_fault_("NOT_READY"),
        cached_boxes_used_(false) {
    validateConfig(config_);
    health_monitor_.reset(new HealthMonitor(
        HealthConfig{config_.max_sync_delta, config_.depth_timeout,
                     config_.odom_timeout, config_.recovery_window,
                     config_.min_valid_depth_ratio}));
    semantic_filter_.reset(
        new SemanticDepthFilter(config_.mask_margin_pixels));
    voxel_map_.reset(new VoxelMap(config_.voxel_resolution,
                                  config_.occupied_hits, config_.free_hits,
                                  config_.dynamic_ttl));
    frontier_selector_.reset(new FrontierSelector(
        config_.min_frontier_cluster_cells, config_.max_frontier_distance));

    planner_depth_publisher_ =
        node_.advertise<sensor_msgs::Image>(config_.planner_depth_topic, 1);
    static_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
        config_.static_cloud_topic, 1);
    dynamic_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
        config_.dynamic_cloud_topic, 1);
    health_publisher_ = node_.advertise<search_msgs::PerceptionHealth>(
        config_.health_topic, 1);
    clearance_publisher_ = node_.advertise<search_msgs::LocalClearance>(
        config_.clearance_topic, 1);
    frontier_goal_publisher_ = node_.advertise<geometry_msgs::PoseStamped>(
        config_.frontier_goal_topic, 1);

    boxes_subscriber_ = node_.subscribe(
        config_.bounding_boxes_topic, 1,
        &LocalMappingNode::boundingBoxesCallback, this);
    health_depth_subscriber_ = node_.subscribe(
        config_.depth_topic, 5, &LocalMappingNode::depthHealthCallback, this);
    health_odom_subscriber_ = node_.subscribe(
        config_.odom_topic, 5, &LocalMappingNode::odomHealthCallback, this);
    depth_subscriber_.subscribe(node_, config_.depth_topic, 5);
    camera_info_subscriber_.subscribe(node_, config_.camera_info_topic, 5);
    odom_subscriber_.subscribe(node_, config_.odom_topic, 5);
    synchronizer_.reset(new Synchronizer(
        SyncPolicy(5), depth_subscriber_, camera_info_subscriber_,
        odom_subscriber_));
    synchronizer_->registerCallback(boost::bind(
        &LocalMappingNode::synchronizedCallback, this,
        boost::placeholders::_1, boost::placeholders::_2,
        boost::placeholders::_3));

    publish_timer_ = node_.createTimer(
        ros::Duration(1.0 / config_.publish_rate),
        &LocalMappingNode::publishTimerCallback, this);
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  bool decodeDepth(const sensor_msgs::ImageConstPtr& message,
                   cv_bridge::CvImageConstPtr* bridge,
                   bool report_error) const {
    if (message->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
        message->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
      return false;
    }
    try {
      *bridge = cv_bridge::toCvShare(message);
      return true;
    } catch (const cv_bridge::Exception& error) {
      if (report_error) {
        ROS_WARN_THROTTLE(1.0, "local mapping depth conversion failed: %s",
                          error.what());
      }
      return false;
    }
  }

  void noteDepthDropOnce(const sensor_msgs::Image& message) {
    const DepthMessageKey key{message.header.seq,
                              message.header.stamp.toNSec()};
    const auto duplicate = std::find_if(
        recent_depth_drop_keys_.begin(), recent_depth_drop_keys_.end(),
        [&key](const DepthMessageKey& previous) {
          return previous.sequence == key.sequence &&
                 previous.stamp_nanoseconds == key.stamp_nanoseconds;
        });
    if (duplicate != recent_depth_drop_keys_.end()) {
      return;
    }

    health_monitor_->noteDroppedFrame();
    recent_depth_drop_keys_.push_back(key);
    if (recent_depth_drop_keys_.size() > kRecentDepthDropCapacity) {
      recent_depth_drop_keys_.pop_front();
    }
  }

  void depthHealthCallback(const sensor_msgs::ImageConstPtr& message) {
    cv_bridge::CvImageConstPtr bridge;
    if (!decodeDepth(message, &bridge, true)) {
      health_monitor_->observeDepth(message->header.stamp.toSec(), 0.0);
      noteDepthDropOnce(*message);
      return;
    }
    health_monitor_->observeDepth(message->header.stamp.toSec(),
                                  validDepthRatio(bridge->image));
  }

  void odomHealthCallback(const nav_msgs::OdometryConstPtr& message) {
    health_monitor_->observeOdom(message->header.stamp.toSec());
  }

  void boundingBoxesCallback(
      const darknet_ros_msgs::BoundingBoxesConstPtr& message) {
    const ros::Time stamp = message->image_header.stamp.isZero()
                                ? message->header.stamp
                                : message->image_header.stamp;
    if (cached_boxes_) {
      if (stamp < cached_boxes_stamp_) {
        health_monitor_->noteDroppedFrame();
        return;
      }
      if (!cached_boxes_used_) {
        health_monitor_->noteDroppedFrame();
      }
    }
    cached_boxes_ = message;
    cached_boxes_stamp_ = stamp;
    cached_boxes_used_ = false;
  }

  std::vector<darknet_ros_msgs::BoundingBox> boxesForDepth(
      const ros::Time& depth_stamp) {
    if (!cached_boxes_) {
      health_monitor_->noteDroppedFrame();
      return {};
    }
    const double delta =
        std::fabs((cached_boxes_stamp_ - depth_stamp).toSec());
    if (finite(delta) && delta <= config_.max_sync_delta) {
      cached_boxes_used_ = true;
      return cached_boxes_->bounding_boxes;
    }

    health_monitor_->noteDroppedFrame();
    if (cached_boxes_stamp_ < depth_stamp) {
      cached_boxes_.reset();
      cached_boxes_used_ = false;
    }
    return {};
  }

  bool depthAt(const cv::Mat& depth, int row, int column,
               double* metres) const {
    double value = 0.0;
    if (depth.type() == CV_16UC1) {
      value = static_cast<double>(depth.at<std::uint16_t>(row, column)) *
              0.001;
    } else if (depth.type() == CV_32FC1) {
      value = static_cast<double>(depth.at<float>(row, column));
    } else {
      return false;
    }
    if (!finite(value) || value < config_.depth_min_m ||
        value > config_.depth_max_m) {
      return false;
    }
    *metres = value;
    return true;
  }

  double validDepthRatio(const cv::Mat& depth) const {
    std::size_t sampled = 0;
    std::size_t valid = 0;
    for (int row = 0; row < depth.rows; row += config_.pixel_stride) {
      for (int column = 0; column < depth.cols;
           column += config_.pixel_stride) {
        ++sampled;
        double metres = 0.0;
        if (depthAt(depth, row, column, &metres)) {
          ++valid;
        }
      }
    }
    return sampled == 0
               ? 0.0
               : static_cast<double>(valid) / static_cast<double>(sampled);
  }

  bool validCameraInfo(const sensor_msgs::CameraInfo& camera_info,
                       const cv::Mat& depth) const {
    return camera_info.width == static_cast<std::uint32_t>(depth.cols) &&
           camera_info.height == static_cast<std::uint32_t>(depth.rows) &&
           finite(camera_info.K[0]) && camera_info.K[0] > 0.0 &&
           finite(camera_info.K[4]) && camera_info.K[4] > 0.0 &&
           finite(camera_info.K[2]) && finite(camera_info.K[5]);
  }

  bool mapFromCamera(const nav_msgs::Odometry& odom,
                     const std::string& camera_frame,
                     tf2::Transform* transform) {
    const auto& position = odom.pose.pose.position;
    tf2::Quaternion odom_rotation;
    if (!finite(position.x) || !finite(position.y) || !finite(position.z) ||
        !normalizedQuaternion(
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w,
            &odom_rotation)) {
      last_mapping_fault_ = "ODOM_POSE_ERROR";
      return false;
    }

    if (camera_frame.empty()) {
      last_mapping_fault_ = "TF_ERROR";
      return false;
    }

    geometry_msgs::TransformStamped base_from_camera_message;
    try {
      base_from_camera_message = tf_buffer_.lookupTransform(
          config_.base_frame, camera_frame, ros::Time(0),
          ros::Duration(config_.tf_timeout));
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(1.0, "local mapping camera TF unavailable: %s",
                        error.what());
      last_mapping_fault_ = "TF_ERROR";
      return false;
    }

    if (!sameFrame(base_from_camera_message.header.frame_id,
                   config_.base_frame) ||
        !sameFrame(base_from_camera_message.child_frame_id, camera_frame)) {
      last_mapping_fault_ = "FRAME_ERROR";
      return false;
    }

    const auto& translation = base_from_camera_message.transform.translation;
    const auto& rotation = base_from_camera_message.transform.rotation;
    tf2::Quaternion camera_rotation;
    if (!finite(translation.x) || !finite(translation.y) ||
        !finite(translation.z) ||
        !normalizedQuaternion(rotation.x, rotation.y, rotation.z, rotation.w,
                              &camera_rotation)) {
      last_mapping_fault_ = "TF_ERROR";
      return false;
    }

    const tf2::Transform map_from_base(
        odom_rotation, tf2::Vector3(position.x, position.y, position.z));
    const tf2::Transform base_from_camera(
        camera_rotation,
        tf2::Vector3(translation.x, translation.y, translation.z));
    *transform = map_from_base * base_from_camera;
    return true;
  }

  void synchronizedCallback(
      const sensor_msgs::ImageConstPtr& depth_message,
      const sensor_msgs::CameraInfoConstPtr& camera_info,
      const nav_msgs::OdometryConstPtr& odom) {
    last_fusion_valid_ = false;
    const double depth_stamp = depth_message->header.stamp.toSec();
    const double odom_stamp = odom->header.stamp.toSec();

    cv_bridge::CvImageConstPtr bridge;
    if (!decodeDepth(depth_message, &bridge, false)) {
      noteDepthDropOnce(*depth_message);
      last_mapping_fault_ = "DEPTH_ENCODING_ERROR";
      return;
    }

    const cv::Mat& depth = bridge->image;
    const double valid_ratio = validDepthRatio(depth);

    FilteredDepth filtered;
    try {
      filtered = semantic_filter_->apply(depth, boxesForDepth(
                                                    depth_message->header.stamp));
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "local mapping semantic filter failed: %s",
                        error.what());
      health_monitor_->noteDroppedFrame();
      last_mapping_fault_ = "DEPTH_ENCODING_ERROR";
      return;
    }

    planner_depth_publisher_.publish(
        cv_bridge::CvImage(depth_message->header, depth_message->encoding,
                           filtered.planner_depth)
            .toImageMsg());

    const double camera_stamp = camera_info->header.stamp.toSec();
    if (!finite(depth_stamp) || !finite(odom_stamp) ||
        !finite(camera_stamp) ||
        std::fabs(depth_stamp - odom_stamp) > config_.max_sync_delta ||
        std::fabs(depth_stamp - camera_stamp) > config_.max_sync_delta) {
      health_monitor_->noteDroppedFrame();
      last_mapping_fault_ = "SYNC_ERROR";
      return;
    }
    if (!validCameraInfo(*camera_info, depth)) {
      health_monitor_->noteDroppedFrame();
      last_mapping_fault_ = "CAMERA_INFO_ERROR";
      return;
    }

    const std::string effective_camera_frame = normalizedFrame(
        config_.camera_frame.empty() ? camera_info->header.frame_id
                                     : config_.camera_frame);
    if (!sameFrame(odom->header.frame_id, kMapFrame) ||
        !sameFrame(odom->child_frame_id, config_.base_frame) ||
        !sameFrame(depth_message->header.frame_id, effective_camera_frame) ||
        !sameFrame(camera_info->header.frame_id, effective_camera_frame)) {
      health_monitor_->noteDroppedFrame();
      last_mapping_fault_ = "FRAME_ERROR";
      return;
    }

    const HealthResult frame_health =
        health_monitor_->evaluate(ros::Time::now().toSec());
    last_health_result_ = frame_health;
    if (!frame_health.healthy ||
        valid_ratio < config_.min_valid_depth_ratio) {
      last_mapping_fault_ = "NOT_READY";
      return;
    }

    tf2::Transform map_from_camera;
    if (!mapFromCamera(*odom, effective_camera_frame, &map_from_camera)) {
      health_monitor_->noteDroppedFrame();
      return;
    }

    const tf2::Vector3 camera_origin_tf = map_from_camera.getOrigin();
    const Vec3 camera_origin{camera_origin_tf.x(), camera_origin_tf.y(),
                             camera_origin_tf.z()};
    const Vec3 odom_position{odom->pose.pose.position.x,
                             odom->pose.pose.position.y,
                             odom->pose.pose.position.z};
    if (!finitePoint(camera_origin) || !finitePoint(odom_position)) {
      health_monitor_->noteDroppedFrame();
      last_mapping_fault_ = "TF_ERROR";
      return;
    }
    latest_odom_position_ = odom_position;
    has_reliable_odom_ = true;

    const double fx = camera_info->K[0];
    const double fy = camera_info->K[4];
    const double cx = camera_info->K[2];
    const double cy = camera_info->K[5];
    try {
      for (int row = 0; row < depth.rows; row += config_.pixel_stride) {
        for (int column = 0; column < depth.cols;
             column += config_.pixel_stride) {
          double metres = 0.0;
          if (!depthAt(depth, row, column, &metres)) {
            continue;
          }
          const tf2::Vector3 optical_point(
              (static_cast<double>(column) - cx) * metres / fx,
              (static_cast<double>(row) - cy) * metres / fy, metres);
          const tf2::Vector3 map_point_tf = map_from_camera * optical_point;
          const Vec3 map_point{map_point_tf.x(), map_point_tf.y(),
                               map_point_tf.z()};
          if (!finitePoint(map_point)) {
            throw std::invalid_argument("transformed depth point is invalid");
          }
          if (filtered.person_mask.at<std::uint8_t>(row, column) != 0u) {
            voxel_map_->integrateDynamicPoint(map_point, depth_stamp);
          } else {
            double static_metres = 0.0;
            if (depthAt(filtered.static_depth, row, column,
                        &static_metres)) {
              voxel_map_->integrateStaticRay(camera_origin, map_point);
            }
          }
        }
      }
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "local mapping integration failed: %s",
                        error.what());
      health_monitor_->noteDroppedFrame();
      last_mapping_fault_ = "MAP_INPUT_ERROR";
      return;
    }

    last_successful_fusion_time_ = ros::Time::now().toSec();
    has_successful_fusion_ = finite(last_successful_fusion_time_);
    last_fusion_valid_ = has_successful_fusion_;
    last_mapping_fault_ = has_successful_fusion_ ? "OK" : "NOT_READY";
  }

  sensor_msgs::PointCloud2 cloudMessage(const std::vector<Vec3>& points,
                                        const ros::Time& stamp) const {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(points.size());
    for (const Vec3& point : points) {
      cloud.emplace_back(static_cast<float>(point.x),
                         static_cast<float>(point.y),
                         static_cast<float>(point.z));
    }
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.stamp = stamp;
    message.header.frame_id = kMapFrame;
    return message;
  }

  Clearance safeClearance(const Vec3& axis, double now) const {
    if (!has_reliable_odom_) {
      return Clearance{false, 0.0};
    }
    try {
      const Clearance clearance = voxel_map_->axisClearance(
          latest_odom_position_, axis, config_.max_clearance_distance, now);
      if (!clearance.known || !finite(clearance.metres) ||
          clearance.metres < 0.0) {
        return Clearance{false, 0.0};
      }
      return clearance;
    } catch (const std::exception&) {
      return Clearance{false, 0.0};
    }
  }

  search_msgs::LocalClearance clearanceMessage(const ros::Time& stamp) const {
    search_msgs::LocalClearance message;
    message.header.stamp = stamp;
    message.header.frame_id = kMapFrame;
    const double now = stamp.toSec();
    const Clearance forward = safeClearance({1.0, 0.0, 0.0}, now);
    const Clearance backward = safeClearance({-1.0, 0.0, 0.0}, now);
    const Clearance left = safeClearance({0.0, 1.0, 0.0}, now);
    const Clearance right = safeClearance({0.0, -1.0, 0.0}, now);
    const Clearance upward = safeClearance({0.0, 0.0, 1.0}, now);
    const Clearance downward = safeClearance({0.0, 0.0, -1.0}, now);
    message.forward_known = forward.known;
    message.backward_known = backward.known;
    message.left_known = left.known;
    message.right_known = right.known;
    message.upward_known = upward.known;
    message.downward_known = downward.known;
    message.forward_m = forward.metres;
    message.backward_m = backward.metres;
    message.left_m = left.metres;
    message.right_m = right.metres;
    message.upward_m = upward.metres;
    message.downward_m = downward.metres;
    return message;
  }

  nav_msgs::OccupancyGrid frontierGrid(const ros::Time& stamp) const {
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = kMapFrame;
    grid.info.resolution = config_.frontier_resolution;
    grid.info.width = static_cast<std::uint32_t>(std::ceil(
        2.0 * config_.max_frontier_distance / config_.frontier_resolution));
    grid.info.height = grid.info.width;
    grid.info.origin.position.x =
        latest_odom_position_.x - config_.max_frontier_distance;
    grid.info.origin.position.y =
        latest_odom_position_.y - config_.max_frontier_distance;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(grid.info.width) *
                         static_cast<std::size_t>(grid.info.height),
                     -1);

    const double minimum_z =
        config_.search_altitude - config_.vehicle_vertical_radius;
    const double maximum_z =
        config_.search_altitude + config_.vehicle_vertical_radius;
    const std::size_t vertical_steps = static_cast<std::size_t>(std::ceil(
        (maximum_z - minimum_z) / config_.voxel_resolution));
    const double now = stamp.toSec();
    for (std::uint32_t y = 0; y < grid.info.height; ++y) {
      for (std::uint32_t x = 0; x < grid.info.width; ++x) {
        const double world_x = grid.info.origin.position.x +
                               (static_cast<double>(x) + 0.5) *
                                   config_.frontier_resolution;
        const double world_y = grid.info.origin.position.y +
                               (static_cast<double>(y) + 0.5) *
                                   config_.frontier_resolution;
        bool occupied = false;
        bool all_free = true;
        for (std::size_t step = 0; step <= vertical_steps; ++step) {
          const double z = std::min(
              maximum_z,
              minimum_z + static_cast<double>(step) * config_.voxel_resolution);
          const CellState state = voxel_map_->stateAt({world_x, world_y, z}, now);
          if (state == CellState::OCCUPIED) {
            occupied = true;
            break;
          }
          if (state != CellState::FREE) {
            all_free = false;
          }
        }
        const std::size_t index = static_cast<std::size_t>(y) * grid.info.width + x;
        grid.data[index] = occupied ? 100 : (all_free ? 0 : -1);
      }
    }
    return grid;
  }

  void publishFrontier(const ros::Time& stamp) {
    if (!has_reliable_odom_) {
      return;
    }
    try {
      const nav_msgs::OccupancyGrid grid = frontierGrid(stamp);
      const FrontierGoal goal = frontier_selector_->select(
          grid, Vec3{latest_odom_position_.x, latest_odom_position_.y,
                     config_.search_altitude});
      if (!goal.valid) {
        return;
      }
      geometry_msgs::PoseStamped message;
      message.header.stamp = stamp;
      message.header.frame_id = kMapFrame;
      message.pose.position.x = goal.x;
      message.pose.position.y = goal.y;
      message.pose.position.z = config_.search_altitude;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, goal.yaw);
      orientation.normalize();
      message.pose.orientation.x = orientation.x();
      message.pose.orientation.y = orientation.y();
      message.pose.orientation.z = orientation.z();
      message.pose.orientation.w = orientation.w();
      frontier_goal_publisher_.publish(message);
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "local mapping frontier projection failed: %s",
                        error.what());
    }
  }

  void publishTimerCallback(const ros::TimerEvent&) {
    const ros::Time stamp = ros::Time::now();
    const double now = stamp.toSec();
    last_health_result_ = health_monitor_->evaluate(now);
    const double fusion_timeout =
        std::min(config_.depth_timeout, config_.odom_timeout);
    const bool fusion_fresh =
        has_successful_fusion_ && finite(now) &&
        finite(last_successful_fusion_time_) &&
        now >= last_successful_fusion_time_ &&
        now - last_successful_fusion_time_ <= fusion_timeout;

    search_msgs::PerceptionHealth health;
    health.header.stamp = stamp;
    health.header.frame_id = kMapFrame;
    health.depth_healthy = last_health_result_.depth_healthy;
    health.odom_healthy = last_health_result_.odom_healthy;
    health.synchronized = last_health_result_.synchronized;
    health.map_healthy =
        last_health_result_.healthy && last_fusion_valid_ && fusion_fresh;
    health.valid_depth_ratio = last_health_result_.valid_depth_ratio;
    health.dropped_frames = health_monitor_->droppedFrames();
    health.fault_code = last_health_result_.fault_code;
    if (last_health_result_.healthy && last_fusion_valid_ &&
        has_successful_fusion_ && !fusion_fresh) {
      health.fault_code = "SYNC_ERROR";
    } else if (!last_fusion_valid_ &&
               (last_health_result_.healthy ||
                concreteMappingFault(last_mapping_fault_))) {
      health.fault_code = last_mapping_fault_;
    }
    health_publisher_.publish(health);

    try {
      static_cloud_publisher_.publish(
          cloudMessage(voxel_map_->staticOccupiedPoints(now), stamp));
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "local mapping static cloud failed: %s",
                        error.what());
      static_cloud_publisher_.publish(cloudMessage({}, stamp));
    }
    try {
      dynamic_cloud_publisher_.publish(
          cloudMessage(voxel_map_->dynamicOccupiedPoints(now), stamp));
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "local mapping dynamic cloud failed: %s",
                        error.what());
      dynamic_cloud_publisher_.publish(cloudMessage({}, stamp));
    }
    clearance_publisher_.publish(clearanceMessage(stamp));
    publishFrontier(stamp);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  NodeConfig config_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<HealthMonitor> health_monitor_;
  std::unique_ptr<SemanticDepthFilter> semantic_filter_;
  std::unique_ptr<VoxelMap> voxel_map_;
  std::unique_ptr<FrontierSelector> frontier_selector_;

  message_filters::Subscriber<sensor_msgs::Image> depth_subscriber_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_subscriber_;
  std::unique_ptr<Synchronizer> synchronizer_;
  ros::Subscriber boxes_subscriber_;
  ros::Subscriber health_depth_subscriber_;
  ros::Subscriber health_odom_subscriber_;

  ros::Publisher planner_depth_publisher_;
  ros::Publisher static_cloud_publisher_;
  ros::Publisher dynamic_cloud_publisher_;
  ros::Publisher health_publisher_;
  ros::Publisher clearance_publisher_;
  ros::Publisher frontier_goal_publisher_;
  ros::Timer publish_timer_;

  HealthResult last_health_result_;
  bool has_reliable_odom_;
  Vec3 latest_odom_position_{0.0, 0.0, 0.0};
  bool last_fusion_valid_;
  bool has_successful_fusion_;
  double last_successful_fusion_time_;
  std::string last_mapping_fault_;
  darknet_ros_msgs::BoundingBoxesConstPtr cached_boxes_;
  ros::Time cached_boxes_stamp_;
  bool cached_boxes_used_;
  std::deque<DepthMessageKey> recent_depth_drop_keys_;
};

}  // namespace local_mapping

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_mapping_node");
  try {
    local_mapping::LocalMappingNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("local mapping node initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
