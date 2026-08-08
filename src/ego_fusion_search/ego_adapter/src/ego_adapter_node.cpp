#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <ros/this_node.h>
#include <search_msgs/LocalClearance.h>
#include <search_msgs/PerceptionHealth.h>
#include <search_msgs/ValidateTrajectory.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <traj_utils/Bspline.h>

#include "ego_adapter/bspline_sampler.h"
#include "ego_adapter/command_policy.h"

namespace ego_adapter {
namespace {

constexpr double kGenerationStartSlack = 0.05;
constexpr double kConnectorSampleGap = 0.10;
constexpr std::size_t kMaximumTrajectorySamples = 10000u;

template <typename T>
T parameter(ros::NodeHandle& node, const std::string& name,
            const T& default_value) {
  T value = default_value;
  node.param(name, value, default_value);
  return value;
}

bool finite(double value) { return std::isfinite(value); }

bool finite(const Vec3& value) {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

bool finite(const geometry_msgs::Quaternion& value) {
  return finite(value.x) && finite(value.y) && finite(value.z) &&
         finite(value.w);
}

double distance(const Vec3& first, const Vec3& second) {
  return std::hypot(std::hypot(second.x - first.x, second.y - first.y),
                    second.z - first.z);
}

Vec3 point(const geometry_msgs::Point& value) {
  return Vec3{value.x, value.y, value.z};
}

geometry_msgs::Point messagePoint(const Vec3& value) {
  geometry_msgs::Point result;
  result.x = value.x;
  result.y = value.y;
  result.z = value.z;
  return result;
}

double yaw(const geometry_msgs::Quaternion& orientation) {
  const double sin_yaw =
      2.0 * (orientation.w * orientation.z +
             orientation.x * orientation.y);
  const double cos_yaw =
      1.0 - 2.0 * (orientation.y * orientation.y +
                   orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}

struct NodeConfig {
  double control_rate{20.0};
  double health_timeout{0.50};
  double odom_timeout{0.50};
  double map_response_timeout{0.20};
  double trajectory_revalidate_period{0.10};
  double trajectory_position_match_tolerance{0.30};
  double trajectory_velocity_match_tolerance{0.50};
  double trajectory_deviation_tolerance{0.50};
  double trajectory_sample_dt{0.10};
  std::string position_command_topic;
  std::string bspline_topic;
  std::string odom_topic;
  std::string health_topic;
  std::string clearance_topic;
  std::string generation_topic;
  std::string mux_selected_topic;
  std::string validate_trajectory_service;
  std::string command_topic;
  std::string status_topic;
  std::string navigator_topic;
};

struct ValidationJob {
  std::uint64_t sequence{0u};
  std::uint64_t generation{0u};
  std::int64_t trajectory_id{0};
  ros::Time request_stamp;
  std::vector<geometry_msgs::Point> samples;
};

NodeConfig loadConfig(ros::NodeHandle& node, PolicyLimits* limits) {
  NodeConfig config;
  config.control_rate = parameter(node, "control_rate", 20.0);
  limits->command_timeout = parameter(node, "command_timeout", 0.20);
  config.health_timeout = parameter(node, "health_timeout", 0.50);
  config.odom_timeout = parameter(node, "odom_timeout", 0.50);
  config.map_response_timeout =
      parameter(node, "map_response_timeout", 0.20);
  config.trajectory_revalidate_period =
      parameter(node, "trajectory_revalidate_period", 0.10);
  config.trajectory_position_match_tolerance =
      parameter(node, "trajectory_position_match_tolerance", 0.30);
  config.trajectory_velocity_match_tolerance =
      parameter(node, "trajectory_velocity_match_tolerance", 0.50);
  config.trajectory_deviation_tolerance =
      parameter(node, "trajectory_deviation_tolerance", 0.50);
  config.trajectory_sample_dt =
      parameter(node, "trajectory_sample_dt", 0.10);
  limits->max_search_altitude =
      parameter(node, "max_search_altitude", 4.0);
  limits->position_gain = parameter(node, "position_gain", 0.60);
  limits->yaw_align_threshold =
      parameter(node, "yaw_align_threshold", 0.5235987756);
  limits->max_forward_speed =
      parameter(node, "max_forward_speed", 1.5);
  limits->max_lateral_speed =
      parameter(node, "max_lateral_speed", 0.25);
  limits->max_reverse_speed =
      parameter(node, "max_reverse_speed", 0.10);
  limits->max_vertical_speed =
      parameter(node, "max_vertical_speed", 0.50);
  limits->max_yaw_rate = parameter(node, "max_yaw_rate", 0.80);
  limits->braking_clearance =
      parameter(node, "braking_clearance", 1.50);
  limits->emergency_clearance =
      parameter(node, "emergency_clearance", 0.80);

  config.position_command_topic = parameter(
      node, "position_command_topic", std::string("/typhoon_h480_0/ego/position_cmd"));
  config.bspline_topic = parameter(
      node, "bspline_topic", std::string("/typhoon_h480_0/ego/broadcast_bspline"));
  config.odom_topic = parameter(
      node, "odom_topic", std::string("/typhoon_h480_0/global_odom"));
  config.health_topic = parameter(
      node, "health_topic", std::string("/typhoon_h480_0/local_mapping/health"));
  config.clearance_topic = parameter(
      node, "clearance_topic", std::string("/typhoon_h480_0/local_mapping/clearance"));
  config.generation_topic = parameter(
      node, "generation_topic", std::string("/typhoon_h480_0/navigation/task_generation"));
  config.mux_selected_topic = parameter(
      node, "mux_selected_topic", std::string("/typhoon_h480_0/pose_cmd_mux/selected"));
  config.validate_trajectory_service = parameter(
      node, "validate_trajectory_service",
      std::string("/typhoon_h480_0/local_mapping/validate_trajectory"));
  config.command_topic = parameter(
      node, "command_topic", std::string("/typhoon_h480_0/mux_inputs/navigator/cmd_vel"));
  config.status_topic = parameter(
      node, "status_topic", std::string("/typhoon_h480_0/ego_adapter/status"));
  config.navigator_topic = parameter(
      node, "navigator_topic", std::string("/typhoon_h480_0/mux_inputs/navigator/cmd_vel"));

  const bool finite_positive =
      finite(config.control_rate) && config.control_rate > 0.0 &&
      finite(config.health_timeout) && config.health_timeout > 0.0 &&
      finite(config.odom_timeout) && config.odom_timeout > 0.0 &&
      finite(config.map_response_timeout) &&
      config.map_response_timeout > 0.0 &&
      finite(config.trajectory_revalidate_period) &&
      config.trajectory_revalidate_period > 0.0 &&
      finite(config.trajectory_position_match_tolerance) &&
      config.trajectory_position_match_tolerance >= 0.0 &&
      finite(config.trajectory_velocity_match_tolerance) &&
      config.trajectory_velocity_match_tolerance >= 0.0 &&
      finite(config.trajectory_deviation_tolerance) &&
      config.trajectory_deviation_tolerance >= 0.0 &&
      finite(config.trajectory_sample_dt) &&
      config.trajectory_sample_dt > 0.0;
  const bool topics_present =
      !config.position_command_topic.empty() && !config.bspline_topic.empty() &&
      !config.odom_topic.empty() && !config.health_topic.empty() &&
      !config.clearance_topic.empty() && !config.generation_topic.empty() &&
      !config.mux_selected_topic.empty() &&
      !config.validate_trajectory_service.empty() &&
      !config.command_topic.empty() && !config.status_topic.empty() &&
      !config.navigator_topic.empty();
  if (!finite_positive || !topics_present) {
    throw std::invalid_argument("invalid ego_adapter node configuration");
  }
  return config;
}

}  // namespace

class EgoAdapterNode {
 public:
  EgoAdapterNode() : private_node_("~") {
    PolicyLimits limits;
    config_ = loadConfig(private_node_, &limits);
    policy_.reset(new CommandPolicy(limits));

    command_publisher_ =
        node_.advertise<geometry_msgs::Twist>(config_.command_topic, 1);
    status_publisher_ =
        node_.advertise<std_msgs::String>(config_.status_topic, 1, true);
    validation_client_ = node_.serviceClient<search_msgs::ValidateTrajectory>(
        config_.validate_trajectory_service, false);
    position_subscriber_ = node_.subscribe(
        config_.position_command_topic, 1,
        &EgoAdapterNode::positionCommandCallback, this);
    spline_subscriber_ = node_.subscribe(
        config_.bspline_topic, 1, &EgoAdapterNode::splineCallback, this);
    odom_subscriber_ = node_.subscribe(
        config_.odom_topic, 1, &EgoAdapterNode::odomCallback, this);
    health_subscriber_ = node_.subscribe(
        config_.health_topic, 1, &EgoAdapterNode::healthCallback, this);
    clearance_subscriber_ = node_.subscribe(
        config_.clearance_topic, 1, &EgoAdapterNode::clearanceCallback, this);
    generation_subscriber_ = node_.subscribe(
        config_.generation_topic, 1, &EgoAdapterNode::generationCallback,
        this);
    mux_subscriber_ = node_.subscribe(
        config_.mux_selected_topic, 1, &EgoAdapterNode::muxCallback, this);
    refreshPublisherOwnership();
    publisher_guard_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0),
        &EgoAdapterNode::publisherGuardCallback, this);
    timer_ = node_.createTimer(ros::Duration(1.0 / config_.control_rate),
                               &EgoAdapterNode::timerCallback, this);
    validation_thread_ = std::thread(&EgoAdapterNode::validationLoop, this);
  }

  ~EgoAdapterNode() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutting_down_ = true;
      validation_queued_ = false;
    }
    validation_condition_.notify_all();
    validation_client_.shutdown();
    if (validation_thread_.joinable()) {
      validation_thread_.join();
    }
  }

 private:
  bool ownsNavigatorTopicExclusively() const {
    XmlRpc::XmlRpcValue request;
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;
    request[0] = ros::this_node::getName();
    if (!ros::master::execute("getSystemState", request, response, payload,
                              false) ||
        payload.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        payload.size() < 1) {
      return false;
    }

    const std::string expected_topic = node_.resolveName(config_.command_topic);
    const std::string expected_node = ros::this_node::getName();
    const XmlRpc::XmlRpcValue& publishers = payload[0];
    if (publishers.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      return false;
    }
    for (int index = 0; index < publishers.size(); ++index) {
      const XmlRpc::XmlRpcValue& entry = publishers[index];
      if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray ||
          entry.size() != 2 ||
          entry[0].getType() != XmlRpc::XmlRpcValue::TypeString ||
          static_cast<std::string>(entry[0]) != expected_topic ||
          entry[1].getType() != XmlRpc::XmlRpcValue::TypeArray) {
        continue;
      }
      const XmlRpc::XmlRpcValue& nodes = entry[1];
      return nodes.size() == 1 &&
             nodes[0].getType() == XmlRpc::XmlRpcValue::TypeString &&
             static_cast<std::string>(nodes[0]) == expected_node;
    }
    return false;
  }

  void refreshPublisherOwnership() {
    const bool exclusive = ownsNavigatorTopicExclusively();
    std::lock_guard<std::mutex> lock(mutex_);
    navigator_publisher_exclusive_ = exclusive;
    if (!exclusive) {
      invalidateBindingLocked("NAVIGATOR_PUBLISHER_CONFLICT");
    }
  }

  void publisherGuardCallback(const ros::WallTimerEvent&) {
    refreshPublisherOwnership();
  }

  void invalidateBindingLocked(const std::string& status) {
    spline_.reset();
    has_position_command_ = false;
    trajectory_valid_ = false;
    validation_pending_ = false;
    validation_queued_ = false;
    bound_generation_ = 0u;
    bound_trajectory_id_ = 0;
    ++validation_sequence_;
    status_ = status;
  }

  void invalidateValidationLocked(const std::string& status) {
    trajectory_valid_ = false;
    validation_pending_ = false;
    validation_queued_ = false;
    bound_generation_ = 0u;
    bound_trajectory_id_ = 0;
    ++validation_sequence_;
    status_ = status;
  }

  void generationCallback(const std_msgs::UInt64ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time now = ros::Time::now();
    if (has_generation_ && message->data <= active_generation_) {
      if (message->data < active_generation_) {
        invalidateBindingLocked("STALE_GENERATION");
      }
      return;
    }
    active_generation_ = message->data;
    has_generation_ = true;
    generation_started_at_ = now;
    invalidateBindingLocked("GENERATION_CHANGED");
  }

  void muxCallback(const std_msgs::StringConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool was_navigator = mux_is_navigator_;
    mux_is_navigator_ = message->data == config_.navigator_topic;
    has_mux_selection_ = true;
    if (was_navigator && !mux_is_navigator_) {
      invalidateBindingLocked("MUX_NOT_NAVIGATOR");
    }
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_ = *message;
    has_odom_ = true;
  }

  void healthCallback(const search_msgs::PerceptionHealthConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    health_ = *message;
    has_health_ = true;
  }

  void clearanceCallback(const search_msgs::LocalClearanceConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearance_ = *message;
    has_clearance_ = true;
  }

  void splineCallback(const traj_utils::BsplineConstPtr& message) {
    BsplineData data;
    data.order = message->order;
    data.traj_id = message->traj_id;
    data.start_time = message->start_time.toSec();
    data.knots.assign(message->knots.begin(), message->knots.end());
    data.control_points.reserve(message->pos_pts.size());
    for (const geometry_msgs::Point& control_point : message->pos_pts) {
      data.control_points.push_back(point(control_point));
    }

    std::shared_ptr<BsplineSampler> candidate;
    try {
      candidate = std::make_shared<BsplineSampler>(data);
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> lock(mutex_);
      invalidateBindingLocked("INVALID_BSPLINE");
      ROS_WARN_THROTTLE(1.0, "ego_adapter rejected B-spline: %s",
                        error.what());
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_generation_ || !has_mux_selection_ || !mux_is_navigator_) {
      invalidateBindingLocked("NOT_READY");
      return;
    }
    if (has_max_seen_trajectory_id_ &&
        message->traj_id <= max_seen_trajectory_id_) {
      invalidateBindingLocked("STALE_TRAJECTORY_ID");
      return;
    }
    max_seen_trajectory_id_ = message->traj_id;
    has_max_seen_trajectory_id_ = true;
    if (message->start_time <
        generation_started_at_ - ros::Duration(kGenerationStartSlack)) {
      invalidateBindingLocked("STALE_TRAJECTORY_START");
      return;
    }
    const ros::Time now = ros::Time::now();
    if (!finite(candidate->endTime()) || candidate->endTime() <= now.toSec()) {
      invalidateBindingLocked("EXPIRED_BSPLINE");
      return;
    }

    invalidateBindingLocked("VALIDATION_PENDING");
    spline_ = std::move(candidate);
    spline_received_at_ = now;
    active_trajectory_id_ = message->traj_id;
  }

  void positionCommandCallback(
      const quadrotor_msgs::PositionCommandConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    has_position_command_ = false;
    if (!spline_) {
      status_ = "NO_BOUND_SPLINE";
      return;
    }
    const double stamp = message->header.stamp.toSec();
    if (!finite(stamp) || message->header.stamp <= spline_received_at_) {
      status_ = "STALE_POSITION_COMMAND";
      return;
    }
    try {
      const BsplineState expected = spline_->evaluate(stamp);
      const Vec3 actual_position = point(message->position);
      const Vec3 actual_velocity{message->velocity.x, message->velocity.y,
                                 message->velocity.z};
      if (distance(expected.position, actual_position) >
              config_.trajectory_position_match_tolerance ||
          distance(expected.velocity, actual_velocity) >
              config_.trajectory_velocity_match_tolerance) {
        status_ = "POSITION_COMMAND_MISMATCH";
        return;
      }
    } catch (const std::exception&) {
      status_ = "POSITION_COMMAND_OUT_OF_RANGE";
      return;
    }
    position_command_ = *message;
    has_position_command_ = true;
  }

  bool inputsFreshLocked(const ros::Time& now, std::string* fault) const {
    if (!navigator_publisher_exclusive_) {
      *fault = "NAVIGATOR_PUBLISHER_CONFLICT";
      return false;
    }
    if (!has_odom_) {
      *fault = "ODOM_MISSING";
      return false;
    }
    const double odom_age = (now - odom_.header.stamp).toSec();
    if (!finite(odom_age) || odom_age < 0.0 ||
        odom_age > config_.odom_timeout) {
      *fault = "ODOM_STALE";
      return false;
    }
    if (!finite(point(odom_.pose.pose.position)) ||
        !finite(odom_.pose.pose.orientation)) {
      *fault = "INVALID_ODOM";
      return false;
    }
    if (!has_health_) {
      *fault = "HEALTH_MISSING";
      return false;
    }
    const double health_age = (now - health_.header.stamp).toSec();
    if (!finite(health_age) || health_age < 0.0 ||
        health_age > config_.health_timeout) {
      *fault = "HEALTH_STALE";
      return false;
    }
    if (!health_.depth_healthy || !health_.odom_healthy ||
        !health_.synchronized || !health_.map_healthy) {
      *fault = health_.fault_code.empty() ? "MAP_UNHEALTHY"
                                          : health_.fault_code;
      return false;
    }
    if (!has_clearance_) {
      *fault = "CLEARANCE_MISSING";
      return false;
    }
    const double clearance_age = (now - clearance_.header.stamp).toSec();
    if (!finite(clearance_age) || clearance_age < 0.0 ||
        clearance_age > config_.health_timeout) {
      *fault = "CLEARANCE_STALE";
      return false;
    }
    if (!has_mux_selection_ || !mux_is_navigator_) {
      *fault = "MUX_NOT_NAVIGATOR";
      return false;
    }
    if (!has_generation_) {
      *fault = "GENERATION_MISSING";
      return false;
    }
    return true;
  }

  std::vector<geometry_msgs::Point> validationSamplesLocked(
      const ros::Time& now) const {
    if (!spline_) {
      throw std::logic_error("validation requested without B-spline");
    }
    const Vec3 current_position = point(odom_.pose.pose.position);
    if (!finite(current_position)) {
      throw std::invalid_argument("non-finite current position");
    }
    const double sample_time =
        std::max(spline_->startTime(), now.toSec());
    if (sample_time > spline_->endTime()) {
      throw std::out_of_range("B-spline has expired");
    }
    const BsplineState first_state = spline_->evaluate(sample_time);
    if (!finite(first_state.position)) {
      throw std::invalid_argument("non-finite B-spline sample");
    }

    std::vector<geometry_msgs::Point> samples;
    samples.push_back(messagePoint(current_position));
    const double connector_distance =
        distance(current_position, first_state.position);
    const double connector_steps_value =
        std::ceil(connector_distance / kConnectorSampleGap);
    if (!finite(connector_steps_value) || connector_steps_value < 0.0 ||
        connector_steps_value >
            static_cast<double>(kMaximumTrajectorySamples - 2u)) {
      throw std::length_error("invalid B-spline connector length");
    }
    const std::size_t connector_steps =
        static_cast<std::size_t>(connector_steps_value);
    for (std::size_t index = 1u; index <= connector_steps; ++index) {
      const double ratio = static_cast<double>(index) /
                           static_cast<double>(connector_steps);
      samples.push_back(messagePoint(Vec3{
          current_position.x +
              ratio * (first_state.position.x - current_position.x),
          current_position.y +
              ratio * (first_state.position.y - current_position.y),
          current_position.z +
              ratio * (first_state.position.z - current_position.z)}));
    }
    if (connector_steps == 0u) {
      samples.push_back(messagePoint(first_state.position));
    }

    double time = sample_time + config_.trajectory_sample_dt;
    while (time < spline_->endTime()) {
      if (samples.size() >= kMaximumTrajectorySamples) {
        throw std::length_error("B-spline has too many validation samples");
      }
      samples.push_back(messagePoint(spline_->evaluate(time).position));
      time += config_.trajectory_sample_dt;
    }
    if (samples.size() >= kMaximumTrajectorySamples) {
      throw std::length_error("B-spline has too many validation samples");
    }
    samples.push_back(messagePoint(
        spline_->evaluate(spline_->endTime()).position));
    return samples;
  }

  void queueValidationLocked(const ros::Time& now) {
    ValidationJob job;
    try {
      job.sequence = ++validation_sequence_;
      job.generation = active_generation_;
      job.trajectory_id = active_trajectory_id_;
      job.request_stamp = now;
      job.samples = validationSamplesLocked(now);
    } catch (const std::exception& error) {
      invalidateValidationLocked("INVALID_TRAJECTORY_SAMPLES");
      ROS_WARN_THROTTLE(1.0, "ego_adapter validation sampling failed: %s",
                        error.what());
      return;
    }
    trajectory_valid_ = false;
    validation_pending_ = true;
    validation_queued_ = true;
    last_validation_request_ = now;
    pending_validation_ = std::move(job);
    status_ = "VALIDATION_PENDING";
    validation_condition_.notify_one();
  }

  void validationLoop() {
    while (ros::ok()) {
      ValidationJob job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        validation_condition_.wait(lock, [this] {
          return shutting_down_ || validation_queued_;
        });
        if (shutting_down_) {
          return;
        }
        job = pending_validation_;
        validation_queued_ = false;
      }

      search_msgs::ValidateTrajectory service;
      service.request.header.stamp = job.request_stamp;
      service.request.header.frame_id = "map";
      service.request.task_generation = job.generation;
      service.request.samples = job.samples;
      const ros::WallTime started = ros::WallTime::now();
      const bool called = validation_client_.call(service);
      const double elapsed = (ros::WallTime::now() - started).toSec();

      std::lock_guard<std::mutex> lock(mutex_);
      if (shutting_down_ || job.sequence != validation_sequence_ ||
          !spline_ || job.generation != active_generation_ ||
          job.trajectory_id != active_trajectory_id_) {
        continue;
      }
      validation_pending_ = false;
      if (!called) {
        status_ = "TRAJECTORY_SERVICE_UNAVAILABLE";
        trajectory_valid_ = false;
        continue;
      }
      if (!finite(elapsed) || elapsed > config_.map_response_timeout) {
        status_ = "TRAJECTORY_VALIDATION_TIMEOUT";
        trajectory_valid_ = false;
        continue;
      }
      if (service.response.task_generation != job.generation ||
          (!last_validated_map_stamp_.isZero() &&
           service.response.map_stamp < last_validated_map_stamp_)) {
        status_ = "STALE_TRAJECTORY_RESPONSE";
        trajectory_valid_ = false;
        continue;
      }
      if (!service.response.valid) {
        status_ = "TRAJECTORY_REJECTED";
        trajectory_valid_ = false;
        continue;
      }
      trajectory_valid_ = true;
      bound_generation_ = job.generation;
      bound_trajectory_id_ = job.trajectory_id;
      last_validated_map_stamp_ = service.response.map_stamp;
      status_ = "TRAJECTORY_VALID";
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    geometry_msgs::Twist output;
    std_msgs::String status_message;
    const ros::Time now = ros::Time::now();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!last_timer_time_.isZero() && now < last_timer_time_) {
        invalidateBindingLocked("TIME_ROLLBACK");
      }
      last_timer_time_ = now;

      std::string input_fault;
      const bool inputs_fresh = inputsFreshLocked(now, &input_fault);
      if (inputs_fresh && spline_) {
        if (now.toSec() > spline_->endTime()) {
          invalidateBindingLocked("TRAJECTORY_COMPLETE");
        } else if (!validation_pending_) {
          bool deviation_requires_validation = false;
          try {
            const double sample_time =
                std::max(now.toSec(), spline_->startTime());
            deviation_requires_validation =
                distance(point(odom_.pose.pose.position),
                         spline_->evaluate(sample_time).position) >
                config_.trajectory_deviation_tolerance;
          } catch (const std::exception&) {
            deviation_requires_validation = true;
          }
          const bool validation_due =
              last_validation_request_.isZero() ||
              (now - last_validation_request_).toSec() >=
                  config_.trajectory_revalidate_period;
          if (validation_due || deviation_requires_validation) {
            queueValidationLocked(now);
          }
        }
      }

      if (!inputs_fresh) {
        status_ = input_fault;
      } else if (spline_ && has_position_command_) {
        PolicyInput input;
        input.now = now.toSec();
        input.command_stamp = position_command_.header.stamp.toSec();
        input.bound_generation = bound_generation_;
        input.active_generation = active_generation_;
        input.map_healthy = health_.map_healthy && health_.depth_healthy &&
                            health_.odom_healthy && health_.synchronized;
        input.mux_is_navigator = mux_is_navigator_;
        input.trajectory_valid =
            trajectory_valid_ && bound_trajectory_id_ == active_trajectory_id_;
        input.current_position = point(odom_.pose.pose.position);
        input.desired_position = point(position_command_.position);
        input.current_yaw = yaw(odom_.pose.pose.orientation);
        input.desired_yaw = position_command_.yaw;
        input.desired_yaw_rate = position_command_.yaw_dot;
        input.world_velocity = Vec3{position_command_.velocity.x,
                                    position_command_.velocity.y,
                                    position_command_.velocity.z};
        input.clearance = DirectionalClearance{
            AxisClearance{clearance_.forward_known != 0u,
                          clearance_.forward_m},
            AxisClearance{clearance_.backward_known != 0u,
                          clearance_.backward_m},
            AxisClearance{clearance_.left_known != 0u, clearance_.left_m},
            AxisClearance{clearance_.right_known != 0u, clearance_.right_m},
            AxisClearance{clearance_.upward_known != 0u, clearance_.upward_m},
            AxisClearance{clearance_.downward_known != 0u,
                          clearance_.downward_m}};
        const PolicyOutput result = policy_->evaluate(input);
        if (result.accepted) {
          output.linear.x = result.forward;
          output.linear.y = result.left;
          output.linear.z = result.up;
          output.angular.z = result.yaw_rate;
          status_ = "EXECUTING:" + std::to_string(bound_generation_) + ":" +
                    std::to_string(bound_trajectory_id_);
        } else if (!validation_pending_ && status_ != "TRAJECTORY_REJECTED") {
          status_ = result.fault_code;
        }
      } else if (inputs_fresh && status_.empty()) {
        status_ = "NOT_READY";
      }
      status_message.data = status_.empty() ? "NOT_READY" : status_;
    }

    command_publisher_.publish(output);
    status_publisher_.publish(status_message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  NodeConfig config_;
  std::unique_ptr<CommandPolicy> policy_;
  ros::Publisher command_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber position_subscriber_;
  ros::Subscriber spline_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber health_subscriber_;
  ros::Subscriber clearance_subscriber_;
  ros::Subscriber generation_subscriber_;
  ros::Subscriber mux_subscriber_;
  ros::ServiceClient validation_client_;
  ros::Timer timer_;
  ros::WallTimer publisher_guard_timer_;

  std::mutex mutex_;
  std::condition_variable validation_condition_;
  std::thread validation_thread_;
  bool shutting_down_{false};
  bool validation_queued_{false};
  bool validation_pending_{false};
  ValidationJob pending_validation_;
  std::uint64_t validation_sequence_{0u};

  bool has_generation_{false};
  std::uint64_t active_generation_{0u};
  ros::Time generation_started_at_;
  bool has_max_seen_trajectory_id_{false};
  std::int64_t max_seen_trajectory_id_{std::numeric_limits<std::int64_t>::min()};
  std::int64_t active_trajectory_id_{0};
  std::int64_t bound_trajectory_id_{0};
  std::uint64_t bound_generation_{0u};

  bool has_mux_selection_{false};
  bool navigator_publisher_exclusive_{false};
  bool mux_is_navigator_{false};
  bool has_odom_{false};
  bool has_health_{false};
  bool has_clearance_{false};
  bool has_position_command_{false};
  nav_msgs::Odometry odom_;
  search_msgs::PerceptionHealth health_;
  search_msgs::LocalClearance clearance_;
  quadrotor_msgs::PositionCommand position_command_;
  std::shared_ptr<BsplineSampler> spline_;
  ros::Time spline_received_at_;

  bool trajectory_valid_{false};
  ros::Time last_validation_request_;
  ros::Time last_validated_map_stamp_;
  ros::Time last_timer_time_;
  std::string status_{"NOT_READY"};
};

}  // namespace ego_adapter

int main(int argc, char** argv) {
  ros::init(argc, argv, "ego_adapter");
  try {
    ego_adapter::EgoAdapterNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("ego_adapter startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
