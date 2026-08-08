#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <search_msgs/PerceptionHealth.h>
#include <search_msgs/ValidateTrajectory.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>

#include "search_coordinator/coordinator.h"

namespace search_coordinator {
namespace {

template <typename T>
T parameter(ros::NodeHandle* node, const std::string& name,
            const T& default_value) {
  T value = default_value;
  node->param(name, value, default_value);
  return value;
}

bool finite(double value) { return std::isfinite(value); }

bool startsWith(const std::string& value, const std::string& prefix) {
  return value.compare(0u, prefix.size(), prefix) == 0;
}

std::string stateName(State state) {
  switch (state) {
    case State::WAIT_READY:
      return "WAIT_READY";
    case State::OBSERVING:
      return "OBSERVING";
    case State::PLANNING:
      return "PLANNING";
    case State::EXECUTING:
      return "EXECUTING";
    case State::HOLD:
      return "HOLD";
    case State::CANDIDATE_HOLD:
      return "CANDIDATE_HOLD";
    case State::TRACKING_EXTERNAL:
      return "TRACKING_EXTERNAL";
    case State::REJOINING:
      return "REJOINING";
  }
  return "HOLD";
}

struct NodeConfig {
  double update_rate{20.0};
  double input_timeout{0.5};
  double validation_sample_spacing{0.1};
  double generation_settle_delay{0.05};
  CoordinatorConfig coordinator;
  std::string high_level_goal_topic;
  std::string odom_topic;
  std::string health_topic;
  std::string frontier_goal_topic;
  std::string mission_active_topic;
  std::string takeoff_complete_topic;
  std::string tracking_phase_topic;
  std::string mux_selected_topic;
  std::string adapter_status_topic;
  std::string validate_trajectory_service;
  std::string ego_goal_topic;
  std::string generation_topic;
  std::string status_topic;
};

NodeConfig loadConfig(ros::NodeHandle* node) {
  NodeConfig config;
  config.update_rate = parameter(node, "update_rate", 20.0);
  config.input_timeout = parameter(node, "input_timeout", 0.5);
  config.validation_sample_spacing =
      parameter(node, "validation_sample_spacing", 0.1);
  config.generation_settle_delay =
      parameter(node, "generation_settle_delay", 0.05);

  config.coordinator.goal_position_epsilon =
      parameter(node, "goal_position_epsilon", 0.20);
  config.coordinator.goal_altitude_epsilon =
      parameter(node, "goal_altitude_epsilon", 0.10);
  config.coordinator.planning_timeout =
      parameter(node, "planning_timeout", 1.0);
  config.coordinator.local_arrival_tolerance =
      parameter(node, "local_arrival_tolerance", 1.0);
  config.coordinator.max_local_goal_distance =
      parameter(node, "max_local_goal_distance", 8.0);
  config.coordinator.min_search_altitude =
      parameter(node, "min_search_altitude", 2.0);
  config.coordinator.max_search_altitude =
      parameter(node, "max_search_altitude", 4.0);
  config.coordinator.frontier_max_age =
      parameter(node, "frontier_max_age", 0.5);
  config.coordinator.navigator_topic = parameter(
      node, "navigator_topic",
      std::string("/typhoon_h480_0/mux_inputs/navigator/cmd_vel"));

  config.high_level_goal_topic = parameter(
      node, "high_level_goal_topic",
      std::string("/typhoon_h480_0/move_base_simple/goal"));
  config.odom_topic = parameter(
      node, "odom_topic", std::string("/typhoon_h480_0/global_odom"));
  config.health_topic = parameter(
      node, "health_topic",
      std::string("/typhoon_h480_0/local_mapping/health"));
  config.frontier_goal_topic = parameter(
      node, "frontier_goal_topic",
      std::string("/typhoon_h480_0/local_mapping/frontier_goal"));
  config.mission_active_topic = parameter(
      node, "mission_active_topic",
      std::string("/typhoon_h480_0/mission/active"));
  config.takeoff_complete_topic = parameter(
      node, "takeoff_complete_topic", std::string("/swarm/takeoff_complete"));
  config.tracking_phase_topic = parameter(
      node, "tracking_phase_topic",
      std::string("/typhoon_h480_0/tracking/phase"));
  config.mux_selected_topic = parameter(
      node, "mux_selected_topic",
      std::string("/typhoon_h480_0/pose_cmd_mux/selected"));
  config.adapter_status_topic = parameter(
      node, "adapter_status_topic",
      std::string("/typhoon_h480_0/ego_adapter/status"));
  config.validate_trajectory_service = parameter(
      node, "validate_trajectory_service",
      std::string("/typhoon_h480_0/local_mapping/validate_trajectory"));
  config.ego_goal_topic = parameter(
      node, "ego_goal_topic", std::string("/typhoon_h480_0/ego/goal"));
  config.generation_topic = parameter(
      node, "generation_topic",
      std::string("/typhoon_h480_0/navigation/task_generation"));
  config.status_topic = parameter(
      node, "status_topic",
      std::string("/typhoon_h480_0/search_coordinator/status"));

  const bool numeric_valid =
      finite(config.update_rate) && config.update_rate > 0.0 &&
      finite(config.input_timeout) && config.input_timeout > 0.0 &&
      finite(config.validation_sample_spacing) &&
      config.validation_sample_spacing > 0.0 &&
      config.validation_sample_spacing <= 0.20 &&
      finite(config.generation_settle_delay) &&
      config.generation_settle_delay >= 0.0;
  const bool topics_valid =
      !config.high_level_goal_topic.empty() && !config.odom_topic.empty() &&
      !config.health_topic.empty() && !config.frontier_goal_topic.empty() &&
      !config.mission_active_topic.empty() &&
      !config.takeoff_complete_topic.empty() &&
      !config.tracking_phase_topic.empty() &&
      !config.mux_selected_topic.empty() &&
      !config.adapter_status_topic.empty() &&
      !config.validate_trajectory_service.empty() &&
      !config.ego_goal_topic.empty() && !config.generation_topic.empty() &&
      !config.status_topic.empty();
  if (!numeric_valid || !topics_valid) {
    throw std::invalid_argument("invalid search coordinator node configuration");
  }
  return config;
}

StampedGoal stampedGoal(const geometry_msgs::PoseStamped& message,
                        const ros::Time& received_at) {
  StampedGoal goal;
  goal.available = true;
  goal.frame_id = message.header.frame_id;
  goal.position = Vec3{message.pose.position.x, message.pose.position.y,
                       message.pose.position.z};
  goal.stamp =
      (message.header.stamp.isZero() ? received_at : message.header.stamp)
          .toSec();
  return goal;
}

}  // namespace

class SearchCoordinatorNode {
 public:
  SearchCoordinatorNode() : private_node_("~") {
    config_ = loadConfig(&private_node_);
    coordinator_.reset(new Coordinator(config_.coordinator));

    generation_publisher_ =
        node_.advertise<std_msgs::UInt64>(config_.generation_topic, 1, true);
    goal_publisher_ =
        node_.advertise<geometry_msgs::PoseStamped>(config_.ego_goal_topic, 1);
    status_publisher_ =
        node_.advertise<std_msgs::String>(config_.status_topic, 1, true);

    high_level_goal_subscriber_ = node_.subscribe(
        config_.high_level_goal_topic, 1,
        &SearchCoordinatorNode::highLevelGoalCallback, this);
    odom_subscriber_ = node_.subscribe(
        config_.odom_topic, 1, &SearchCoordinatorNode::odomCallback, this);
    health_subscriber_ = node_.subscribe(
        config_.health_topic, 1, &SearchCoordinatorNode::healthCallback, this);
    frontier_subscriber_ = node_.subscribe(
        config_.frontier_goal_topic, 1,
        &SearchCoordinatorNode::frontierCallback, this);
    mission_subscriber_ = node_.subscribe(
        config_.mission_active_topic, 1,
        &SearchCoordinatorNode::missionCallback, this);
    takeoff_subscriber_ = node_.subscribe(
        config_.takeoff_complete_topic, 1,
        &SearchCoordinatorNode::takeoffCallback, this);
    tracking_subscriber_ = node_.subscribe(
        config_.tracking_phase_topic, 1,
        &SearchCoordinatorNode::trackingCallback, this);
    mux_subscriber_ = node_.subscribe(
        config_.mux_selected_topic, 1,
        &SearchCoordinatorNode::muxCallback, this);
    adapter_subscriber_ = node_.subscribe(
        config_.adapter_status_topic, 1,
        &SearchCoordinatorNode::adapterCallback, this);

    validation_client_ = node_.serviceClient<search_msgs::ValidateTrajectory>(
        config_.validate_trajectory_service, false);
    validation_thread_ =
        std::thread(&SearchCoordinatorNode::validationLoop, this);
    timer_ = node_.createTimer(ros::Duration(1.0 / config_.update_rate),
                               &SearchCoordinatorNode::timerCallback, this);
  }

  ~SearchCoordinatorNode() {
    {
      std::lock_guard<std::mutex> lock(validation_mutex_);
      shutting_down_ = true;
    }
    validation_condition_.notify_all();
    validation_client_.shutdown();
    if (validation_thread_.joinable()) {
      validation_thread_.join();
    }
  }

 private:
  struct ValidationJob {
    std::uint64_t sequence{0u};
    std::uint64_t generation{0u};
    ValidationKind kind{ValidationKind::NONE};
    Vec3 start;
    Vec3 goal;
    bool wait_for_generation{false};
  };

  void highLevelGoalCallback(
      const geometry_msgs::PoseStampedConstPtr& message) {
    high_level_goal_ = stampedGoal(*message, ros::Time::now());
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    odom_ = *message;
    odom_received_at_ = ros::Time::now();
    has_odom_ = true;
  }

  void healthCallback(
      const search_msgs::PerceptionHealthConstPtr& message) {
    health_ = *message;
    health_received_at_ = ros::Time::now();
    has_health_ = true;
  }

  void frontierCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    frontier_goal_ = stampedGoal(*message, ros::Time::now());
  }

  void missionCallback(const std_msgs::BoolConstPtr& message) {
    mission_active_ = message->data;
  }

  void takeoffCallback(const std_msgs::BoolConstPtr& message) {
    takeoff_complete_ = message->data;
  }

  void trackingCallback(const std_msgs::StringConstPtr& message) {
    tracking_phase_ = message->data;
  }

  void muxCallback(const std_msgs::StringConstPtr& message) {
    mux_selected_ = message->data;
  }

  void adapterCallback(const std_msgs::StringConstPtr& message) {
    adapter_status_ = message->data;
  }

  CoordinatorInput input(const ros::Time& now) {
    CoordinatorInput value;
    value.now = now.toSec();
    value.ready = takeoff_complete_;
    value.mission_active = mission_active_;
    value.tracking_candidate = startsWith(tracking_phase_, "DETECTING:");
    value.tracking_active = startsWith(tracking_phase_, "DASH:") ||
                            startsWith(tracking_phase_, "TRACKING:") ||
                            startsWith(tracking_phase_, "LOST:");
    value.mux_selected = mux_selected_;
    value.high_level_goal = high_level_goal_;
    value.frontier_goal = frontier_goal_;
    value.adapter_status = adapter_status_;

    const bool odom_fresh =
        has_odom_ && (now - odom_received_at_).toSec() >= 0.0 &&
        (now - odom_received_at_).toSec() <= config_.input_timeout;
    value.has_odom = odom_fresh;
    if (has_odom_) {
      value.odom = Vec3{odom_.pose.pose.position.x, odom_.pose.pose.position.y,
                        odom_.pose.pose.position.z};
    }

    const bool health_fresh =
        has_health_ && (now - health_received_at_).toSec() >= 0.0 &&
        (now - health_received_at_).toSec() <= config_.input_timeout;
    value.map_healthy =
        health_fresh && health_.depth_healthy && health_.odom_healthy &&
        health_.synchronized && health_.map_healthy;

    {
      std::lock_guard<std::mutex> lock(validation_mutex_);
      if (has_validation_result_) {
        value.validation = validation_result_;
        has_validation_result_ = false;
      }
    }
    return value;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    CoordinatorInput current = input(now);
    const CoordinatorOutput output = coordinator_->step(current);

    if (output.publish_generation) {
      std_msgs::UInt64 message;
      message.data = output.generation;
      generation_publisher_.publish(message);
    }
    if (output.publish_ego_goal) {
      geometry_msgs::PoseStamped message;
      message.header.stamp = now;
      message.header.frame_id = "map";
      message.pose.position.x = output.goal.x;
      message.pose.position.y = output.goal.y;
      message.pose.position.z = output.goal.z;
      message.pose.orientation.w = 1.0;
      goal_publisher_.publish(message);
    }
    if (output.request_validation) {
      queueValidation(output.generation, output.validation_kind, current.odom,
                      output.validation_goal, output.publish_generation);
    }

    std_msgs::String status;
    const std::string visible_state =
        output.fault_code == "NO_KNOWN_FREE_GOAL"
            ? "OBSERVING"
            : stateName(output.state);
    status.data = visible_state + ":" + output.fault_code;
    status_publisher_.publish(status);
  }

  void queueValidation(std::uint64_t generation, ValidationKind kind,
                       const Vec3& start, const Vec3& goal,
                       bool wait_for_generation) {
    std::lock_guard<std::mutex> lock(validation_mutex_);
    queued_validation_.sequence = ++validation_sequence_;
    queued_validation_.generation = generation;
    queued_validation_.kind = kind;
    queued_validation_.start = start;
    queued_validation_.goal = goal;
    queued_validation_.wait_for_generation = wait_for_generation;
    validation_queued_ = true;
    validation_condition_.notify_one();
  }

  std::vector<geometry_msgs::Point> validationSamples(
      const ValidationJob& job) const {
    const double dx = job.goal.x - job.start.x;
    const double dy = job.goal.y - job.start.y;
    const double dz = job.goal.z - job.start.z;
    const double distance = std::hypot(std::hypot(dx, dy), dz);
    const std::size_t segments = std::max<std::size_t>(
        1u, static_cast<std::size_t>(
                std::ceil(distance / config_.validation_sample_spacing)));
    std::vector<geometry_msgs::Point> samples;
    samples.reserve(segments + 1u);
    for (std::size_t index = 0u; index <= segments; ++index) {
      const double ratio = static_cast<double>(index) / segments;
      geometry_msgs::Point sample;
      sample.x = job.start.x + dx * ratio;
      sample.y = job.start.y + dy * ratio;
      sample.z = job.start.z + dz * ratio;
      samples.push_back(sample);
    }
    return samples;
  }

  void validationLoop() {
    while (ros::ok()) {
      ValidationJob job;
      {
        std::unique_lock<std::mutex> lock(validation_mutex_);
        validation_condition_.wait(lock, [this] {
          return shutting_down_ || validation_queued_;
        });
        if (shutting_down_) {
          return;
        }
        job = queued_validation_;
        validation_queued_ = false;
      }

      if (job.wait_for_generation && config_.generation_settle_delay > 0.0) {
        ros::WallDuration(config_.generation_settle_delay).sleep();
      }
      search_msgs::ValidateTrajectory service;
      service.request.header.stamp = ros::Time::now();
      service.request.header.frame_id = "map";
      service.request.task_generation = job.generation;
      service.request.samples = validationSamples(job);
      const bool called = validation_client_.call(service);

      ValidationResult result;
      result.available = true;
      result.generation =
          called ? service.response.task_generation : job.generation;
      result.kind = job.kind;
      result.valid = called && service.response.valid;
      {
        std::lock_guard<std::mutex> lock(validation_mutex_);
        if (job.sequence >= validation_result_sequence_) {
          validation_result_sequence_ = job.sequence;
          validation_result_ = result;
          has_validation_result_ = true;
        }
      }
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  NodeConfig config_;
  std::unique_ptr<Coordinator> coordinator_;

  ros::Publisher generation_publisher_;
  ros::Publisher goal_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber high_level_goal_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber health_subscriber_;
  ros::Subscriber frontier_subscriber_;
  ros::Subscriber mission_subscriber_;
  ros::Subscriber takeoff_subscriber_;
  ros::Subscriber tracking_subscriber_;
  ros::Subscriber mux_subscriber_;
  ros::Subscriber adapter_subscriber_;
  ros::ServiceClient validation_client_;
  ros::Timer timer_;

  StampedGoal high_level_goal_;
  StampedGoal frontier_goal_;
  nav_msgs::Odometry odom_;
  search_msgs::PerceptionHealth health_;
  ros::Time odom_received_at_;
  ros::Time health_received_at_;
  bool has_odom_{false};
  bool has_health_{false};
  bool mission_active_{false};
  bool takeoff_complete_{false};
  std::string tracking_phase_;
  std::string mux_selected_;
  std::string adapter_status_;

  std::mutex validation_mutex_;
  std::condition_variable validation_condition_;
  std::thread validation_thread_;
  bool shutting_down_{false};
  bool validation_queued_{false};
  ValidationJob queued_validation_;
  std::uint64_t validation_sequence_{0u};
  std::uint64_t validation_result_sequence_{0u};
  bool has_validation_result_{false};
  ValidationResult validation_result_;
};

}  // namespace search_coordinator

int main(int argc, char** argv) {
  ros::init(argc, argv, "search_coordinator");
  try {
    search_coordinator::SearchCoordinatorNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("search_coordinator failed: %s", error.what());
    return 1;
  }
  return 0;
}
