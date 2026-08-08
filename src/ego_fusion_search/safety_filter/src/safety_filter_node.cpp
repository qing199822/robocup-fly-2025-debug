#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <search_msgs/LocalClearance.h>
#include <search_msgs/PerceptionHealth.h>
#include <std_msgs/String.h>

#include "safety_filter/safety_policy.h"

class SafetyFilterNode {
 public:
  SafetyFilterNode() : nh_(), private_nh_("~") {
    std::string raw_topic;
    std::string odom_topic;
    std::string final_topic;
    std::string status_topic;
    std::string health_topic;
    std::string clearance_topic;
    std::string mux_selected_topic;
    double publish_rate = 20.0;
    safety_filter::Limits limits;

    private_nh_.param("raw_command_topic", raw_topic,
                      std::string("control/raw_cmd_vel"));
    private_nh_.param("odom_topic", odom_topic, std::string("global_odom"));
    private_nh_.param("final_command_topic", final_topic,
                      std::string("final_cmd_vel"));
    private_nh_.param("status_topic", status_topic,
                      std::string("safety/status"));
    private_nh_.param("publish_rate", publish_rate, 20.0);
    private_nh_.param("command_timeout", command_timeout_, 0.25);
    private_nh_.param("odom_timeout", odom_timeout_, 0.25);
    private_nh_.param("max_xy_speed", limits.max_xy_speed, 3.0);
    private_nh_.param("max_z_speed", limits.max_z_speed, 1.0);
    private_nh_.param("max_yaw_rate", limits.max_yaw_rate, 1.0);
    private_nh_.param("max_xy_acceleration", limits.max_xy_acceleration, 2.0);
    private_nh_.param("max_z_acceleration", limits.max_z_acceleration, 1.0);
    private_nh_.param("min_altitude", limits.min_altitude, 0.5);
    private_nh_.param("max_altitude", limits.max_altitude, 4.0);
    private_nh_.param("perception_guard_enabled", perception_guard_enabled_,
                      false);
    private_nh_.param("perception_timeout", perception_timeout_, 0.50);
    private_nh_.param("perception_recovery_time", perception_recovery_time_,
                      1.0);
    private_nh_.param("navigator_max_altitude", navigator_max_altitude_, 4.0);
    private_nh_.param("external_max_altitude", external_max_altitude_, 6.0);
    private_nh_.param("braking_clearance", perception_limits_.braking_clearance,
                      1.50);
    private_nh_.param("emergency_clearance",
                      perception_limits_.emergency_clearance, 0.80);
    private_nh_.param("health_topic", health_topic,
                      std::string("local_mapping/health"));
    private_nh_.param("clearance_topic", clearance_topic,
                      std::string("local_mapping/clearance"));
    private_nh_.param("mux_selected_topic", mux_selected_topic,
                      std::string("pose_cmd_mux/selected"));
    private_nh_.param("takeoff_topic", takeoff_topic_,
                      std::string("mux_inputs/takeoff/cmd_vel"));
    private_nh_.param("navigator_topic", navigator_topic_,
                      std::string("mux_inputs/navigator/cmd_vel"));
    private_nh_.param("external_topic", external_topic_,
                      std::string("mux_inputs/external/pose_cmd"));

    if (!std::isfinite(publish_rate) || publish_rate <= 0.0 ||
        !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
        !std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0 ||
        !std::isfinite(limits.max_xy_speed) || limits.max_xy_speed <= 0.0 ||
        !std::isfinite(limits.max_z_speed) || limits.max_z_speed <= 0.0 ||
        !std::isfinite(limits.max_yaw_rate) || limits.max_yaw_rate <= 0.0 ||
        !std::isfinite(limits.max_xy_acceleration) ||
        limits.max_xy_acceleration <= 0.0 ||
        !std::isfinite(limits.max_z_acceleration) ||
        limits.max_z_acceleration <= 0.0 ||
        !std::isfinite(limits.min_altitude) ||
        !std::isfinite(limits.max_altitude) ||
        limits.min_altitude >= limits.max_altitude ||
        !std::isfinite(perception_timeout_) || perception_timeout_ <= 0.0 ||
        !std::isfinite(perception_recovery_time_) ||
        perception_recovery_time_ < 0.0 ||
        !std::isfinite(navigator_max_altitude_) ||
        navigator_max_altitude_ <= limits.min_altitude ||
        !std::isfinite(external_max_altitude_) ||
        external_max_altitude_ < navigator_max_altitude_) {
      throw std::invalid_argument(
          "invalid safety_filter timing or altitude parameters");
    }

    policy_.reset(new safety_filter::SafetyPolicy(limits));
    perception_guard_.reset(
        new safety_filter::PerceptionGuard(perception_limits_));
    default_max_altitude_ = limits.max_altitude;
    raw_command_sub_ = nh_.subscribe(
        raw_topic, 1, &SafetyFilterNode::rawCommandCallback, this);
    odom_sub_ =
        nh_.subscribe(odom_topic, 1, &SafetyFilterNode::odomCallback, this);
    final_command_pub_ = nh_.advertise<geometry_msgs::Twist>(final_topic, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic, 1, true);
    if (perception_guard_enabled_) {
      health_sub_ = nh_.subscribe(
          health_topic, 1, &SafetyFilterNode::healthCallback, this);
      clearance_sub_ = nh_.subscribe(
          clearance_topic, 1, &SafetyFilterNode::clearanceCallback, this);
      mux_selected_sub_ = nh_.subscribe(
          mux_selected_topic, 1, &SafetyFilterNode::muxSelectedCallback, this);
    }
    publish_timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_rate), &SafetyFilterNode::tick, this);
  }

 private:
  void rawCommandCallback(const geometry_msgs::Twist::ConstPtr& message) {
    latest_raw_command_ = *message;
    raw_command_received_at_ = ros::Time::now();
    has_raw_command_ = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_odom_frame_ = message->header.frame_id;
    latest_altitude_ = message->pose.pose.position.z;
    odom_received_at_ = ros::Time::now();
    has_odom_ = true;
  }

  void healthCallback(const search_msgs::PerceptionHealth::ConstPtr& message) {
    latest_health_ = *message;
    health_received_at_ = ros::Time::now();
    has_health_ = true;
    if (message->depth_healthy && message->odom_healthy &&
        message->synchronized && message->map_healthy) {
      if (healthy_since_.isZero()) {
        healthy_since_ = health_received_at_;
      }
    } else {
      healthy_since_ = ros::Time();
    }
  }

  void clearanceCallback(const search_msgs::LocalClearance::ConstPtr& message) {
    latest_clearance_ = *message;
    clearance_received_at_ = ros::Time::now();
    has_clearance_ = true;
  }

  void muxSelectedCallback(const std_msgs::String::ConstPtr& message) {
    if (!has_mux_selection_ || selected_topic_ != message->data) {
      has_raw_command_ = false;
      policy_->reset();
    }
    selected_topic_ = message->data;
    has_mux_selection_ = true;
  }

  void tick(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    std::string status;
    geometry_msgs::Twist output;
    if (!has_odom_ || (now - odom_received_at_).toSec() > odom_timeout_) {
      policy_->reset();
      output = safety_filter::zeroCommand();
      status = "ODOM_TIMEOUT";
    } else if (!has_raw_command_ ||
               (now - raw_command_received_at_).toSec() > command_timeout_) {
      policy_->reset();
      output = safety_filter::zeroCommand();
      status = "COMMAND_TIMEOUT";
    } else {
      double max_altitude = default_max_altitude_;
      geometry_msgs::Twist guarded_command = latest_raw_command_;
      bool perception_blocked = false;
      if (perception_guard_enabled_) {
        if (!has_mux_selection_ ||
            (selected_topic_ != takeoff_topic_ &&
             selected_topic_ != navigator_topic_ &&
             selected_topic_ != external_topic_)) {
          policy_->reset();
          output = safety_filter::zeroCommand();
          status = "MUX_UNKNOWN";
          publish(now, output, status);
          return;
        }
        max_altitude = selected_topic_ == external_topic_
                           ? external_max_altitude_
                           : navigator_max_altitude_;
        const bool perception_fresh =
            has_health_ && has_clearance_ &&
            (now - health_received_at_).toSec() <= perception_timeout_ &&
            (now - clearance_received_at_).toSec() <= perception_timeout_;
        const bool perception_healthy =
            perception_fresh && latest_health_.depth_healthy &&
            latest_health_.odom_healthy && latest_health_.synchronized &&
            latest_health_.map_healthy && !healthy_since_.isZero() &&
            (now - healthy_since_).toSec() >= perception_recovery_time_;
        if (!perception_healthy) {
          if (!perception_fresh) {
            healthy_since_ = ros::Time();
          }
          policy_->reset();
          output = safety_filter::zeroCommand();
          status = "PERCEPTION_TIMEOUT";
          publish(now, output, status);
          return;
        }
        if (latest_odom_frame_.empty() ||
            latest_health_.header.frame_id != latest_odom_frame_ ||
            latest_clearance_.header.frame_id != latest_odom_frame_) {
          healthy_since_ = ros::Time();
          policy_->reset();
          output = safety_filter::zeroCommand();
          status = "PERCEPTION_FRAME_MISMATCH";
          publish(now, output, status);
          return;
        }
        const safety_filter::DirectionalClearance clearance{
            {latest_clearance_.forward_known != 0u,
             latest_clearance_.forward_m},
            {latest_clearance_.backward_known != 0u,
             latest_clearance_.backward_m},
            {latest_clearance_.left_known != 0u, latest_clearance_.left_m},
            {latest_clearance_.right_known != 0u, latest_clearance_.right_m},
            {latest_clearance_.upward_known != 0u,
             latest_clearance_.upward_m},
            {latest_clearance_.downward_known != 0u,
             latest_clearance_.downward_m}};
        const auto guarded = perception_guard_->apply(guarded_command, clearance);
        guarded_command = guarded.command;
        perception_blocked = guarded.blocked;
      }
      const double dt = previous_tick_.isZero()
                            ? 0.05
                            : (now - previous_tick_).toSec();
      const auto result = policy_->apply(guarded_command, latest_altitude_, dt,
                                         max_altitude);
      output = result.command;
      status = result.fault == safety_filter::Fault::NONE && perception_blocked
                   ? "PERCEPTION_BLOCKED"
                   : safety_filter::faultCode(result.fault);
    }
    publish(now, output, status);
  }

  void publish(const ros::Time& now, const geometry_msgs::Twist& output,
               const std::string& status) {
    previous_tick_ = now;
    final_command_pub_.publish(output);
    std_msgs::String message;
    message.data = status;
    status_pub_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber raw_command_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber health_sub_;
  ros::Subscriber clearance_sub_;
  ros::Subscriber mux_selected_sub_;
  ros::Publisher final_command_pub_;
  ros::Publisher status_pub_;
  ros::Timer publish_timer_;
  geometry_msgs::Twist latest_raw_command_;
  double latest_altitude_{0.0};
  std::string latest_odom_frame_;
  search_msgs::PerceptionHealth latest_health_;
  search_msgs::LocalClearance latest_clearance_;
  std::string selected_topic_;
  ros::Time raw_command_received_at_;
  ros::Time odom_received_at_;
  ros::Time health_received_at_;
  ros::Time clearance_received_at_;
  ros::Time healthy_since_;
  ros::Time previous_tick_;
  bool has_raw_command_{false};
  bool has_odom_{false};
  bool has_health_{false};
  bool has_clearance_{false};
  bool has_mux_selection_{false};
  double command_timeout_{0.25};
  double odom_timeout_{0.25};
  bool perception_guard_enabled_{false};
  double perception_timeout_{0.50};
  double perception_recovery_time_{1.0};
  double default_max_altitude_{4.0};
  double navigator_max_altitude_{4.0};
  double external_max_altitude_{6.0};
  std::string takeoff_topic_;
  std::string navigator_topic_;
  std::string external_topic_;
  safety_filter::PerceptionLimits perception_limits_;
  std::unique_ptr<safety_filter::SafetyPolicy> policy_;
  std::unique_ptr<safety_filter::PerceptionGuard> perception_guard_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "safety_filter");
  try {
    SafetyFilterNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("safety_filter initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
