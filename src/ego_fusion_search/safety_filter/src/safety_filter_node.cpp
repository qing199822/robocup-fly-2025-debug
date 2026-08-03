#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "safety_filter/safety_policy.h"

class SafetyFilterNode {
 public:
  SafetyFilterNode() : nh_(), private_nh_("~") {
    std::string raw_topic;
    std::string odom_topic;
    std::string final_topic;
    std::string status_topic;
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
    private_nh_.param("max_altitude", limits.max_altitude, 5.5);

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
        limits.min_altitude >= limits.max_altitude) {
      throw std::invalid_argument(
          "invalid safety_filter timing or altitude parameters");
    }

    policy_.reset(new safety_filter::SafetyPolicy(limits));
    raw_command_sub_ = nh_.subscribe(
        raw_topic, 1, &SafetyFilterNode::rawCommandCallback, this);
    odom_sub_ =
        nh_.subscribe(odom_topic, 1, &SafetyFilterNode::odomCallback, this);
    final_command_pub_ = nh_.advertise<geometry_msgs::Twist>(final_topic, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic, 1, true);
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
    latest_altitude_ = message->pose.pose.position.z;
    odom_received_at_ = ros::Time::now();
    has_odom_ = true;
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
      const double dt = previous_tick_.isZero()
                            ? 0.05
                            : (now - previous_tick_).toSec();
      const auto result =
          policy_->apply(latest_raw_command_, latest_altitude_, dt);
      output = result.command;
      status = safety_filter::faultCode(result.fault);
    }
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
  ros::Publisher final_command_pub_;
  ros::Publisher status_pub_;
  ros::Timer publish_timer_;
  geometry_msgs::Twist latest_raw_command_;
  double latest_altitude_{0.0};
  ros::Time raw_command_received_at_;
  ros::Time odom_received_at_;
  ros::Time previous_tick_;
  bool has_raw_command_{false};
  bool has_odom_{false};
  double command_timeout_{0.25};
  double odom_timeout_{0.25};
  std::unique_ptr<safety_filter::SafetyPolicy> policy_;
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
