#pragma once

#include <geometry_msgs/Twist.h>

namespace safety_filter {

enum class Fault { NONE, NON_FINITE_COMMAND, INVALID_DT, ALTITUDE_LIMIT };

struct Limits {
  double max_xy_speed{3.0};
  double max_z_speed{1.0};
  double max_yaw_rate{1.0};
  double max_xy_acceleration{2.0};
  double max_z_acceleration{1.0};
  double min_altitude{0.5};
  double max_altitude{4.0};
};

struct Result {
  geometry_msgs::Twist command;
  Fault fault{Fault::NONE};
};

struct AxisClearance {
  bool known{false};
  double metres{0.0};
};

struct DirectionalClearance {
  AxisClearance forward;
  AxisClearance backward;
  AxisClearance left;
  AxisClearance right;
  AxisClearance upward;
  AxisClearance downward;
};

struct PerceptionLimits {
  double braking_clearance{1.5};
  double emergency_clearance{0.8};
};

struct PerceptionResult {
  geometry_msgs::Twist command;
  bool blocked{false};
};

class PerceptionGuard {
 public:
  explicit PerceptionGuard(const PerceptionLimits& limits);
  PerceptionResult apply(const geometry_msgs::Twist& requested,
                         const DirectionalClearance& clearance) const;

 private:
  PerceptionLimits limits_;
};

class SafetyPolicy {
 public:
  explicit SafetyPolicy(const Limits& limits);
  Result apply(const geometry_msgs::Twist& requested, double altitude, double dt,
               double max_altitude);
  void reset();

 private:
  Limits limits_;
  geometry_msgs::Twist previous_;
};

const char* faultCode(Fault fault);
geometry_msgs::Twist zeroCommand();

}  // namespace safety_filter
