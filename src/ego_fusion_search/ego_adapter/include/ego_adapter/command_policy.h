#pragma once

#include <cstdint>
#include <string>

namespace ego_adapter {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
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
  AxisClearance up;
  AxisClearance down;
};

struct PolicyInput {
  double now{0.0};
  double command_stamp{0.0};
  std::uint64_t bound_generation{0u};
  std::uint64_t active_generation{0u};
  bool map_healthy{false};
  bool mux_is_navigator{false};
  bool trajectory_valid{false};
  Vec3 current_position;
  Vec3 desired_position;
  double current_yaw{0.0};
  double desired_yaw{0.0};
  double desired_yaw_rate{0.0};
  Vec3 world_velocity;
  DirectionalClearance clearance;
};

struct PolicyOutput {
  bool accepted{false};
  std::string fault_code{"NOT_READY"};
  double forward{0.0};
  double left{0.0};
  double up{0.0};
  double yaw_rate{0.0};
};

struct PolicyLimits {
  double command_timeout = 0.20;
  double max_search_altitude = 4.0;
  double position_gain = 0.60;
  double yaw_align_threshold = 0.5235987756;
  double max_forward_speed = 1.5;
  double max_lateral_speed = 0.25;
  double max_reverse_speed = 0.10;
  double max_vertical_speed = 0.50;
  double max_yaw_rate = 0.80;
  double braking_clearance = 1.50;
  double emergency_clearance = 0.80;
};

class CommandPolicy {
 public:
  explicit CommandPolicy(const PolicyLimits& limits);
  PolicyOutput evaluate(const PolicyInput& input) const;

 private:
  PolicyLimits limits_;
};

}  // namespace ego_adapter
