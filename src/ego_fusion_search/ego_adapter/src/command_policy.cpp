#include "ego_adapter/command_policy.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ego_adapter {
namespace {

bool finite(double value) { return std::isfinite(value); }

bool finite(const Vec3& value) {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

double clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

PolicyOutput reject(const std::string& fault_code) {
  return PolicyOutput{false, fault_code, 0.0, 0.0, 0.0, 0.0};
}

bool validClearance(const AxisClearance& clearance) {
  return !clearance.known ||
         (finite(clearance.metres) && clearance.metres >= 0.0);
}

double normalizedYawError(double desired, double current) {
  const double difference = desired - current;
  return std::atan2(std::sin(difference), std::cos(difference));
}

double applyClearance(double velocity, const AxisClearance& positive,
                      const AxisClearance& negative, const PolicyLimits& limits,
                      bool* limited) {
  if (velocity == 0.0) {
    return 0.0;
  }
  const AxisClearance& clearance = velocity > 0.0 ? positive : negative;
  if (!clearance.known || clearance.metres <= limits.emergency_clearance) {
    *limited = true;
    return 0.0;
  }
  if (clearance.metres < limits.braking_clearance) {
    const double scale =
        (clearance.metres - limits.emergency_clearance) /
        (limits.braking_clearance - limits.emergency_clearance);
    *limited = true;
    return velocity * scale;
  }
  return velocity;
}

bool validLimits(const PolicyLimits& limits) {
  return finite(limits.command_timeout) && limits.command_timeout > 0.0 &&
         finite(limits.max_search_altitude) &&
         limits.max_search_altitude > 0.0 &&
         finite(limits.position_gain) && limits.position_gain >= 0.0 &&
         finite(limits.yaw_align_threshold) &&
         limits.yaw_align_threshold > 0.0 &&
         limits.yaw_align_threshold <= 3.14159265358979323846 &&
         finite(limits.max_forward_speed) &&
         limits.max_forward_speed >= 0.0 &&
         finite(limits.max_lateral_speed) &&
         limits.max_lateral_speed >= 0.0 &&
         finite(limits.max_reverse_speed) &&
         limits.max_reverse_speed >= 0.0 &&
         finite(limits.max_vertical_speed) &&
         limits.max_vertical_speed >= 0.0 && finite(limits.max_yaw_rate) &&
         limits.max_yaw_rate >= 0.0 && finite(limits.braking_clearance) &&
         finite(limits.emergency_clearance) &&
         limits.emergency_clearance >= 0.0 &&
         limits.braking_clearance > limits.emergency_clearance;
}

}  // namespace

CommandPolicy::CommandPolicy(const PolicyLimits& limits) : limits_(limits) {
  if (!validLimits(limits_)) {
    throw std::invalid_argument("invalid EGO command policy limits");
  }
}

PolicyOutput CommandPolicy::evaluate(const PolicyInput& input) const {
  if (!finite(input.now) || !finite(input.command_stamp) ||
      !finite(input.current_position) || !finite(input.desired_position) ||
      !finite(input.current_yaw) || !finite(input.desired_yaw) ||
      !finite(input.desired_yaw_rate) || !finite(input.world_velocity)) {
    return reject("NON_FINITE_INPUT");
  }
  const DirectionalClearance& clearance = input.clearance;
  if (!validClearance(clearance.forward) ||
      !validClearance(clearance.backward) ||
      !validClearance(clearance.left) || !validClearance(clearance.right) ||
      !validClearance(clearance.up) || !validClearance(clearance.down)) {
    return reject("INVALID_CLEARANCE");
  }
  if (input.command_stamp > input.now) {
    return reject("TIME_ROLLBACK");
  }
  if (input.now - input.command_stamp > limits_.command_timeout) {
    return reject("STALE_COMMAND");
  }
  if (input.bound_generation != input.active_generation) {
    return reject("WRONG_GENERATION");
  }
  if (!input.map_healthy) {
    return reject("MAP_UNHEALTHY");
  }
  if (!input.mux_is_navigator) {
    return reject("MUX_NOT_NAVIGATOR");
  }
  if (!input.trajectory_valid) {
    return reject("TRAJECTORY_INVALID");
  }
  if (input.desired_position.z > limits_.max_search_altitude) {
    return reject("HEIGHT_LIMIT");
  }

  const Vec3 corrected_world_velocity{
      input.world_velocity.x +
          limits_.position_gain *
              (input.desired_position.x - input.current_position.x),
      input.world_velocity.y +
          limits_.position_gain *
              (input.desired_position.y - input.current_position.y),
      input.world_velocity.z +
          limits_.position_gain *
              (input.desired_position.z - input.current_position.z)};
  if (!finite(corrected_world_velocity)) {
    return reject("NON_FINITE_INPUT");
  }

  const double cosine = std::cos(input.current_yaw);
  const double sine = std::sin(input.current_yaw);
  double forward = cosine * corrected_world_velocity.x +
                   sine * corrected_world_velocity.y;
  double left = -sine * corrected_world_velocity.x +
                cosine * corrected_world_velocity.y;
  double up = corrected_world_velocity.z;
  forward = clamp(forward, -limits_.max_reverse_speed,
                  limits_.max_forward_speed);
  left = clamp(left, -limits_.max_lateral_speed, limits_.max_lateral_speed);
  up = clamp(up, -limits_.max_vertical_speed, limits_.max_vertical_speed);

  const double yaw_error =
      normalizedYawError(input.desired_yaw, input.current_yaw);
  const double yaw_rate =
      clamp(input.desired_yaw_rate + yaw_error, -limits_.max_yaw_rate,
            limits_.max_yaw_rate);
  if (std::abs(yaw_error) > limits_.yaw_align_threshold) {
    return PolicyOutput{true, "YAW_ALIGNING", 0.0, 0.0, 0.0, yaw_rate};
  }

  bool clearance_limited = false;
  forward = applyClearance(forward, clearance.forward, clearance.backward,
                           limits_, &clearance_limited);
  left = applyClearance(left, clearance.left, clearance.right, limits_,
                        &clearance_limited);
  up = applyClearance(up, clearance.up, clearance.down, limits_,
                      &clearance_limited);
  return PolicyOutput{true, clearance_limited ? "CLEARANCE_LIMITED" : "OK",
                      forward, left, up, yaw_rate};
}

}  // namespace ego_adapter
