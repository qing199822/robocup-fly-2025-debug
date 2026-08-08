#include "safety_filter/safety_policy.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace safety_filter {
namespace {

double clamp(double value, double limit) {
  return std::max(-limit, std::min(limit, value));
}

bool finite(const geometry_msgs::Twist& value) {
  return std::isfinite(value.linear.x) && std::isfinite(value.linear.y) &&
         std::isfinite(value.linear.z) && std::isfinite(value.angular.x) &&
         std::isfinite(value.angular.y) && std::isfinite(value.angular.z);
}

double guardedComponent(double value, const AxisClearance& clearance,
                        const PerceptionLimits& limits, bool* blocked) {
  if (value == 0.0) {
    return 0.0;
  }
  if (!clearance.known || !std::isfinite(clearance.metres) ||
      clearance.metres < 0.0 ||
      clearance.metres <= limits.emergency_clearance) {
    *blocked = true;
    return 0.0;
  }
  if (clearance.metres < limits.braking_clearance) {
    *blocked = true;
    return value * (clearance.metres - limits.emergency_clearance) /
                   (limits.braking_clearance - limits.emergency_clearance);
  }
  return value;
}

}  // namespace

SafetyPolicy::SafetyPolicy(const Limits& limits) : limits_(limits) {}

PerceptionGuard::PerceptionGuard(const PerceptionLimits& limits)
    : limits_(limits) {
  if (!std::isfinite(limits_.braking_clearance) ||
      !std::isfinite(limits_.emergency_clearance) ||
      limits_.emergency_clearance < 0.0 ||
      limits_.braking_clearance <= limits_.emergency_clearance) {
    throw std::invalid_argument("invalid perception clearance limits");
  }
}

PerceptionResult PerceptionGuard::apply(
    const geometry_msgs::Twist& requested,
    const DirectionalClearance& clearance) const {
  PerceptionResult result;
  result.command = requested;
  const AxisClearance* axes[] = {
      &clearance.forward, &clearance.backward, &clearance.left,
      &clearance.right,   &clearance.upward,   &clearance.downward};
  for (const AxisClearance* axis : axes) {
    if (!std::isfinite(axis->metres) || axis->metres < 0.0) {
      result.command.linear.x = 0.0;
      result.command.linear.y = 0.0;
      result.command.linear.z = 0.0;
      result.blocked = true;
      return result;
    }
  }
  const AxisClearance& x_clearance =
      requested.linear.x >= 0.0 ? clearance.forward : clearance.backward;
  const AxisClearance& y_clearance =
      requested.linear.y >= 0.0 ? clearance.left : clearance.right;
  const AxisClearance& z_clearance =
      requested.linear.z >= 0.0 ? clearance.upward : clearance.downward;
  result.command.linear.x = guardedComponent(
      requested.linear.x, x_clearance, limits_, &result.blocked);
  result.command.linear.y = guardedComponent(
      requested.linear.y, y_clearance, limits_, &result.blocked);
  result.command.linear.z = guardedComponent(
      requested.linear.z, z_clearance, limits_, &result.blocked);
  return result;
}

geometry_msgs::Twist zeroCommand() { return geometry_msgs::Twist{}; }

void SafetyPolicy::reset() { previous_ = zeroCommand(); }

Result SafetyPolicy::apply(const geometry_msgs::Twist& requested,
                           double altitude, double dt,
                           double max_altitude) {
  if (!finite(requested) || !std::isfinite(altitude) ||
      !std::isfinite(max_altitude) || max_altitude <= limits_.min_altitude) {
    reset();
    return {zeroCommand(), Fault::NON_FINITE_COMMAND};
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    reset();
    return {zeroCommand(), Fault::INVALID_DT};
  }

  Result result;
  result.command = requested;
  const double horizontal =
      std::hypot(result.command.linear.x, result.command.linear.y);
  if (horizontal > limits_.max_xy_speed && horizontal > 0.0) {
    const double scale = limits_.max_xy_speed / horizontal;
    result.command.linear.x *= scale;
    result.command.linear.y *= scale;
  }
  result.command.linear.z = clamp(result.command.linear.z, limits_.max_z_speed);
  result.command.angular.x = 0.0;
  result.command.angular.y = 0.0;
  result.command.angular.z =
      clamp(result.command.angular.z, limits_.max_yaw_rate);

  const bool altitude_limited =
      (altitude >= max_altitude && result.command.linear.z > 0.0) ||
      (altitude <= limits_.min_altitude && result.command.linear.z < 0.0);

  const double max_xy_step = limits_.max_xy_acceleration * dt;
  const double delta_x = result.command.linear.x - previous_.linear.x;
  const double delta_y = result.command.linear.y - previous_.linear.y;
  const double delta_xy = std::hypot(delta_x, delta_y);
  if (delta_xy > max_xy_step && delta_xy > 0.0) {
    const double scale = max_xy_step / delta_xy;
    result.command.linear.x = previous_.linear.x + delta_x * scale;
    result.command.linear.y = previous_.linear.y + delta_y * scale;
  }
  const double max_z_step = limits_.max_z_acceleration * dt;
  result.command.linear.z =
      previous_.linear.z +
      clamp(result.command.linear.z - previous_.linear.z, max_z_step);

  // The altitude boundary is a hard gate and must override acceleration
  // smoothing, which could otherwise reintroduce an earlier climb command.
  if (altitude_limited) {
    result.command.linear.z = 0.0;
    result.fault = Fault::ALTITUDE_LIMIT;
  }
  previous_ = result.command;
  return result;
}

const char* faultCode(Fault fault) {
  switch (fault) {
    case Fault::NONE:
      return "OK";
    case Fault::NON_FINITE_COMMAND:
      return "NON_FINITE_COMMAND";
    case Fault::INVALID_DT:
      return "INVALID_DT";
    case Fault::ALTITUDE_LIMIT:
      return "ALTITUDE_LIMIT";
  }
  return "UNKNOWN_FAULT";
}

}  // namespace safety_filter
