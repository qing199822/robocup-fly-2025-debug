#include "local_mapping/health_monitor.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace local_mapping {
namespace {

bool validConfig(const HealthConfig& config) {
  return std::isfinite(config.max_sync_delta) &&
         std::isfinite(config.depth_timeout) &&
         std::isfinite(config.odom_timeout) &&
         std::isfinite(config.recovery_window) &&
         std::isfinite(config.min_valid_depth_ratio) &&
         config.max_sync_delta >= 0.0 && config.depth_timeout > 0.0 &&
         config.odom_timeout > 0.0 && config.recovery_window > 0.0 &&
         config.min_valid_depth_ratio >= 0.0 &&
         config.min_valid_depth_ratio <= 1.0;
}

double timestampTolerance(double first, double second) {
  return 8.0 * std::numeric_limits<double>::epsilon() *
         std::fmax(1.0, std::fmax(std::fabs(first), std::fabs(second)));
}

bool intervalExceeds(double current, double previous, double limit) {
  return current - previous > limit + timestampTolerance(current, previous);
}

bool intervalWithin(double current, double previous, double limit) {
  return current >= previous && !intervalExceeds(current, previous, limit);
}

bool intervalReached(double current, double previous, double limit) {
  return current - previous + timestampTolerance(current, previous) >= limit;
}

}  // namespace

HealthMonitor::HealthMonitor(const HealthConfig& config)
    : config_(config),
      has_depth_(false),
      has_odom_(false),
      has_evaluation_(false),
      recovery_active_(false),
      depth_timestamp_(0.0),
      odom_timestamp_(0.0),
      valid_depth_ratio_(0.0),
      last_evaluation_timestamp_(0.0),
      recovery_start_timestamp_(0.0),
      dropped_frames_(0) {
  if (!validConfig(config_)) {
    throw std::invalid_argument("invalid health monitor configuration");
  }
}

void HealthMonitor::observeDepth(double timestamp,
                                 double valid_depth_ratio) {
  valid_depth_ratio_ = valid_depth_ratio;
  if (!std::isfinite(timestamp) || !std::isfinite(valid_depth_ratio)) {
    has_depth_ = false;
    resetRecovery();
    return;
  }

  if (has_depth_ &&
      (timestamp < depth_timestamp_ ||
       intervalExceeds(timestamp, depth_timestamp_, config_.depth_timeout))) {
    resetRecovery();
  }
  if (valid_depth_ratio < config_.min_valid_depth_ratio ||
      valid_depth_ratio > 1.0) {
    resetRecovery();
  }

  depth_timestamp_ = timestamp;
  has_depth_ = true;
}

void HealthMonitor::observeOdom(double timestamp) {
  if (!std::isfinite(timestamp)) {
    has_odom_ = false;
    resetRecovery();
    return;
  }

  if (has_odom_ &&
      (timestamp < odom_timestamp_ ||
       intervalExceeds(timestamp, odom_timestamp_, config_.odom_timeout))) {
    resetRecovery();
  }

  odom_timestamp_ = timestamp;
  has_odom_ = true;
}

void HealthMonitor::noteDroppedFrame() {
  if (dropped_frames_ < std::numeric_limits<std::uint32_t>::max()) {
    ++dropped_frames_;
  }
}

HealthResult HealthMonitor::evaluate(double timestamp) {
  HealthResult result{false, false, false, false, valid_depth_ratio_,
                      "NOT_READY"};

  if (!std::isfinite(timestamp)) {
    resetRecovery();
    return result;
  }

  const bool evaluation_rolled_back =
      has_evaluation_ && timestamp < last_evaluation_timestamp_;
  has_evaluation_ = true;
  last_evaluation_timestamp_ = timestamp;

  const bool depth_fresh =
      has_depth_ &&
      intervalWithin(timestamp, depth_timestamp_, config_.depth_timeout);
  const bool odom_fresh =
      has_odom_ && intervalWithin(timestamp, odom_timestamp_, config_.odom_timeout);
  const bool depth_ratio_valid =
      has_depth_ && valid_depth_ratio_ >= config_.min_valid_depth_ratio &&
      valid_depth_ratio_ <= 1.0;

  result.depth_healthy = depth_fresh && depth_ratio_valid;
  result.odom_healthy = odom_fresh;

  if (!has_depth_ || !has_odom_) {
    resetRecovery();
    return result;
  }
  if (evaluation_rolled_back) {
    resetRecovery();
    result.depth_healthy = false;
    result.synchronized = false;
    result.fault_code = "DEPTH_TIMEOUT";
    return result;
  }
  if (!depth_fresh) {
    resetRecovery();
    result.depth_healthy = false;
    result.fault_code = "DEPTH_TIMEOUT";
    return result;
  }
  if (!odom_fresh) {
    resetRecovery();
    result.odom_healthy = false;
    result.fault_code = "ODOM_TIMEOUT";
    return result;
  }

  result.synchronized =
      std::fabs(depth_timestamp_ - odom_timestamp_) <=
      config_.max_sync_delta +
          timestampTolerance(depth_timestamp_, odom_timestamp_);
  if (!result.synchronized) {
    resetRecovery();
    result.fault_code = "SYNC_ERROR";
    return result;
  }

  if (!depth_ratio_valid) {
    resetRecovery();
    result.depth_healthy = false;
    result.fault_code = "DEPTH_INVALID";
    return result;
  }

  if (!recovery_active_) {
    recovery_active_ = true;
    recovery_start_timestamp_ = timestamp;
    return result;
  }
  if (!intervalReached(timestamp, recovery_start_timestamp_,
                       config_.recovery_window)) {
    return result;
  }

  result.healthy = true;
  result.fault_code = "OK";
  return result;
}

std::uint32_t HealthMonitor::droppedFrames() const { return dropped_frames_; }

void HealthMonitor::resetRecovery() { recovery_active_ = false; }

}  // namespace local_mapping
