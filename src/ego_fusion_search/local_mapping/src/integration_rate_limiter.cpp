#include "local_mapping/integration_rate_limiter.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace local_mapping {
namespace {

double timestampTolerance(double first, double second) {
  return 8.0 * std::numeric_limits<double>::epsilon() *
         std::fmax(1.0, std::fmax(std::fabs(first), std::fabs(second)));
}

}  // namespace

IntegrationRateLimiter::IntegrationRateLimiter(double rate_hz)
    : period_(1.0 / rate_hz),
      has_integration_(false),
      last_integration_timestamp_(0.0) {
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0 ||
      !std::isfinite(period_)) {
    throw std::invalid_argument("invalid integration rate");
  }
}

bool IntegrationRateLimiter::due(double timestamp) const {
  if (!std::isfinite(timestamp)) {
    return false;
  }
  if (!has_integration_) {
    return true;
  }
  if (timestamp < last_integration_timestamp_) {
    return false;
  }
  return timestamp - last_integration_timestamp_ +
             timestampTolerance(timestamp, last_integration_timestamp_) >=
         period_;
}

void IntegrationRateLimiter::markIntegrated(double timestamp) {
  if (!std::isfinite(timestamp)) {
    throw std::invalid_argument("integration timestamp must be finite");
  }
  has_integration_ = true;
  last_integration_timestamp_ = timestamp;
}

void IntegrationRateLimiter::reset() {
  has_integration_ = false;
  last_integration_timestamp_ = 0.0;
}

}  // namespace local_mapping
