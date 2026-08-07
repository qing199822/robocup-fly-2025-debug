#include "local_mapping/time_rollback_guard.h"

#include <cmath>

namespace local_mapping {

TimeRollbackGuard::TimeRollbackGuard()
    : has_timestamp_(false), last_timestamp_(0.0) {}

bool TimeRollbackGuard::observe(double timestamp) {
  if (!std::isfinite(timestamp)) {
    return false;
  }
  if (!has_timestamp_) {
    has_timestamp_ = true;
    last_timestamp_ = timestamp;
    return false;
  }

  const bool rolled_back = timestamp < last_timestamp_;
  last_timestamp_ = timestamp;
  return rolled_back;
}

}  // namespace local_mapping
