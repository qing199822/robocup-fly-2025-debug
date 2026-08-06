#ifndef LOCAL_MAPPING_HEALTH_MONITOR_H_
#define LOCAL_MAPPING_HEALTH_MONITOR_H_

#include <cstdint>
#include <string>

namespace local_mapping {

struct HealthConfig {
  double max_sync_delta;
  double depth_timeout;
  double odom_timeout;
  double recovery_window;
  double min_valid_depth_ratio;
};

struct HealthResult {
  bool healthy;
  bool depth_healthy;
  bool odom_healthy;
  bool synchronized;
  double valid_depth_ratio;
  std::string fault_code;
};

class HealthMonitor {
 public:
  explicit HealthMonitor(const HealthConfig& config);

  void observeDepth(double timestamp, double valid_depth_ratio);
  void observeOdom(double timestamp);
  void noteDroppedFrame();
  HealthResult evaluate(double timestamp);
  std::uint32_t droppedFrames() const;

 private:
  void resetRecovery();

  HealthConfig config_;
  bool has_depth_;
  bool has_odom_;
  bool has_evaluation_;
  bool recovery_active_;
  double depth_timestamp_;
  double odom_timestamp_;
  double valid_depth_ratio_;
  double last_evaluation_timestamp_;
  double recovery_start_timestamp_;
  std::uint32_t dropped_frames_;
};

}  // namespace local_mapping

#endif  // LOCAL_MAPPING_HEALTH_MONITOR_H_
