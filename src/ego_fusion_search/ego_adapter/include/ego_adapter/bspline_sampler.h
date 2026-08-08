#pragma once

#include <cstdint>
#include <vector>

#include "ego_adapter/command_policy.h"

namespace ego_adapter {

struct BsplineData {
  int order{0};
  std::int64_t traj_id{0};
  double start_time{0.0};
  std::vector<double> knots;
  std::vector<Vec3> control_points;
};

struct BsplineState {
  Vec3 position;
  Vec3 velocity;
};

class BsplineSampler {
 public:
  explicit BsplineSampler(const BsplineData& data);

  BsplineState evaluate(double absolute_time) const;
  double startTime() const;
  double endTime() const;
  std::int64_t trajectoryId() const;

 private:
  BsplineData data_;
  std::vector<Vec3> derivative_points_;
  std::vector<double> derivative_knots_;
  double parameter_start_{0.0};
  double parameter_end_{0.0};
};

}  // namespace ego_adapter
