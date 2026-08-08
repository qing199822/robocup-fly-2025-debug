#include "ego_adapter/bspline_sampler.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ego_adapter {
namespace {

bool finite(const Vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

Vec3 add(const Vec3& first, const Vec3& second) {
  return Vec3{first.x + second.x, first.y + second.y, first.z + second.z};
}

Vec3 scale(const Vec3& value, double factor) {
  return Vec3{factor * value.x, factor * value.y, factor * value.z};
}

Vec3 deBoor(const std::vector<Vec3>& points,
            const std::vector<double>& knots, int degree, double parameter) {
  const std::size_t last_point = points.size() - 1u;
  std::size_t span = last_point;
  if (parameter < knots[last_point + 1u]) {
    const auto upper = std::upper_bound(
        knots.begin() + degree, knots.begin() + last_point + 2u, parameter);
    span = static_cast<std::size_t>(upper - knots.begin() - 1);
  }

  std::vector<Vec3> values;
  values.reserve(static_cast<std::size_t>(degree) + 1u);
  for (int index = 0; index <= degree; ++index) {
    values.push_back(points[span - static_cast<std::size_t>(degree) +
                            static_cast<std::size_t>(index)]);
  }
  for (int recursion = 1; recursion <= degree; ++recursion) {
    for (int index = degree; index >= recursion; --index) {
      const std::size_t left =
          static_cast<std::size_t>(index) + span - degree;
      const std::size_t right =
          static_cast<std::size_t>(index + 1) + span - recursion;
      const double denominator = knots[right] - knots[left];
      if (!std::isfinite(denominator) || denominator <= 0.0) {
        throw std::invalid_argument("invalid B-spline knot span");
      }
      const double alpha = (parameter - knots[left]) / denominator;
      values[static_cast<std::size_t>(index)] =
          add(scale(values[static_cast<std::size_t>(index - 1)],
                    1.0 - alpha),
              scale(values[static_cast<std::size_t>(index)], alpha));
    }
  }
  return values[static_cast<std::size_t>(degree)];
}

}  // namespace

BsplineSampler::BsplineSampler(const BsplineData& data) : data_(data) {
  if (data_.order < 1 || !std::isfinite(data_.start_time) ||
      data_.control_points.size() <
          static_cast<std::size_t>(data_.order + 1) ||
      data_.knots.size() !=
          data_.control_points.size() + static_cast<std::size_t>(data_.order) +
              1u) {
    throw std::invalid_argument("invalid B-spline dimensions");
  }
  for (const Vec3& point : data_.control_points) {
    if (!finite(point)) {
      throw std::invalid_argument("non-finite B-spline control point");
    }
  }
  for (std::size_t index = 0; index < data_.knots.size(); ++index) {
    if (!std::isfinite(data_.knots[index]) ||
        (index > 0u && data_.knots[index] < data_.knots[index - 1u])) {
      throw std::invalid_argument("invalid B-spline knots");
    }
  }

  parameter_start_ = data_.knots[static_cast<std::size_t>(data_.order)];
  parameter_end_ = data_.knots[data_.control_points.size()];
  if (!(parameter_end_ > parameter_start_)) {
    throw std::invalid_argument("empty B-spline time domain");
  }

  derivative_points_.reserve(data_.control_points.size() - 1u);
  for (std::size_t index = 0u; index + 1u < data_.control_points.size();
       ++index) {
    const double denominator =
        data_.knots[index + static_cast<std::size_t>(data_.order) + 1u] -
        data_.knots[index + 1u];
    if (!std::isfinite(denominator) || denominator <= 0.0) {
      throw std::invalid_argument("invalid B-spline derivative knots");
    }
    const Vec3 difference{
        data_.control_points[index + 1u].x - data_.control_points[index].x,
        data_.control_points[index + 1u].y - data_.control_points[index].y,
        data_.control_points[index + 1u].z - data_.control_points[index].z};
    derivative_points_.push_back(
        scale(difference, static_cast<double>(data_.order) / denominator));
  }
  derivative_knots_.assign(data_.knots.begin() + 1, data_.knots.end() - 1);
}

BsplineState BsplineSampler::evaluate(double absolute_time) const {
  if (!std::isfinite(absolute_time)) {
    throw std::invalid_argument("non-finite B-spline sample time");
  }
  const double relative_time = absolute_time - data_.start_time;
  const double duration = parameter_end_ - parameter_start_;
  if (relative_time < 0.0 || relative_time > duration) {
    throw std::out_of_range("B-spline sample time is outside trajectory");
  }
  const double parameter = parameter_start_ + relative_time;
  return BsplineState{
      deBoor(data_.control_points, data_.knots, data_.order, parameter),
      deBoor(derivative_points_, derivative_knots_, data_.order - 1,
             parameter)};
}

double BsplineSampler::startTime() const { return data_.start_time; }

double BsplineSampler::endTime() const {
  return data_.start_time + parameter_end_ - parameter_start_;
}

std::int64_t BsplineSampler::trajectoryId() const { return data_.traj_id; }

}  // namespace ego_adapter
