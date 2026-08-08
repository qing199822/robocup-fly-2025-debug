#include "local_mapping/voxel_map.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace local_mapping {
namespace {

bool isFinite(const Vec3& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

void incrementSaturated(std::uint32_t* value) {
  if (*value < std::numeric_limits<std::uint32_t>::max()) {
    ++(*value);
  }
}

}  // namespace

bool VoxelMap::Key::operator<(const Key& other) const {
  if (x != other.x) {
    return x < other.x;
  }
  if (y != other.y) {
    return y < other.y;
  }
  return z < other.z;
}

bool VoxelMap::Key::operator==(const Key& other) const {
  return x == other.x && y == other.y && z == other.z;
}

std::size_t VoxelMap::KeyHash::operator()(const Key& key) const {
  std::size_t result = std::hash<std::int64_t>{}(key.x);
  result ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9u +
            (result << 6u) + (result >> 2u);
  result ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9u +
            (result << 6u) + (result >> 2u);
  return result;
}

VoxelMap::VoxelMap(double resolution, int occupied_hits, int free_hits,
                   double dynamic_ttl)
    : resolution_(resolution),
      occupied_hits_(occupied_hits > 0
                         ? static_cast<std::uint32_t>(occupied_hits)
                         : 0u),
      free_hits_(free_hits > 0 ? static_cast<std::uint32_t>(free_hits) : 0u),
      dynamic_ttl_(dynamic_ttl) {
  if (!std::isfinite(resolution_) || resolution_ <= 0.0 ||
      occupied_hits <= 0 || free_hits <= 0 ||
      !std::isfinite(dynamic_ttl_) || dynamic_ttl_ <= 0.0) {
    throw std::invalid_argument("invalid voxel map configuration");
  }
}

VoxelMap::Key VoxelMap::keyFor(const Vec3& point) const {
  if (!isFinite(point)) {
    throw std::invalid_argument("voxel coordinate must be finite");
  }

  const auto coordinateKey = [this](double coordinate) {
    const long double scaled = std::floor(
        static_cast<long double>(coordinate) /
        static_cast<long double>(resolution_));
    const long double minimum =
        static_cast<long double>(std::numeric_limits<std::int64_t>::min());
    const long double maximum =
        static_cast<long double>(std::numeric_limits<std::int64_t>::max());
    if (!std::isfinite(scaled) || scaled < minimum || scaled > maximum) {
      throw std::invalid_argument("voxel coordinate is outside key range");
    }
    return static_cast<std::int64_t>(scaled);
  };

  return Key{coordinateKey(point.x), coordinateKey(point.y),
             coordinateKey(point.z)};
}

Vec3 VoxelMap::centreFor(const Key& key) const {
  const long double resolution = resolution_;
  const long double x =
      (static_cast<long double>(key.x) + 0.5L) * resolution;
  const long double y =
      (static_cast<long double>(key.y) + 0.5L) * resolution;
  const long double z =
      (static_cast<long double>(key.z) + 0.5L) * resolution;
  const long double double_limit = std::numeric_limits<double>::max();
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      std::fabs(x) > double_limit || std::fabs(y) > double_limit ||
      std::fabs(z) > double_limit) {
    throw std::invalid_argument("voxel centre is outside coordinate range");
  }
  return Vec3{static_cast<double>(x), static_cast<double>(y),
              static_cast<double>(z)};
}

std::size_t VoxelMap::sampleCount(long double distance) const {
  if (!std::isfinite(distance) || distance < 0.0L) {
    throw std::invalid_argument("sampling distance is invalid");
  }
  const long double count = std::ceil(
      distance / (static_cast<long double>(resolution_) / 2.0L));
  if (count >=
      static_cast<long double>(std::numeric_limits<std::size_t>::max())) {
    throw std::invalid_argument("sampling distance is too large");
  }
  return static_cast<std::size_t>(count);
}

void VoxelMap::integrateStaticRay(const Vec3& origin,
                                  const Vec3& endpoint) {
  integrateStaticRays(origin, {endpoint});
}

void VoxelMap::integrateStaticRays(const Vec3& origin,
                                   const std::vector<Vec3>& endpoints) {
  keyFor(origin);

  std::unordered_map<Key, Vec3, KeyHash> unique_endpoints;
  unique_endpoints.reserve(endpoints.size());
  for (const auto& endpoint : endpoints) {
    unique_endpoints.emplace(keyFor(endpoint), endpoint);
  }

  std::unordered_set<Key, KeyHash> occupied_keys;
  occupied_keys.reserve(unique_endpoints.size());
  for (const auto& endpoint : unique_endpoints) {
    occupied_keys.insert(endpoint.first);
  }

  std::unordered_set<Key, KeyHash> free_keys;
  for (const auto& endpoint_entry : unique_endpoints) {
    const Vec3& endpoint = endpoint_entry.second;
    const long double dx = static_cast<long double>(endpoint.x) - origin.x;
    const long double dy = static_cast<long double>(endpoint.y) - origin.y;
    const long double dz = static_cast<long double>(endpoint.z) - origin.z;
    const long double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const std::size_t samples = sampleCount(distance);

    if (samples > 0u) {
      for (std::size_t index = 0; index < samples; ++index) {
        const long double fraction =
            static_cast<long double>(index) / samples;
        const Vec3 sample{
            static_cast<double>(static_cast<long double>(origin.x) +
                                fraction * dx),
            static_cast<double>(static_cast<long double>(origin.y) +
                                fraction * dy),
            static_cast<double>(static_cast<long double>(origin.z) +
                                fraction * dz)};
        const Key key = keyFor(sample);
        if (occupied_keys.count(key) == 0u) {
          free_keys.insert(key);
        }
      }
    }
  }

  for (const auto& key : free_keys) {
    incrementSaturated(&static_cells_[key].free);
  }
  for (const auto& key : occupied_keys) {
    incrementSaturated(&static_cells_[key].occupied);
  }
}

void VoxelMap::integrateDynamicPoint(const Vec3& point, double stamp) {
  if (!std::isfinite(stamp)) {
    throw std::invalid_argument("dynamic timestamp must be finite");
  }
  const Key key = keyFor(point);
  const auto found = dynamic_cells_.find(key);
  if (found == dynamic_cells_.end()) {
    dynamic_cells_.emplace(key, stamp);
  } else {
    found->second = std::max(found->second, stamp);
  }
}

void VoxelMap::clear() {
  static_cells_.clear();
  dynamic_cells_.clear();
}

CellState VoxelMap::staticState(const Key& key) const {
  const auto found = static_cells_.find(key);
  if (found == static_cells_.end()) {
    return CellState::UNKNOWN;
  }
  if (found->second.occupied >= occupied_hits_) {
    return CellState::OCCUPIED;
  }
  if (found->second.free >= free_hits_) {
    return CellState::FREE;
  }
  return CellState::UNKNOWN;
}

CellState VoxelMap::stateForKey(const Key& key, double now) const {
  const auto dynamic = dynamic_cells_.find(key);
  if (dynamic != dynamic_cells_.end() &&
      (now <= dynamic->second || now - dynamic->second <= dynamic_ttl_)) {
    return CellState::OCCUPIED;
  }
  return staticState(key);
}

void VoxelMap::pruneExpiredDynamic(double now) const {
  auto cell = dynamic_cells_.begin();
  while (cell != dynamic_cells_.end()) {
    if (now > cell->second && now - cell->second > dynamic_ttl_) {
      cell = dynamic_cells_.erase(cell);
    } else {
      ++cell;
    }
  }
}

CellState VoxelMap::stateAt(const Vec3& point, double now) const {
  if (!std::isfinite(now)) {
    throw std::invalid_argument("query timestamp must be finite");
  }
  return stateForKey(keyFor(point), now);
}

double VoxelMap::entryDistance(const Vec3& origin, const Vec3& unit_axis,
                               const Key& key) const {
  const long double resolution = resolution_;
  long double entry = 0.0L;
  const auto updateEntry = [&entry, resolution](double coordinate,
                                                 double direction,
                                                 std::int64_t key_value) {
    if (direction > 0.0) {
      const long double lower =
          static_cast<long double>(key_value) * resolution;
      entry = std::max(entry, (lower - coordinate) / direction);
    } else if (direction < 0.0) {
      const long double upper =
          (static_cast<long double>(key_value) + 1.0L) * resolution;
      entry = std::max(entry, (upper - coordinate) / direction);
    }
  };

  updateEntry(origin.x, unit_axis.x, key.x);
  updateEntry(origin.y, unit_axis.y, key.y);
  updateEntry(origin.z, unit_axis.z, key.z);
  return std::max(0.0, static_cast<double>(entry));
}

Clearance VoxelMap::axisClearance(const Vec3& origin, const Vec3& unit_axis,
                                  double max_distance, double now) const {
  if (!isFinite(origin) || !isFinite(unit_axis) ||
      !std::isfinite(max_distance) || max_distance < 0.0 ||
      !std::isfinite(now)) {
    throw std::invalid_argument("invalid clearance query");
  }

  const bool x_axis =
      (unit_axis.x == 1.0 || unit_axis.x == -1.0) && unit_axis.y == 0.0 &&
      unit_axis.z == 0.0;
  const bool y_axis =
      unit_axis.x == 0.0 &&
      (unit_axis.y == 1.0 || unit_axis.y == -1.0) && unit_axis.z == 0.0;
  const bool z_axis =
      unit_axis.x == 0.0 && unit_axis.y == 0.0 &&
      (unit_axis.z == 1.0 || unit_axis.z == -1.0);
  if (!(x_axis || y_axis || z_axis)) {
    throw std::invalid_argument(
        "clearance axis must be an axis-aligned unit vector");
  }

  const long double endpoint_x =
      static_cast<long double>(origin.x) +
      static_cast<long double>(unit_axis.x) * max_distance;
  const long double endpoint_y =
      static_cast<long double>(origin.y) +
      static_cast<long double>(unit_axis.y) * max_distance;
  const long double endpoint_z =
      static_cast<long double>(origin.z) +
      static_cast<long double>(unit_axis.z) * max_distance;
  const long double double_limit = std::numeric_limits<double>::max();
  if (!std::isfinite(endpoint_x) || !std::isfinite(endpoint_y) ||
      !std::isfinite(endpoint_z) || std::fabs(endpoint_x) > double_limit ||
      std::fabs(endpoint_y) > double_limit ||
      std::fabs(endpoint_z) > double_limit) {
    throw std::invalid_argument(
        "clearance endpoint is outside coordinate range");
  }
  keyFor({static_cast<double>(endpoint_x), static_cast<double>(endpoint_y),
          static_cast<double>(endpoint_z)});

  const std::size_t samples = sampleCount(max_distance);
  std::set<Key> visited;
  bool occupied_found = false;
  double first_occupied_distance = 0.0;
  for (std::size_t index = 0; index <= samples; ++index) {
    const long double distance =
        samples == 0u
            ? 0.0L
            : static_cast<long double>(max_distance) * index / samples;
    const Vec3 sample{
        static_cast<double>(static_cast<long double>(origin.x) +
                            distance * unit_axis.x),
        static_cast<double>(static_cast<long double>(origin.y) +
                            distance * unit_axis.y),
        static_cast<double>(static_cast<long double>(origin.z) +
                            distance * unit_axis.z)};
    const Key key = keyFor(sample);
    if (!visited.insert(key).second) {
      continue;
    }

    const CellState state = stateForKey(key, now);
    if (state == CellState::UNKNOWN) {
      return Clearance{false, 0.0};
    }
    if (state == CellState::OCCUPIED && !occupied_found) {
      occupied_found = true;
      first_occupied_distance = entryDistance(origin, unit_axis, key);
    }
  }
  if (occupied_found) {
    return Clearance{true, first_occupied_distance};
  }
  return Clearance{true, max_distance};
}

SweepResult VoxelMap::validateSweptVolume(
    const std::vector<Vec3>& samples, double horizontal_radius,
    double vertical_radius, double safety_margin, double now) const {
  return validateSweptVolumeImpl(samples, horizontal_radius, vertical_radius,
                                 safety_margin, now, nullptr);
}

SweepResult VoxelMap::validateSweptVolume(
    const std::vector<Vec3>& samples, double horizontal_radius,
    double vertical_radius, double safety_margin, double now,
    const Vec3& trusted_start) const {
  if (!isFinite(trusted_start)) {
    throw std::invalid_argument("trusted swept volume start must be finite");
  }
  return validateSweptVolumeImpl(samples, horizontal_radius, vertical_radius,
                                 safety_margin, now, &trusted_start);
}

SweepResult VoxelMap::validateSweptVolumeImpl(
    const std::vector<Vec3>& samples, double horizontal_radius,
    double vertical_radius, double safety_margin, double now,
    const Vec3* trusted_start) const {
  if (samples.empty()) {
    throw std::invalid_argument("swept volume samples must not be empty");
  }
  if (!std::isfinite(horizontal_radius) || horizontal_radius < 0.0 ||
      !std::isfinite(vertical_radius) || vertical_radius < 0.0 ||
      !std::isfinite(safety_margin) || safety_margin < 0.0 ||
      !std::isfinite(now)) {
    throw std::invalid_argument("invalid swept volume query");
  }
  for (const Vec3& sample : samples) {
    if (!isFinite(sample)) {
      throw std::invalid_argument("swept volume samples must be finite");
    }
  }

  const double radius = horizontal_radius + safety_margin;
  const double half_height = vertical_radius + safety_margin;
  if (!std::isfinite(radius) || !std::isfinite(half_height)) {
    throw std::invalid_argument("swept volume dimensions are too large");
  }

  std::vector<std::size_t> interpolation_counts;
  interpolation_counts.reserve(samples.size() - 1u);
  for (std::size_t segment = 1u; segment < samples.size(); ++segment) {
    const Vec3& start = samples[segment - 1u];
    const Vec3& end = samples[segment];
    const long double dx =
        static_cast<long double>(end.x) - static_cast<long double>(start.x);
    const long double dy =
        static_cast<long double>(end.y) - static_cast<long double>(start.y);
    const long double dz =
        static_cast<long double>(end.z) - static_cast<long double>(start.z);
    interpolation_counts.push_back(
        sampleCount(std::sqrt(dx * dx + dy * dy + dz * dz)));
  }

  const auto validate_centre = [this, radius, half_height, safety_margin, now,
                                trusted_start](const Vec3& centre) {
    const Key lower = keyFor({centre.x - radius, centre.y - radius,
                              centre.z - half_height});
    const Key upper = keyFor({centre.x + radius, centre.y + radius,
                              centre.z + half_height});
    const double half_voxel = resolution_ / 2.0;

    for (std::int64_t x = lower.x;; ++x) {
      for (std::int64_t y = lower.y;; ++y) {
        for (std::int64_t z = lower.z;; ++z) {
          const Key key{x, y, z};
          const Vec3 voxel_centre = centreFor(key);
          const double dx = std::max(
              0.0, std::abs(voxel_centre.x - centre.x) - half_voxel);
          const double dy = std::max(
              0.0, std::abs(voxel_centre.y - centre.y) - half_voxel);
          const double dz = std::max(
              0.0, std::abs(voxel_centre.z - centre.z) - half_voxel);
          if (std::hypot(dx, dy) <= radius && dz <= half_height) {
            const CellState state = stateForKey(key, now);
            const double trusted_dx =
                trusted_start == nullptr
                    ? std::numeric_limits<double>::infinity()
                    : std::max(0.0,
                               std::abs(voxel_centre.x - trusted_start->x) -
                                   half_voxel);
            const double trusted_dy =
                trusted_start == nullptr
                    ? std::numeric_limits<double>::infinity()
                    : std::max(0.0,
                               std::abs(voxel_centre.y - trusted_start->y) -
                                   half_voxel);
            const double trusted_dz =
                trusted_start == nullptr
                    ? std::numeric_limits<double>::infinity()
                    : std::max(0.0,
                               std::abs(voxel_centre.z - trusted_start->z) -
                                   half_voxel);
            const bool unknown_is_inside_trusted_start =
                trusted_start != nullptr &&
                std::hypot(trusted_dx, trusted_dy) <= radius &&
                trusted_dz <= half_height;
            if (state == CellState::OCCUPIED ||
                (state == CellState::UNKNOWN &&
                 !unknown_is_inside_trusted_start)) {
              return SweepResult{
                  false,
                  state == CellState::UNKNOWN ? SweepFault::UNKNOWN
                                              : SweepFault::OCCUPIED,
                  0.0};
            }
          }
          if (z == upper.z) {
            break;
          }
        }
        if (y == upper.y) {
          break;
        }
      }
      if (x == upper.x) {
        break;
      }
    }
    return SweepResult{true, SweepFault::NONE, safety_margin};
  };

  SweepResult result = validate_centre(samples.front());
  if (!result.valid) {
    return result;
  }
  for (std::size_t segment = 1; segment < samples.size(); ++segment) {
    const Vec3& start = samples[segment - 1u];
    const Vec3& end = samples[segment];
    const long double dx =
        static_cast<long double>(end.x) - static_cast<long double>(start.x);
    const long double dy =
        static_cast<long double>(end.y) - static_cast<long double>(start.y);
    const long double dz =
        static_cast<long double>(end.z) - static_cast<long double>(start.z);
    const std::size_t interpolation_count = interpolation_counts[segment - 1u];
    for (std::size_t index = 1u; index <= interpolation_count; ++index) {
      const long double fraction =
          static_cast<long double>(index) / interpolation_count;
      const Vec3 centre{
          static_cast<double>(static_cast<long double>(start.x) +
                              fraction * dx),
          static_cast<double>(static_cast<long double>(start.y) +
                              fraction * dy),
          static_cast<double>(static_cast<long double>(start.z) +
                              fraction * dz)};
      result = validate_centre(centre);
      if (!result.valid) {
        return result;
      }
    }
  }
  return SweepResult{true, SweepFault::NONE, safety_margin};
}

std::vector<Vec3> VoxelMap::staticOccupiedPoints(double now) const {
  if (!std::isfinite(now)) {
    throw std::invalid_argument("query timestamp must be finite");
  }
  std::vector<Vec3> points;
  for (const auto& cell : static_cells_) {
    if (cell.second.occupied >= occupied_hits_) {
      points.push_back(centreFor(cell.first));
    }
  }
  return points;
}

std::vector<Vec3> VoxelMap::dynamicOccupiedPoints(double now) const {
  if (!std::isfinite(now)) {
    throw std::invalid_argument("query timestamp must be finite");
  }
  pruneExpiredDynamic(now);
  std::vector<Vec3> points;
  for (const auto& cell : dynamic_cells_) {
    points.push_back(centreFor(cell.first));
  }
  return points;
}

}  // namespace local_mapping
