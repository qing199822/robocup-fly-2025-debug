#include "local_mapping/frontier_selector.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace local_mapping {
namespace {

constexpr double kQuaternionTolerance = 1e-9;
constexpr double kScoreTolerance = 1e-12;

FrontierGoal invalidGoal() {
  return FrontierGoal{false, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

bool finiteVec3(const Vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

void validateGrid(const nav_msgs::OccupancyGrid& grid, const Vec3& robot) {
  if (!std::isfinite(grid.info.resolution) || grid.info.resolution <= 0.0) {
    throw std::invalid_argument("grid resolution must be finite and positive");
  }

  const std::size_t width = grid.info.width;
  const std::size_t height = grid.info.height;
  if (height != 0 && width > std::numeric_limits<std::size_t>::max() / height) {
    throw std::invalid_argument("grid dimensions overflow size_t");
  }
  const std::size_t cell_count = width * height;
  if (cell_count != grid.data.size()) {
    throw std::invalid_argument("grid data size does not match dimensions");
  }
  if (width > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      height > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("grid dimensions exceed goal cell range");
  }

  const auto& position = grid.info.origin.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !finiteVec3(robot)) {
    throw std::invalid_argument("grid origin and robot must be finite");
  }

  const auto& orientation = grid.info.origin.orientation;
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w)) {
    throw std::invalid_argument("grid orientation must be finite");
  }
  const double norm_squared =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(norm_squared) ||
      std::abs(norm_squared - 1.0) > kQuaternionTolerance) {
    throw std::invalid_argument("grid orientation must be a unit quaternion");
  }
  if (std::abs(orientation.x) > kQuaternionTolerance ||
      std::abs(orientation.y) > kQuaternionTolerance ||
      std::abs(orientation.z) > kQuaternionTolerance ||
      std::abs(std::abs(orientation.w) - 1.0) > kQuaternionTolerance) {
    throw std::invalid_argument("rotated occupancy grids are unsupported");
  }
}

bool isFrontier(const nav_msgs::OccupancyGrid& grid, std::size_t x,
                std::size_t y) {
  const std::size_t width = grid.info.width;
  const std::size_t height = grid.info.height;
  if (grid.data[y * width + x] != 0) {
    return false;
  }
  return (x > 0 && grid.data[y * width + x - 1] == -1) ||
         (x + 1 < width && grid.data[y * width + x + 1] == -1) ||
         (y > 0 && grid.data[(y - 1) * width + x] == -1) ||
         (y + 1 < height && grid.data[(y + 1) * width + x] == -1);
}

struct Candidate {
  FrontierGoal goal;
  std::size_t cluster_size;
  double score;
};

bool betterCandidate(const Candidate& candidate, const Candidate& current) {
  if (!current.goal.valid) {
    return true;
  }
  if (std::abs(candidate.score - current.score) > kScoreTolerance) {
    return candidate.score > current.score;
  }
  if (candidate.cluster_size != current.cluster_size) {
    return candidate.cluster_size > current.cluster_size;
  }
  if (std::abs(candidate.goal.distance_from_robot -
               current.goal.distance_from_robot) > kScoreTolerance) {
    return candidate.goal.distance_from_robot <
           current.goal.distance_from_robot;
  }
  if (candidate.goal.cell_y != current.goal.cell_y) {
    return candidate.goal.cell_y < current.goal.cell_y;
  }
  return candidate.goal.cell_x < current.goal.cell_x;
}

Candidate candidateForCell(const nav_msgs::OccupancyGrid& grid,
                           std::size_t index, std::size_t cluster_size,
                           const Vec3& robot) {
  const std::size_t width = grid.info.width;
  const std::size_t x = index % width;
  const std::size_t y = index / width;
  const double resolution = grid.info.resolution;
  const double world_x =
      grid.info.origin.position.x + (static_cast<double>(x) + 0.5) * resolution;
  const double world_y =
      grid.info.origin.position.y + (static_cast<double>(y) + 0.5) * resolution;
  const double delta_x = world_x - robot.x;
  const double delta_y = world_y - robot.y;
  const double distance = std::hypot(delta_x, delta_y);
  const double yaw = distance == 0.0 ? 0.0 : std::atan2(delta_y, delta_x);
  const double normalized_yaw = std::atan2(std::sin(yaw), std::cos(yaw));

  // This API has no robot heading, so yaw change is measured from map +X.
  const double score = static_cast<double>(cluster_size) - 0.5 * distance -
                       0.2 * std::abs(normalized_yaw);
  return Candidate{FrontierGoal{true,
                                static_cast<int>(x),
                                static_cast<int>(y),
                                world_x,
                                world_y,
                                robot.z,
                                yaw,
                                distance},
                   cluster_size,
                   score};
}

}  // namespace

FrontierSelector::FrontierSelector(int min_cluster_cells, double max_distance)
    : min_cluster_cells_(min_cluster_cells), max_distance_(max_distance) {
  if (min_cluster_cells_ <= 0) {
    throw std::invalid_argument(
        "minimum frontier cluster size must be positive");
  }
  if (!std::isfinite(max_distance_) || max_distance_ < 0.0) {
    throw std::invalid_argument("maximum frontier distance must be finite");
  }
}

FrontierGoal FrontierSelector::select(const nav_msgs::OccupancyGrid& grid,
                                      const Vec3& robot) const {
  validateGrid(grid, robot);
  const std::size_t width = grid.info.width;
  const std::size_t height = grid.info.height;
  if (width == 0 || height == 0) {
    return invalidGoal();
  }

  const std::size_t cell_count = width * height;
  std::vector<std::uint8_t> frontier(cell_count, 0);
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      frontier[y * width + x] = isFrontier(grid, x, y) ? 1 : 0;
    }
  }

  std::vector<std::uint8_t> visited(cell_count, 0);
  Candidate best{invalidGoal(), 0, 0.0};
  for (std::size_t start = 0; start < cell_count; ++start) {
    if (frontier[start] == 0 || visited[start] != 0) {
      continue;
    }

    std::vector<std::size_t> cluster;
    cluster.push_back(start);
    visited[start] = 1;
    for (std::size_t cursor = 0; cursor < cluster.size(); ++cursor) {
      const std::size_t index = cluster[cursor];
      const std::int64_t x = static_cast<std::int64_t>(index % width);
      const std::int64_t y = static_cast<std::int64_t>(index / width);
      for (std::int64_t delta_y = -1; delta_y <= 1; ++delta_y) {
        for (std::int64_t delta_x = -1; delta_x <= 1; ++delta_x) {
          if (delta_x == 0 && delta_y == 0) {
            continue;
          }
          const std::int64_t neighbor_x = x + delta_x;
          const std::int64_t neighbor_y = y + delta_y;
          if (neighbor_x < 0 || neighbor_y < 0 ||
              neighbor_x >= static_cast<std::int64_t>(width) ||
              neighbor_y >= static_cast<std::int64_t>(height)) {
            continue;
          }
          const std::size_t neighbor =
              static_cast<std::size_t>(neighbor_y) * width +
              static_cast<std::size_t>(neighbor_x);
          if (frontier[neighbor] != 0 && visited[neighbor] == 0) {
            visited[neighbor] = 1;
            cluster.push_back(neighbor);
          }
        }
      }
    }

    if (cluster.size() < static_cast<std::size_t>(min_cluster_cells_)) {
      continue;
    }
    for (const std::size_t index : cluster) {
      const Candidate candidate =
          candidateForCell(grid, index, cluster.size(), robot);
      if (candidate.goal.distance_from_robot <= max_distance_ &&
          betterCandidate(candidate, best)) {
        best = candidate;
      }
    }
  }

  return best.goal;
}

}  // namespace local_mapping
