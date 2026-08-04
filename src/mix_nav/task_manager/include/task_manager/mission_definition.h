#pragma once

#include <istream>
#include <string>
#include <vector>

namespace task_manager {

struct Waypoint {
  double x;
  double y;
  double z;
};

struct MissionDefinition {
  std::string vehicle_id;
  std::vector<Waypoint> entry_waypoints;
  std::vector<Waypoint> patrol_waypoints;
};

std::vector<MissionDefinition> loadMissionDefinitions(
    std::istream& input,
    const std::vector<std::string>& requested_vehicle_ids);

}  // namespace task_manager
