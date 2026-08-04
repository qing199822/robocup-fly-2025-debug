#include "task_manager/mission_definition.h"

#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

#include <json/json.h>

namespace task_manager {
namespace {

Waypoint parseWaypoint(const Json::Value& value,
                       const std::string& context) {
  if (!value.isObject() || !value.isMember("x") ||
      !value.isMember("y") || !value.isMember("z") ||
      !value["x"].isNumeric() || !value["y"].isNumeric() ||
      !value["z"].isNumeric()) {
    throw std::runtime_error(context + " must contain numeric x/y/z");
  }

  Waypoint point{value["x"].asDouble(), value["y"].asDouble(),
                 value["z"].asDouble()};
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z)) {
    throw std::runtime_error(context + " contains a non-finite value");
  }
  return point;
}

std::vector<Waypoint> parseWaypointArray(const Json::Value& value,
                                         const std::string& context,
                                         bool require_non_empty) {
  if (!value.isArray() || (require_non_empty && value.empty())) {
    throw std::runtime_error(context + " must be a non-empty array");
  }

  std::vector<Waypoint> points;
  for (Json::ArrayIndex index = 0; index < value.size(); ++index) {
    points.push_back(parseWaypoint(
        value[index], context + "[" + std::to_string(index) + "]"));
  }
  return points;
}

}  // namespace

std::vector<MissionDefinition> loadMissionDefinitions(
    std::istream& input,
    const std::vector<std::string>& requested_vehicle_ids) {
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  if (!Json::parseFromStream(builder, input, &root, &errors) ||
      !root.isArray()) {
    throw std::runtime_error("mission root must be a JSON array: " + errors);
  }

  std::map<std::string, MissionDefinition> by_id;
  for (const auto& item : root) {
    if (!item.isObject() || !item["vehicle_id"].isString() ||
        item["vehicle_id"].asString().empty()) {
      throw std::runtime_error(
          "mission vehicle_id must be a non-empty string");
    }

    MissionDefinition mission;
    mission.vehicle_id = item["vehicle_id"].asString();
    if (by_id.count(mission.vehicle_id) != 0) {
      throw std::runtime_error("duplicate vehicle_id: " +
                               mission.vehicle_id);
    }
    if (item.isMember("entry_waypoints")) {
      mission.entry_waypoints = parseWaypointArray(
          item["entry_waypoints"],
          mission.vehicle_id + ".entry_waypoints", true);
    }
    if (!item.isMember("waypoints")) {
      throw std::runtime_error(mission.vehicle_id +
                               ".waypoints is missing");
    }
    mission.patrol_waypoints = parseWaypointArray(
        item["waypoints"], mission.vehicle_id + ".waypoints", true);
    by_id.emplace(mission.vehicle_id, mission);
  }

  std::set<std::string> requested_once;
  std::vector<MissionDefinition> result;
  for (const auto& id : requested_vehicle_ids) {
    if (!requested_once.insert(id).second) {
      throw std::runtime_error("requested vehicle_id is duplicated: " + id);
    }
    const auto found = by_id.find(id);
    if (found == by_id.end()) {
      throw std::runtime_error("requested vehicle_id is missing: " + id);
    }
    result.push_back(found->second);
  }
  return result;
}

}  // namespace task_manager
