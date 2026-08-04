#include <sstream>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "task_manager/mission_definition.h"

TEST(MissionDefinition, ParsesOptionalEntryWaypoints) {
  std::istringstream input(R"json([
    {"vehicle_id":"typhoon_h480_0",
     "entry_waypoints":[{"x":1,"y":2,"z":3.5}],
     "waypoints":[{"x":4,"y":5,"z":3.5}]}
  ])json");
  const auto missions = task_manager::loadMissionDefinitions(
      input, {"typhoon_h480_0"});
  ASSERT_EQ(1u, missions.size());
  ASSERT_EQ(1u, missions[0].entry_waypoints.size());
  EXPECT_DOUBLE_EQ(1.0, missions[0].entry_waypoints[0].x);
  EXPECT_DOUBLE_EQ(4.0, missions[0].patrol_waypoints[0].x);
}

TEST(MissionDefinition, KeepsLegacyMissionWithoutEntryCompatible) {
  std::istringstream input(R"json([
    {"vehicle_id":"typhoon_h480_0",
     "waypoints":[{"x":4,"y":5,"z":3.5}]}
  ])json");
  const auto missions = task_manager::loadMissionDefinitions(
      input, {"typhoon_h480_0"});
  EXPECT_TRUE(missions[0].entry_waypoints.empty());
}

TEST(MissionDefinition, RejectsDuplicateVehicleIds) {
  std::istringstream input(R"json([
    {"vehicle_id":"typhoon_h480_0","waypoints":[{"x":1,"y":2,"z":3}]},
    {"vehicle_id":"typhoon_h480_0","waypoints":[{"x":4,"y":5,"z":3}]}
  ])json");
  EXPECT_THROW(task_manager::loadMissionDefinitions(
                   input, {"typhoon_h480_0"}),
               std::runtime_error);
}

TEST(MissionDefinition, RejectsMissingRequestedVehicleBeforeAnyThreadStarts) {
  std::istringstream input(R"json([
    {"vehicle_id":"typhoon_h480_0","waypoints":[{"x":1,"y":2,"z":3}]}
  ])json");
  EXPECT_THROW(task_manager::loadMissionDefinitions(
                   input, {"typhoon_h480_0", "typhoon_h480_1"}),
               std::runtime_error);
}

TEST(MissionDefinition, RejectsMalformedWaypointsAndEmptyArrays) {
  std::istringstream bad_coordinate(R"json([
    {"vehicle_id":"typhoon_h480_0","waypoints":[{"x":"bad","y":2,"z":3}]}
  ])json");
  EXPECT_THROW(task_manager::loadMissionDefinitions(
                   bad_coordinate, {"typhoon_h480_0"}),
               std::runtime_error);

  std::istringstream empty_entry(R"json([
    {"vehicle_id":"typhoon_h480_0","entry_waypoints":[],
     "waypoints":[{"x":1,"y":2,"z":3}]}
  ])json");
  EXPECT_THROW(task_manager::loadMissionDefinitions(
                   empty_entry, {"typhoon_h480_0"}),
               std::runtime_error);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
