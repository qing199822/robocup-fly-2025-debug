#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>

#include "local_mapping/frontier_selector.h"

namespace {

using local_mapping::FrontierGoal;
using local_mapping::FrontierSelector;
using local_mapping::Vec3;

nav_msgs::OccupancyGrid makeGrid(std::uint32_t width, std::uint32_t height,
                                 double resolution, std::int8_t fill = 100) {
  nav_msgs::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * height, fill);
  return grid;
}

void setCell(nav_msgs::OccupancyGrid* grid, int x, int y, std::int8_t value) {
  grid->data[static_cast<std::size_t>(y) * grid->info.width + x] = value;
}

std::int8_t cellAt(const nav_msgs::OccupancyGrid& grid, int x, int y) {
  return grid.data[static_cast<std::size_t>(y) * grid.info.width + x];
}

bool hasInGridUnknownFourNeighbor(const nav_msgs::OccupancyGrid& grid, int x,
                                  int y) {
  const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (const auto& offset : offsets) {
    const int neighbor_x = x + offset[0];
    const int neighbor_y = y + offset[1];
    if (neighbor_x >= 0 && neighbor_y >= 0 &&
        neighbor_x < static_cast<int>(grid.info.width) &&
        neighbor_y < static_cast<int>(grid.info.height) &&
        cellAt(grid, neighbor_x, neighbor_y) == -1) {
      return true;
    }
  }
  return false;
}

void expectFullyInitializedInvalid(const FrontierGoal& goal) {
  EXPECT_FALSE(goal.valid);
  EXPECT_EQ(0, goal.cell_x);
  EXPECT_EQ(0, goal.cell_y);
  EXPECT_DOUBLE_EQ(0.0, goal.x);
  EXPECT_DOUBLE_EQ(0.0, goal.y);
  EXPECT_DOUBLE_EQ(0.0, goal.z);
  EXPECT_DOUBLE_EQ(0.0, goal.yaw);
  EXPECT_DOUBLE_EQ(0.0, goal.distance_from_robot);
}

TEST(FrontierSelector, GoalIsFreeAndAdjacentToInGridUnknown) {
  auto grid = makeGrid(5, 5, 1.0, -1);
  for (int y = 1; y <= 3; ++y) {
    for (int x = 1; x <= 2; ++x) {
      setCell(&grid, x, y, 0);
    }
  }

  const auto goal = FrontierSelector(2, 8.0).select(grid, {1.5, 2.5, 2.0});

  ASSERT_TRUE(goal.valid);
  EXPECT_LE(goal.distance_from_robot, 8.0);
  EXPECT_EQ(0, cellAt(grid, goal.cell_x, goal.cell_y));
  EXPECT_TRUE(hasInGridUnknownFourNeighbor(grid, goal.cell_x, goal.cell_y));
  EXPECT_DOUBLE_EQ(2.0, goal.z);
}

TEST(FrontierSelector, DiagonalFrontiersFormOneEightConnectedCluster) {
  auto grid = makeGrid(4, 4, 1.0);
  setCell(&grid, 1, 1, 0);
  setCell(&grid, 2, 2, 0);
  setCell(&grid, 1, 0, -1);
  setCell(&grid, 2, 3, -1);

  EXPECT_TRUE(FrontierSelector(2, 10.0).select(grid, {0.0, 0.0, 0.0}).valid);
  expectFullyInitializedInvalid(
      FrontierSelector(3, 10.0).select(grid, {0.0, 0.0, 0.0}));
}

TEST(FrontierSelector, IncludesCandidateExactlyAtMaximumDistance) {
  auto grid = makeGrid(3, 3, 1.0);
  setCell(&grid, 1, 1, 0);
  setCell(&grid, 1, 2, -1);

  const Vec3 robot{0.5, 1.5, 3.0};
  const auto at_limit = FrontierSelector(1, 1.0).select(grid, robot);
  ASSERT_TRUE(at_limit.valid);
  EXPECT_DOUBLE_EQ(1.0, at_limit.distance_from_robot);
  expectFullyInitializedInvalid(
      FrontierSelector(1, 0.99).select(grid, robot));
}

TEST(FrontierSelector, UnknownAndOccupiedCellsAreNeverGoals) {
  auto grid = makeGrid(3, 2, 1.0, -1);
  setCell(&grid, 0, 0, 100);
  setCell(&grid, 1, 0, 50);
  setCell(&grid, 2, 0, 1);

  expectFullyInitializedInvalid(
      FrontierSelector(1, 10.0).select(grid, {0.0, 0.0, 0.0}));
}

TEST(FrontierSelector, GridBoundaryDoesNotCountAsUnknown) {
  auto one_free_cell = makeGrid(1, 1, 1.0, 0);
  expectFullyInitializedInvalid(
      FrontierSelector(1, 10.0).select(one_free_cell, {0.0, 0.0, 0.0}));

  auto all_free = makeGrid(3, 3, 1.0, 0);
  expectFullyInitializedInvalid(
      FrontierSelector(1, 10.0).select(all_free, {0.0, 0.0, 0.0}));
}

TEST(FrontierSelector, UsesTranslatedCellCentreAndRobotSearchHeight) {
  auto grid = makeGrid(3, 3, 0.5);
  grid.info.origin.position.x = 10.0;
  grid.info.origin.position.y = -4.0;
  grid.info.origin.position.z = 7.0;
  setCell(&grid, 1, 1, 0);
  setCell(&grid, 1, 2, -1);

  const auto goal = FrontierSelector(1, 10.0).select(
      grid, {10.75, -2.25, 2.75});

  ASSERT_TRUE(goal.valid);
  EXPECT_DOUBLE_EQ(10.75, goal.x);
  EXPECT_DOUBLE_EQ(-3.25, goal.y);
  EXPECT_DOUBLE_EQ(2.75, goal.z);
  EXPECT_DOUBLE_EQ(1.0, goal.distance_from_robot);
  EXPECT_NEAR(-std::acos(-1.0) / 2.0, goal.yaw, 1e-12);
}

TEST(FrontierSelector, ScoreUsesYawChangeRelativeToMapPositiveX) {
  auto grid = makeGrid(7, 7, 1.0);
  setCell(&grid, 5, 3, 0);
  setCell(&grid, 5, 4, -1);
  setCell(&grid, 3, 5, 0);
  setCell(&grid, 4, 5, -1);

  const auto goal = FrontierSelector(1, 10.0).select(grid, {3.5, 3.5, 1.0});

  ASSERT_TRUE(goal.valid);
  EXPECT_EQ(5, goal.cell_x);
  EXPECT_EQ(3, goal.cell_y);
  EXPECT_DOUBLE_EQ(0.0, goal.yaw);
}

TEST(FrontierSelector, DeterministicTiePrefersLowerCellY) {
  auto grid = makeGrid(7, 7, 1.0);
  setCell(&grid, 3, 1, 0);
  setCell(&grid, 4, 1, -1);
  setCell(&grid, 3, 5, 0);
  setCell(&grid, 4, 5, -1);

  for (int iteration = 0; iteration < 20; ++iteration) {
    const auto goal =
        FrontierSelector(1, 10.0).select(grid, {3.5, 3.5, 1.0});
    ASSERT_TRUE(goal.valid);
    EXPECT_EQ(3, goal.cell_x);
    EXPECT_EQ(1, goal.cell_y);
  }
}

TEST(FrontierSelector, CoincidentGoalHasZeroYaw) {
  auto grid = makeGrid(3, 3, 1.0);
  setCell(&grid, 1, 1, 0);
  setCell(&grid, 1, 2, -1);

  const auto goal =
      FrontierSelector(1, 0.0).select(grid, {1.5, 1.5, 4.0});

  ASSERT_TRUE(goal.valid);
  EXPECT_DOUBLE_EQ(0.0, goal.distance_from_robot);
  EXPECT_DOUBLE_EQ(0.0, goal.yaw);
}

TEST(FrontierSelector, NoFrontierReturnsStableFullyInitializedResult) {
  const auto grid = makeGrid(2, 2, 1.0);
  const FrontierSelector selector(1, 5.0);

  expectFullyInitializedInvalid(selector.select(grid, {0.0, 0.0, 0.0}));
  expectFullyInitializedInvalid(selector.select(grid, {0.0, 0.0, 0.0}));
}

TEST(FrontierSelector, EmptyGridReturnsStableInvalidResult) {
  const auto grid = makeGrid(0, 0, 1.0);
  expectFullyInitializedInvalid(
      FrontierSelector(1, 5.0).select(grid, {0.0, 0.0, 0.0}));
}

TEST(FrontierSelector, AcceptsSignEquivalentIdentityOrientation) {
  auto grid = makeGrid(3, 3, 1.0);
  grid.info.origin.orientation.w = -1.0;
  setCell(&grid, 1, 1, 0);
  setCell(&grid, 1, 2, -1);

  EXPECT_TRUE(FrontierSelector(1, 5.0)
                  .select(grid, {1.5, 1.5, 0.0})
                  .valid);
}

TEST(FrontierSelector, RejectsInvalidConstructorArguments) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(FrontierSelector(0, 1.0), std::invalid_argument);
  EXPECT_THROW(FrontierSelector(-1, 1.0), std::invalid_argument);
  EXPECT_THROW(FrontierSelector(1, -0.01), std::invalid_argument);
  EXPECT_THROW(FrontierSelector(1, nan), std::invalid_argument);
  EXPECT_THROW(FrontierSelector(1, inf), std::invalid_argument);
}

TEST(FrontierSelector, RejectsMalformedGridDimensionsAndResolution) {
  const FrontierSelector selector(1, 5.0);
  auto wrong_size = makeGrid(2, 2, 1.0);
  wrong_size.data.pop_back();
  EXPECT_THROW(selector.select(wrong_size, {0.0, 0.0, 0.0}),
               std::invalid_argument);

  auto huge_dimensions = makeGrid(0, 0, 1.0);
  huge_dimensions.info.width = std::numeric_limits<std::uint32_t>::max();
  huge_dimensions.info.height = std::numeric_limits<std::uint32_t>::max();
  EXPECT_THROW(selector.select(huge_dimensions, {0.0, 0.0, 0.0}),
               std::invalid_argument);

  for (const double resolution :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()}) {
    auto invalid_resolution = makeGrid(0, 0, 1.0);
    invalid_resolution.info.resolution = resolution;
    EXPECT_THROW(selector.select(invalid_resolution, {0.0, 0.0, 0.0}),
                 std::invalid_argument);
  }
}

TEST(FrontierSelector, RejectsNonFiniteOriginAndRobotCoordinates) {
  const FrontierSelector selector(1, 5.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  auto invalid_origin = makeGrid(0, 0, 1.0);
  invalid_origin.info.origin.position.x = nan;
  EXPECT_THROW(selector.select(invalid_origin, {0.0, 0.0, 0.0}),
               std::invalid_argument);
  invalid_origin = makeGrid(0, 0, 1.0);
  invalid_origin.info.origin.position.z = inf;
  EXPECT_THROW(selector.select(invalid_origin, {0.0, 0.0, 0.0}),
               std::invalid_argument);

  const auto grid = makeGrid(0, 0, 1.0);
  EXPECT_THROW(selector.select(grid, {nan, 0.0, 0.0}),
               std::invalid_argument);
  EXPECT_THROW(selector.select(grid, {0.0, inf, 0.0}),
               std::invalid_argument);
  EXPECT_THROW(selector.select(grid, {0.0, 0.0, nan}),
               std::invalid_argument);
}

TEST(FrontierSelector, RejectsInvalidOrRotatedOriginOrientation) {
  const FrontierSelector selector(1, 5.0);
  auto invalid = makeGrid(0, 0, 1.0);
  invalid.info.origin.orientation.w = 0.0;
  EXPECT_THROW(selector.select(invalid, {0.0, 0.0, 0.0}),
               std::invalid_argument);

  invalid = makeGrid(0, 0, 1.0);
  invalid.info.origin.orientation.x =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(selector.select(invalid, {0.0, 0.0, 0.0}),
               std::invalid_argument);

  auto non_unit = makeGrid(0, 0, 1.0);
  non_unit.info.origin.orientation.w = 2.0;
  EXPECT_THROW(selector.select(non_unit, {0.0, 0.0, 0.0}),
               std::invalid_argument);

  auto rotated = makeGrid(0, 0, 1.0);
  rotated.info.origin.orientation.z = std::sqrt(0.5);
  rotated.info.origin.orientation.w = std::sqrt(0.5);
  EXPECT_THROW(selector.select(rotated, {0.0, 0.0, 0.0}),
               std::invalid_argument);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
