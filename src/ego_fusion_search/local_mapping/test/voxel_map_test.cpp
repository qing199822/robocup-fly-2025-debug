#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "local_mapping/voxel_map.h"

namespace {

using local_mapping::CellState;
using local_mapping::Vec3;
using local_mapping::VoxelMap;

void integrateTwice(VoxelMap* map, const Vec3& origin,
                    const Vec3& endpoint) {
  map->integrateStaticRay(origin, endpoint);
  map->integrateStaticRay(origin, endpoint);
}

void expectPoint(const Vec3& expected, const Vec3& actual) {
  EXPECT_DOUBLE_EQ(expected.x, actual.x);
  EXPECT_DOUBLE_EQ(expected.y, actual.y);
  EXPECT_DOUBLE_EQ(expected.z, actual.z);
}

TEST(VoxelMap, StaticEvidenceNeedsThresholdAndOneRayCountsEachVoxelOnce) {
  VoxelMap map(1.0, 2, 2, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {3.1, 0.1, 0.1});

  EXPECT_EQ(CellState::UNKNOWN, map.stateAt({1.1, 0.1, 0.1}, 0.0));
  EXPECT_EQ(CellState::UNKNOWN, map.stateAt({3.1, 0.1, 0.1}, 0.0));

  map.integrateStaticRay({0.1, 0.1, 0.1}, {3.1, 0.1, 0.1});
  EXPECT_EQ(CellState::FREE, map.stateAt({1.1, 0.1, 0.1}, 0.0));
  EXPECT_EQ(CellState::OCCUPIED, map.stateAt({3.1, 0.1, 0.1}, 0.0));
  EXPECT_EQ(CellState::UNKNOWN, map.stateAt({8.1, 0.1, 0.1}, 0.0));
}

TEST(VoxelMap, OriginAndEndpointInOneVoxelOnlyAddOccupiedEvidence) {
  VoxelMap map(1.0, 2, 1, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {0.9, 0.9, 0.9});
  EXPECT_EQ(CellState::UNKNOWN, map.stateAt({0.5, 0.5, 0.5}, 0.0));

  map.integrateStaticRay({0.2, 0.2, 0.2}, {0.8, 0.8, 0.8});
  EXPECT_EQ(CellState::OCCUPIED, map.stateAt({0.5, 0.5, 0.5}, 0.0));
}

TEST(VoxelMap, FloorsNegativeCoordinatesAndHonoursZeroBoundary) {
  VoxelMap map(1.0, 2, 2, 1.0);
  integrateTwice(&map, {-0.1, -0.1, -0.1}, {-2.1, -0.1, -0.1});

  EXPECT_EQ(CellState::FREE, map.stateAt({-0.1, -0.1, -0.1}, 0.0));
  EXPECT_EQ(CellState::FREE, map.stateAt({-1.0, -0.1, -0.1}, 0.0));
  EXPECT_EQ(CellState::OCCUPIED,
            map.stateAt({-2.1, -0.1, -0.1}, 0.0));
  EXPECT_EQ(CellState::UNKNOWN, map.stateAt({0.0, -0.1, -0.1}, 0.0));
}

TEST(VoxelMap, StaticOccupiedEvidenceAlwaysWinsOverFreeEvidence) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {0.1, 0.1, 0.1});
  EXPECT_EQ(CellState::OCCUPIED, map.stateAt({0.1, 0.1, 0.1}, 0.0));

  map.integrateStaticRay({-1.1, 0.1, 0.1}, {1.1, 0.1, 0.1});
  EXPECT_EQ(CellState::OCCUPIED, map.stateAt({0.1, 0.1, 0.1}, 0.0));
}

TEST(VoxelMap, DynamicTtlIncludesBoundaryAndRestoresStaticState) {
  VoxelMap map(1.0, 2, 2, 1.0);
  integrateTwice(&map, {0.1, 0.1, 0.1}, {3.1, 0.1, 0.1});
  map.integrateDynamicPoint({2.1, 0.1, 0.1}, 10.0);

  EXPECT_EQ(CellState::OCCUPIED, map.stateAt({2.1, 0.1, 0.1}, 10.5));
  EXPECT_EQ(CellState::OCCUPIED, map.stateAt({2.1, 0.1, 0.1}, 11.0));
  EXPECT_EQ(CellState::FREE, map.stateAt({2.1, 0.1, 0.1}, 11.1));
}

TEST(VoxelMap, OldDynamicStampAndClockRollbackDoNotExpireEarly) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {3.1, 0.1, 0.1});
  map.integrateDynamicPoint({2.1, 0.1, 0.1}, 10.0);
  map.integrateDynamicPoint({2.1, 0.1, 0.1}, 9.0);

  EXPECT_EQ(CellState::OCCUPIED, map.stateAt({2.1, 0.1, 0.1}, 8.0));
  EXPECT_EQ(CellState::OCCUPIED, map.stateAt({2.1, 0.1, 0.1}, 10.5));
  EXPECT_EQ(CellState::FREE, map.stateAt({2.1, 0.1, 0.1}, 11.1));
}

TEST(VoxelMap, UnknownDirectionIsNeverReportedClear) {
  VoxelMap map(0.5, 2, 2, 1.0);
  const auto clearance =
      map.axisClearance({0.0, 0.0, 2.0}, {1.0, 0.0, 0.0}, 4.0, 0.0);

  EXPECT_FALSE(clearance.known);
  EXPECT_DOUBLE_EQ(0.0, clearance.metres);
}

TEST(VoxelMap, UnknownAfterKnownFreeSpaceStillReturnsNoPartialDistance) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {-1.1, 0.1, 0.1});

  const auto clearance =
      map.axisClearance({0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 2.0, 0.0);
  EXPECT_FALSE(clearance.known);
  EXPECT_DOUBLE_EQ(0.0, clearance.metres);
}

TEST(VoxelMap, UnknownAfterStaticObstacleOverridesObstacleDistance) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {1.1, 0.1, 0.1});

  const auto clearance =
      map.axisClearance({0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 2.0, 0.0);
  EXPECT_FALSE(clearance.known);
  EXPECT_DOUBLE_EQ(0.0, clearance.metres);
}

TEST(VoxelMap, UnknownAfterDynamicObstacleOverridesObstacleDistance) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {-1.1, 0.1, 0.1});
  map.integrateDynamicPoint({1.1, 0.1, 0.1}, 10.0);

  const auto clearance =
      map.axisClearance({0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 2.0, 10.5);
  EXPECT_FALSE(clearance.known);
  EXPECT_DOUBLE_EQ(0.0, clearance.metres);
}

TEST(VoxelMap, ReportsMaximumDistanceWhenEveryTraversedVoxelIsFree) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {4.1, 0.1, 0.1});

  const auto clearance =
      map.axisClearance({0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 3.5, 0.0);
  EXPECT_TRUE(clearance.known);
  EXPECT_DOUBLE_EQ(3.5, clearance.metres);
}

TEST(VoxelMap, StaticObstacleDistanceIsVoxelEntryDistance) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {3.1, 0.1, 0.1});

  const auto clearance =
      map.axisClearance({0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 3.0, 0.0);
  EXPECT_TRUE(clearance.known);
  EXPECT_NEAR(2.9, clearance.metres, 1e-12);
}

TEST(VoxelMap, DynamicObstacleAffectsClearanceUntilItExpires) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateStaticRay({0.1, 0.1, 0.1}, {4.1, 0.1, 0.1});
  map.integrateDynamicPoint({2.1, 0.1, 0.1}, 10.0);

  const auto blocked =
      map.axisClearance({0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 3.0, 10.5);
  EXPECT_TRUE(blocked.known);
  EXPECT_NEAR(1.9, blocked.metres, 1e-12);

  const auto restored =
      map.axisClearance({0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 3.0, 11.1);
  EXPECT_TRUE(restored.known);
  EXPECT_DOUBLE_EQ(3.0, restored.metres);
}

TEST(VoxelMap, ChecksAllSixAxisDirections) {
  const std::vector<Vec3> axes{{1.0, 0.0, 0.0},  {-1.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0},  {0.0, -1.0, 0.0},
                               {0.0, 0.0, 1.0},  {0.0, 0.0, -1.0}};
  for (const auto& axis : axes) {
    SCOPED_TRACE(testing::Message() << axis.x << ',' << axis.y << ','
                                    << axis.z);
    VoxelMap map(1.0, 1, 1, 1.0);
    const Vec3 endpoint{0.5 + axis.x * 3.0, 0.5 + axis.y * 3.0,
                        0.5 + axis.z * 3.0};
    map.integrateStaticRay({0.5, 0.5, 0.5}, endpoint);

    const auto clearance =
        map.axisClearance({0.5, 0.5, 0.5}, axis, 3.0, 0.0);
    EXPECT_TRUE(clearance.known);
    EXPECT_NEAR(2.5, clearance.metres, 1e-12);
  }
}

TEST(VoxelMap, ZeroDistanceStillChecksOriginState) {
  VoxelMap unknown(1.0, 1, 1, 1.0);
  const auto unknown_result =
      unknown.axisClearance({0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 0.0, 0.0);
  EXPECT_FALSE(unknown_result.known);
  EXPECT_DOUBLE_EQ(0.0, unknown_result.metres);

  VoxelMap free(1.0, 1, 1, 1.0);
  free.integrateStaticRay({0.1, 0.1, 0.1}, {1.1, 0.1, 0.1});
  const auto free_result =
      free.axisClearance({0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 0.0, 0.0);
  EXPECT_TRUE(free_result.known);
  EXPECT_DOUBLE_EQ(0.0, free_result.metres);

  VoxelMap occupied(1.0, 1, 1, 1.0);
  occupied.integrateStaticRay({0.1, 0.1, 0.1}, {0.1, 0.1, 0.1});
  const auto occupied_result = occupied.axisClearance(
      {0.1, 0.1, 0.1}, {1.0, 0.0, 0.0}, 0.0, 0.0);
  EXPECT_TRUE(occupied_result.known);
  EXPECT_DOUBLE_EQ(0.0, occupied_result.metres);
}

TEST(VoxelMap, OccupiedPointSetsAreSeparatedSortedAndUseVoxelCentres) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateStaticRay({1.1, 0.1, 0.1}, {1.1, 0.1, 0.1});
  map.integrateStaticRay({-0.1, 2.1, 0.1}, {-0.1, 2.1, 0.1});
  map.integrateStaticRay({-0.1, -0.1, 1.1}, {-0.1, -0.1, 1.1});
  map.integrateDynamicPoint({3.1, 0.1, 0.1}, 10.0);
  map.integrateDynamicPoint({-1.1, 0.1, 0.1}, 10.0);
  map.integrateDynamicPoint({0.1, -2.1, 0.1}, 8.0);

  const auto static_points = map.staticOccupiedPoints(10.5);
  ASSERT_EQ(3u, static_points.size());
  expectPoint({-0.5, -0.5, 1.5}, static_points[0]);
  expectPoint({-0.5, 2.5, 0.5}, static_points[1]);
  expectPoint({1.5, 0.5, 0.5}, static_points[2]);

  const auto dynamic_points = map.dynamicOccupiedPoints(10.5);
  ASSERT_EQ(2u, dynamic_points.size());
  expectPoint({-1.5, 0.5, 0.5}, dynamic_points[0]);
  expectPoint({3.5, 0.5, 0.5}, dynamic_points[1]);
}

TEST(VoxelMap, DynamicPointsNeverPolluteStaticEvidence) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateDynamicPoint({0.1, 0.1, 0.1}, 4.0);

  EXPECT_TRUE(map.staticOccupiedPoints(4.0).empty());
  ASSERT_EQ(1u, map.dynamicOccupiedPoints(4.0).size());
  EXPECT_EQ(CellState::UNKNOWN, map.stateAt({0.1, 0.1, 0.1}, 5.1));
}

TEST(VoxelMap, DynamicPointQueryPrunesExpiredTimestampBeforeOlderUpdate) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateDynamicPoint({0.1, 0.1, 0.1}, 10.0);
  EXPECT_TRUE(map.dynamicOccupiedPoints(12.0).empty());

  map.integrateDynamicPoint({0.1, 0.1, 0.1}, 9.0);
  EXPECT_TRUE(map.dynamicOccupiedPoints(10.5).empty());
}

TEST(VoxelMap, FutureDynamicPointSurvivesClockRollbackQuery) {
  VoxelMap map(1.0, 1, 1, 1.0);
  map.integrateDynamicPoint({1.1, 0.1, 0.1}, 20.0);

  const auto points = map.dynamicOccupiedPoints(5.0);
  ASSERT_EQ(1u, points.size());
  expectPoint({1.5, 0.5, 0.5}, points[0]);
}

TEST(VoxelMap, DynamicPointQueryPrunesEveryExpiredVoxel) {
  VoxelMap map(1.0, 1, 1, 1.0);
  for (int index = 0; index < 128; ++index) {
    map.integrateDynamicPoint(
        {static_cast<double>(index) + 0.1, 0.1, 0.1}, 10.0);
  }
  EXPECT_TRUE(map.dynamicOccupiedPoints(12.0).empty());

  for (int index = 0; index < 128; ++index) {
    map.integrateDynamicPoint(
        {static_cast<double>(index) + 0.1, 0.1, 0.1}, 9.0);
  }
  EXPECT_TRUE(map.dynamicOccupiedPoints(10.5).empty());
}

TEST(VoxelMap, RejectsInvalidConfiguration) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(VoxelMap(0.0, 1, 1, 1.0), std::invalid_argument);
  EXPECT_THROW(VoxelMap(-1.0, 1, 1, 1.0), std::invalid_argument);
  EXPECT_THROW(VoxelMap(nan, 1, 1, 1.0), std::invalid_argument);
  EXPECT_THROW(VoxelMap(inf, 1, 1, 1.0), std::invalid_argument);
  EXPECT_THROW(VoxelMap(1.0, 0, 1, 1.0), std::invalid_argument);
  EXPECT_THROW(VoxelMap(1.0, 1, 0, 1.0), std::invalid_argument);
  EXPECT_THROW(VoxelMap(1.0, 1, 1, 0.0), std::invalid_argument);
  EXPECT_THROW(VoxelMap(1.0, 1, 1, -1.0), std::invalid_argument);
  EXPECT_THROW(VoxelMap(1.0, 1, 1, nan), std::invalid_argument);
  EXPECT_THROW(VoxelMap(1.0, 1, 1, inf), std::invalid_argument);
}

TEST(VoxelMap, RejectsNonFiniteIntegrationInputs) {
  VoxelMap map(1.0, 1, 1, 1.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_THROW(map.integrateStaticRay({nan, 0.0, 0.0}, {0.0, 0.0, 0.0}),
               std::invalid_argument);
  EXPECT_THROW(map.integrateStaticRay({0.0, 0.0, 0.0}, {0.0, inf, 0.0}),
               std::invalid_argument);
  EXPECT_THROW(map.integrateDynamicPoint({0.0, 0.0, 0.0}, nan),
               std::invalid_argument);
  EXPECT_THROW(map.integrateDynamicPoint({0.0, -inf, 0.0}, 0.0),
               std::invalid_argument);
}

TEST(VoxelMap, RejectsInvalidQueryInputs) {
  VoxelMap map(1.0, 1, 1, 1.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_THROW(map.stateAt({nan, 0.0, 0.0}, 0.0), std::invalid_argument);
  EXPECT_THROW(map.stateAt({0.0, 0.0, 0.0}, inf), std::invalid_argument);
  EXPECT_THROW(map.staticOccupiedPoints(nan), std::invalid_argument);
  EXPECT_THROW(map.dynamicOccupiedPoints(inf), std::invalid_argument);
  EXPECT_THROW(map.axisClearance({nan, 0.0, 0.0}, {1.0, 0.0, 0.0},
                                 1.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(map.axisClearance({0.0, 0.0, 0.0}, {inf, 0.0, 0.0},
                                 1.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(map.axisClearance({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                                 -0.1, 0.0),
               std::invalid_argument);
  EXPECT_THROW(map.axisClearance({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                                 nan, 0.0),
               std::invalid_argument);
  EXPECT_THROW(map.axisClearance({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                                 1.0, inf),
               std::invalid_argument);
  EXPECT_THROW(map.axisClearance({0.0, 0.0, 0.0}, {1.01, 0.0, 0.0},
                                 1.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(map.axisClearance({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                                 1.0, 0.0),
               std::invalid_argument);
}

TEST(VoxelMap, RejectsDiagonalUnitClearanceAxis) {
  VoxelMap map(1.0, 1, 1, 1.0);
  const double diagonal = std::sqrt(0.5);

  EXPECT_THROW(map.axisClearance({0.0, 0.0, 0.0},
                                 {diagonal, diagonal, 0.0}, 0.0, 0.0),
               std::invalid_argument);
}

TEST(VoxelMap, RejectsAnyNonzeroSecondaryClearanceAxisComponent) {
  VoxelMap map(1.0, 1, 1, 1.0);

  for (const double secondary : {1e-6, 1e-12}) {
    SCOPED_TRACE(secondary);
    EXPECT_THROW(map.axisClearance({0.0, 0.0, 0.0},
                                   {1.0, secondary, 0.0}, 0.0, 0.0),
                 std::invalid_argument);
  }
}

TEST(VoxelMap, RejectsCoordinatesOutsideRepresentableVoxelKeys) {
  VoxelMap map(1.0, 1, 1, 1.0);
  const double huge = std::numeric_limits<double>::max();

  EXPECT_THROW(map.integrateStaticRay({0.0, 0.0, 0.0}, {huge, 0.0, 0.0}),
               std::invalid_argument);
  EXPECT_THROW(map.integrateDynamicPoint({0.0, huge, 0.0}, 0.0),
               std::invalid_argument);
  EXPECT_THROW(map.stateAt({0.0, 0.0, -huge}, 0.0),
               std::invalid_argument);
}

TEST(VoxelMap, RejectsVoxelCentresOutsideFiniteCoordinateRange) {
  const double huge = std::numeric_limits<double>::max();

  VoxelMap static_map(huge, 1, 1, 1.0);
  static_map.integrateStaticRay({huge, 0.0, 0.0}, {huge, 0.0, 0.0});
  EXPECT_THROW(static_map.staticOccupiedPoints(0.0), std::invalid_argument);

  VoxelMap dynamic_map(huge, 1, 1, 1.0);
  dynamic_map.integrateDynamicPoint({huge, 0.0, 0.0}, 0.0);
  EXPECT_THROW(dynamic_map.dynamicOccupiedPoints(0.0),
               std::invalid_argument);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
