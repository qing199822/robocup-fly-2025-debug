#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "local_mapping/health_monitor.h"

namespace {

using local_mapping::HealthConfig;
using local_mapping::HealthMonitor;

HealthConfig standardConfig() {
  return HealthConfig{0.15, 0.50, 0.50, 1.00, 0.20};
}

TEST(HealthMonitor, RequiresContinuousRecoveryWindow) {
  HealthMonitor monitor(standardConfig());

  monitor.observeDepth(10.00, 0.80);
  monitor.observeOdom(10.05);
  EXPECT_FALSE(monitor.evaluate(10.05).healthy);

  monitor.observeDepth(11.00, 0.80);
  monitor.observeOdom(11.05);
  EXPECT_FALSE(monitor.evaluate(11.05).healthy);

  monitor.observeDepth(11.40, 0.80);
  monitor.observeOdom(11.45);
  EXPECT_FALSE(monitor.evaluate(11.45).healthy);
  monitor.observeDepth(11.80, 0.80);
  monitor.observeOdom(11.85);
  EXPECT_FALSE(monitor.evaluate(11.85).healthy);
  monitor.observeDepth(12.00, 0.80);
  monitor.observeOdom(12.05);
  const auto result = monitor.evaluate(12.05);
  EXPECT_TRUE(result.healthy);
  EXPECT_EQ("OK", result.fault_code);
}

TEST(HealthMonitor, ReportsNotReadyWithoutBothObservations) {
  HealthMonitor monitor(standardConfig());
  EXPECT_EQ("NOT_READY", monitor.evaluate(1.0).fault_code);
  EXPECT_EQ("NOT_READY", monitor.evaluate(0.9).fault_code);

  monitor.observeDepth(1.0, 0.8);
  const auto result = monitor.evaluate(1.0);
  EXPECT_FALSE(result.healthy);
  EXPECT_TRUE(result.depth_healthy);
  EXPECT_FALSE(result.odom_healthy);
  EXPECT_FALSE(result.synchronized);
  EXPECT_DOUBLE_EQ(0.8, result.valid_depth_ratio);
  EXPECT_EQ("NOT_READY", result.fault_code);
}

TEST(HealthMonitor, ReportsDepthTimeout) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(1.0, 0.8);
  monitor.observeOdom(1.4);

  const auto result = monitor.evaluate(1.6);
  EXPECT_FALSE(result.depth_healthy);
  EXPECT_TRUE(result.odom_healthy);
  EXPECT_FALSE(result.synchronized);
  EXPECT_EQ("DEPTH_TIMEOUT", result.fault_code);
}

TEST(HealthMonitor, ReportsOdomTimeout) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(1.4, 0.8);
  monitor.observeOdom(1.0);

  const auto result = monitor.evaluate(1.6);
  EXPECT_TRUE(result.depth_healthy);
  EXPECT_FALSE(result.odom_healthy);
  EXPECT_FALSE(result.synchronized);
  EXPECT_EQ("ODOM_TIMEOUT", result.fault_code);
}

TEST(HealthMonitor, ReportsSyncError) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(2.00, 0.8);
  monitor.observeOdom(2.16);

  const auto result = monitor.evaluate(2.16);
  EXPECT_TRUE(result.depth_healthy);
  EXPECT_TRUE(result.odom_healthy);
  EXPECT_FALSE(result.synchronized);
  EXPECT_EQ("SYNC_ERROR", result.fault_code);
}

TEST(HealthMonitor, ReportsInvalidDepthRatio) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(5.00, 0.10);
  monitor.observeOdom(5.05);

  const auto result = monitor.evaluate(5.05);
  EXPECT_FALSE(result.healthy);
  EXPECT_FALSE(result.depth_healthy);
  EXPECT_TRUE(result.odom_healthy);
  EXPECT_TRUE(result.synchronized);
  EXPECT_DOUBLE_EQ(0.10, result.valid_depth_ratio);
  EXPECT_EQ("DEPTH_INVALID", result.fault_code);
}

TEST(HealthMonitor, RejectsDepthRatiosOutsideUnitInterval) {
  HealthMonitor below_range(standardConfig());
  below_range.observeDepth(5.00, -0.01);
  below_range.observeOdom(5.05);
  const auto below_result = below_range.evaluate(5.05);
  EXPECT_FALSE(below_result.depth_healthy);
  EXPECT_TRUE(below_result.synchronized);
  EXPECT_DOUBLE_EQ(-0.01, below_result.valid_depth_ratio);
  EXPECT_EQ("DEPTH_INVALID", below_result.fault_code);

  HealthMonitor above_range(standardConfig());
  above_range.observeDepth(5.00, 1.01);
  above_range.observeOdom(5.05);
  const auto above_result = above_range.evaluate(5.05);
  EXPECT_FALSE(above_result.depth_healthy);
  EXPECT_TRUE(above_result.synchronized);
  EXPECT_DOUBLE_EQ(1.01, above_result.valid_depth_ratio);
  EXPECT_EQ("DEPTH_INVALID", above_result.fault_code);
}

TEST(HealthMonitor, RejectsInvalidConfiguration) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(HealthMonitor(HealthConfig{nan, 0.5, 0.5, 1.0, 0.2}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{0.1, inf, 0.5, 1.0, 0.2}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{0.1, 0.5, inf, 1.0, 0.2}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{0.1, 0.5, 0.5, inf, 0.2}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{0.1, 0.5, 0.5, 1.0, inf}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{-0.1, 0.5, 0.5, 1.0, 0.2}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{0.1, 0.0, 0.5, 1.0, 0.2}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{0.1, 0.5, -1.0, 1.0, 0.2}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{0.1, 0.5, 0.5, 0.0, 0.2}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{0.1, 0.5, 0.5, 1.0, -0.1}),
               std::invalid_argument);
  EXPECT_THROW(HealthMonitor(HealthConfig{0.1, 0.5, 0.5, 1.0, 1.1}),
               std::invalid_argument);
}

TEST(HealthMonitor, FailsClosedForNonFiniteDepthObservation) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(std::numeric_limits<double>::infinity(), 0.8);
  monitor.observeOdom(1.0);
  EXPECT_FALSE(monitor.evaluate(1.0).healthy);
  EXPECT_EQ("NOT_READY", monitor.evaluate(1.0).fault_code);

  monitor.observeDepth(1.0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(monitor.evaluate(1.0).healthy);
}

TEST(HealthMonitor, FailsClosedForNonFiniteOdomObservation) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(1.0, 0.8);
  monitor.observeOdom(std::numeric_limits<double>::quiet_NaN());

  const auto result = monitor.evaluate(1.0);
  EXPECT_FALSE(result.healthy);
  EXPECT_EQ("NOT_READY", result.fault_code);
}

TEST(HealthMonitor, FailsClosedWhenEvaluationTimeIsNotFinite) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(1.0, 0.8);
  monitor.observeOdom(1.0);

  const auto result =
      monitor.evaluate(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(result.healthy);
  EXPECT_EQ("NOT_READY", result.fault_code);
}

TEST(HealthMonitor, CountsDroppedFrames) {
  HealthMonitor monitor(standardConfig());
  EXPECT_EQ(0u, monitor.droppedFrames());
  monitor.noteDroppedFrame();
  monitor.noteDroppedFrame();
  EXPECT_EQ(2u, monitor.droppedFrames());
}

TEST(HealthMonitor, FailureRestartsRecoveryWindow) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(1.00, 0.8);
  monitor.observeOdom(1.05);
  EXPECT_FALSE(monitor.evaluate(1.05).healthy);
  EXPECT_FALSE(monitor.evaluate(1.55).healthy);

  monitor.observeDepth(1.60, 0.1);
  monitor.observeOdom(1.65);
  EXPECT_EQ("DEPTH_INVALID", monitor.evaluate(1.65).fault_code);

  monitor.observeDepth(2.00, 0.8);
  monitor.observeOdom(2.05);
  EXPECT_FALSE(monitor.evaluate(2.05).healthy);
  monitor.observeDepth(2.40, 0.8);
  monitor.observeOdom(2.45);
  EXPECT_FALSE(monitor.evaluate(2.45).healthy);
  monitor.observeDepth(2.80, 0.8);
  monitor.observeOdom(2.85);
  EXPECT_FALSE(monitor.evaluate(2.85).healthy);
  monitor.observeDepth(3.00, 0.8);
  monitor.observeOdom(3.05);
  EXPECT_TRUE(monitor.evaluate(3.05).healthy);
}

TEST(HealthMonitor, TimeoutBoundaryDoesNotBreakRecovery) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(1.00, 0.20);
  monitor.observeOdom(1.05);
  EXPECT_FALSE(monitor.evaluate(1.05).healthy);

  monitor.observeDepth(1.50, 0.20);
  monitor.observeOdom(1.55);
  EXPECT_FALSE(monitor.evaluate(1.55).healthy);

  monitor.observeDepth(2.00, 0.20);
  monitor.observeOdom(2.05);
  const auto result = monitor.evaluate(2.05);
  EXPECT_TRUE(result.healthy);
  EXPECT_TRUE(result.depth_healthy);
  EXPECT_TRUE(result.odom_healthy);
  EXPECT_TRUE(result.synchronized);
  EXPECT_EQ("OK", result.fault_code);
}

TEST(HealthMonitor, ObservationTimeRollbackRestartsRecovery) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(10.00, 0.80);
  monitor.observeOdom(10.05);
  EXPECT_FALSE(monitor.evaluate(10.05).healthy);
  monitor.observeDepth(10.40, 0.80);
  monitor.observeOdom(10.45);
  EXPECT_FALSE(monitor.evaluate(10.45).healthy);

  monitor.observeDepth(9.00, 0.80);
  monitor.observeOdom(9.05);
  EXPECT_FALSE(monitor.evaluate(9.05).healthy);

  monitor.observeDepth(9.40, 0.80);
  monitor.observeOdom(9.45);
  EXPECT_FALSE(monitor.evaluate(9.45).healthy);
  monitor.observeDepth(9.80, 0.80);
  monitor.observeOdom(9.85);
  EXPECT_FALSE(monitor.evaluate(9.85).healthy);
  monitor.observeDepth(10.00, 0.80);
  monitor.observeOdom(10.05);
  EXPECT_FALSE(monitor.evaluate(10.05).healthy);
  monitor.observeDepth(10.40, 0.80);
  monitor.observeOdom(10.45);
  EXPECT_TRUE(monitor.evaluate(10.45).healthy);
}

TEST(HealthMonitor, TimeRollbackFailsClosedAndRestartsRecovery) {
  HealthMonitor monitor(standardConfig());
  monitor.observeDepth(4.00, 0.8);
  monitor.observeOdom(4.05);
  EXPECT_FALSE(monitor.evaluate(4.05).healthy);

  const auto rollback = monitor.evaluate(3.95);
  EXPECT_FALSE(rollback.healthy);
  EXPECT_EQ("DEPTH_TIMEOUT", rollback.fault_code);

  EXPECT_FALSE(monitor.evaluate(4.05).healthy);
  monitor.observeDepth(4.40, 0.8);
  monitor.observeOdom(4.45);
  EXPECT_FALSE(monitor.evaluate(4.45).healthy);
  monitor.observeDepth(4.80, 0.8);
  monitor.observeOdom(4.85);
  EXPECT_FALSE(monitor.evaluate(4.85).healthy);
  monitor.observeDepth(5.00, 0.8);
  monitor.observeOdom(5.05);
  EXPECT_TRUE(monitor.evaluate(5.05).healthy);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
