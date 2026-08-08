#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "safety_filter/safety_policy.h"

using safety_filter::Fault;
using safety_filter::Limits;
using safety_filter::DirectionalClearance;
using safety_filter::PerceptionGuard;
using safety_filter::PerceptionLimits;
using safety_filter::SafetyPolicy;

TEST(SafetyPolicy, DefaultMaximumAltitudeIsFourMetres) {
  EXPECT_DOUBLE_EQ(4.0, Limits{}.max_altitude);
}

TEST(SafetyPolicy, RejectsNonFiniteCommand) {
  SafetyPolicy policy(Limits{});
  geometry_msgs::Twist input;
  input.linear.x = std::numeric_limits<double>::quiet_NaN();
  const auto result = policy.apply(input, 3.0, 0.05, 4.0);
  EXPECT_EQ(Fault::NON_FINITE_COMMAND, result.fault);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.x);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);
}

TEST(SafetyPolicy, ClampsHorizontalVectorAndYawRate) {
  Limits limits;
  limits.max_xy_speed = 3.0;
  limits.max_yaw_rate = 1.0;
  limits.max_xy_acceleration = 100.0;
  SafetyPolicy policy(limits);
  geometry_msgs::Twist input;
  input.linear.x = 3.0;
  input.linear.y = 4.0;
  input.angular.z = 2.0;
  const auto result = policy.apply(input, 3.0, 1.0, 4.0);
  EXPECT_EQ(Fault::NONE, result.fault);
  EXPECT_NEAR(
      3.0, std::hypot(result.command.linear.x, result.command.linear.y), 1e-9);
  EXPECT_DOUBLE_EQ(1.0, result.command.angular.z);
}

TEST(SafetyPolicy, BlocksOnlyVelocityThatCrossesAltitudeBoundary) {
  Limits limits;
  limits.min_altitude = 0.5;
  limits.max_altitude = 5.5;
  limits.max_z_acceleration = 100.0;
  SafetyPolicy high_policy(limits);
  geometry_msgs::Twist climb;
  climb.linear.z = 0.8;
  auto result = high_policy.apply(climb, 5.4, 1.0, 5.5);
  EXPECT_DOUBLE_EQ(0.8, result.command.linear.z);
  result = high_policy.apply(climb, 5.5, 1.0, 5.5);
  EXPECT_EQ(Fault::ALTITUDE_LIMIT, result.fault);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);

  SafetyPolicy low_policy(limits);
  geometry_msgs::Twist descend;
  descend.linear.z = -0.8;
  result = low_policy.apply(descend, 0.5, 1.0, 5.5);
  EXPECT_EQ(Fault::ALTITUDE_LIMIT, result.fault);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);
}

TEST(SafetyPolicy, LimitsAccelerationFromPreviousOutput) {
  Limits limits;
  limits.max_xy_speed = 10.0;
  limits.max_xy_acceleration = 2.0;
  SafetyPolicy policy(limits);
  geometry_msgs::Twist input;
  input.linear.x = 8.0;
  const auto result = policy.apply(input, 3.0, 0.05, 4.0);
  EXPECT_NEAR(0.1, result.command.linear.x, 1e-9);
}

TEST(SafetyPolicy, ResetClearsPreviousOutput) {
  Limits limits;
  limits.max_xy_speed = 10.0;
  limits.max_xy_acceleration = 2.0;
  SafetyPolicy policy(limits);
  geometry_msgs::Twist input;
  input.linear.x = 8.0;
  EXPECT_NEAR(0.2, policy.apply(input, 3.0, 0.1, 4.0).command.linear.x, 1e-9);
  policy.reset();
  EXPECT_NEAR(0.2, policy.apply(input, 3.0, 0.1, 4.0).command.linear.x, 1e-9);
}

TEST(SafetyPolicy, AppliesPerFrameAltitudeLimitWithoutMutatingDefaults) {
  Limits limits;
  limits.max_z_acceleration = 100.0;
  SafetyPolicy policy(limits);
  geometry_msgs::Twist climb;
  climb.linear.z = 0.5;

  auto result = policy.apply(climb, 4.01, 1.0, 4.0);
  EXPECT_EQ(Fault::ALTITUDE_LIMIT, result.fault);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);
  result = policy.apply(climb, 4.01, 1.0, 6.0);
  EXPECT_EQ(Fault::NONE, result.fault);
  EXPECT_GT(result.command.linear.z, 0.0);
  result = policy.apply(climb, 6.01, 1.0, 6.0);
  EXPECT_EQ(Fault::ALTITUDE_LIMIT, result.fault);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);
}

TEST(PerceptionGuard, MapsFluSignsAndScalesBrakingClearance) {
  PerceptionGuard guard(PerceptionLimits{1.5, 0.8});
  DirectionalClearance clearance;
  clearance.forward = {true, 1.15};
  clearance.backward = {true, 10.0};
  clearance.left = {false, 0.0};
  clearance.right = {true, 10.0};
  clearance.upward = {true, 0.7};
  clearance.downward = {true, 10.0};
  geometry_msgs::Twist command;
  command.linear.x = 1.0;
  command.linear.y = 1.0;
  command.linear.z = 1.0;

  const auto result = guard.apply(command, clearance);
  EXPECT_NEAR(0.5, result.command.linear.x, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.y);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);
  EXPECT_TRUE(result.blocked);
}

TEST(PerceptionGuard, InvalidDistanceZerosAllTranslation) {
  PerceptionGuard guard(PerceptionLimits{1.5, 0.8});
  DirectionalClearance clearance;
  clearance.forward = {true, 10.0};
  clearance.backward = {true, std::numeric_limits<double>::quiet_NaN()};
  clearance.left = {true, 10.0};
  clearance.right = {true, 10.0};
  clearance.upward = {true, 10.0};
  clearance.downward = {true, 10.0};
  geometry_msgs::Twist command;
  command.linear.x = 1.0;
  command.angular.z = 0.2;

  const auto result = guard.apply(command, clearance);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.x);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.y);
  EXPECT_DOUBLE_EQ(0.0, result.command.linear.z);
  EXPECT_DOUBLE_EQ(0.2, result.command.angular.z);
  EXPECT_TRUE(result.blocked);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
