#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "ego_adapter/command_policy.h"

namespace {

using ego_adapter::AxisClearance;
using ego_adapter::CommandPolicy;
using ego_adapter::DirectionalClearance;
using ego_adapter::PolicyInput;
using ego_adapter::PolicyLimits;
using ego_adapter::Vec3;

DirectionalClearance clearSpace(double metres = 10.0) {
  const AxisClearance clear{true, metres};
  return DirectionalClearance{clear, clear, clear, clear, clear, clear};
}

PolicyInput safeInput() {
  PolicyInput input;
  input.now = 10.0;
  input.command_stamp = 9.9;
  input.bound_generation = 7u;
  input.active_generation = 7u;
  input.map_healthy = true;
  input.mux_is_navigator = true;
  input.trajectory_valid = true;
  input.current_position = Vec3{0.0, 0.0, 3.0};
  input.desired_position = Vec3{0.0, 0.0, 3.0};
  input.current_yaw = 0.0;
  input.desired_yaw = 0.0;
  input.desired_yaw_rate = 0.0;
  input.world_velocity = Vec3{1.0, 0.0, 0.0};
  input.clearance = clearSpace();
  return input;
}

void expectStopped(const ego_adapter::PolicyOutput& output,
                   const std::string& fault_code) {
  EXPECT_FALSE(output.accepted);
  EXPECT_EQ(fault_code, output.fault_code);
  EXPECT_DOUBLE_EQ(0.0, output.forward);
  EXPECT_DOUBLE_EQ(0.0, output.left);
  EXPECT_DOUBLE_EQ(0.0, output.up);
  EXPECT_DOUBLE_EQ(0.0, output.yaw_rate);
}

TEST(CommandPolicy, UsesPinnedSafetyDefaults) {
  const PolicyLimits limits;
  EXPECT_DOUBLE_EQ(0.20, limits.command_timeout);
  EXPECT_DOUBLE_EQ(4.0, limits.max_search_altitude);
  EXPECT_DOUBLE_EQ(0.60, limits.position_gain);
  EXPECT_NEAR(0.5235987756, limits.yaw_align_threshold, 1e-12);
  EXPECT_DOUBLE_EQ(1.5, limits.max_forward_speed);
  EXPECT_DOUBLE_EQ(0.25, limits.max_lateral_speed);
  EXPECT_DOUBLE_EQ(0.10, limits.max_reverse_speed);
  EXPECT_DOUBLE_EQ(0.50, limits.max_vertical_speed);
  EXPECT_DOUBLE_EQ(0.80, limits.max_yaw_rate);
  EXPECT_DOUBLE_EQ(1.50, limits.braking_clearance);
  EXPECT_DOUBLE_EQ(0.80, limits.emergency_clearance);
}

TEST(CommandPolicy, RejectsStaleAndFutureCommands) {
  CommandPolicy policy(PolicyLimits{});
  PolicyInput stale = safeInput();
  stale.command_stamp = 9.79;
  expectStopped(policy.evaluate(stale), "STALE_COMMAND");

  PolicyInput future = safeInput();
  future.command_stamp = 10.01;
  expectStopped(policy.evaluate(future), "TIME_ROLLBACK");
}

TEST(CommandPolicy, RejectsNonFiniteInputAndInvalidKnownClearance) {
  CommandPolicy policy(PolicyLimits{});
  PolicyInput non_finite = safeInput();
  non_finite.world_velocity.x =
      std::numeric_limits<double>::quiet_NaN();
  expectStopped(policy.evaluate(non_finite), "NON_FINITE_INPUT");

  PolicyInput bad_clearance = safeInput();
  bad_clearance.clearance.forward.metres = -0.1;
  expectStopped(policy.evaluate(bad_clearance), "INVALID_CLEARANCE");
}

TEST(CommandPolicy, RejectsFailedSafetyContracts) {
  CommandPolicy policy(PolicyLimits{});

  PolicyInput generation = safeInput();
  generation.bound_generation = 6u;
  expectStopped(policy.evaluate(generation), "WRONG_GENERATION");

  PolicyInput map = safeInput();
  map.map_healthy = false;
  expectStopped(policy.evaluate(map), "MAP_UNHEALTHY");

  PolicyInput mux = safeInput();
  mux.mux_is_navigator = false;
  expectStopped(policy.evaluate(mux), "MUX_NOT_NAVIGATOR");

  PolicyInput trajectory = safeInput();
  trajectory.trajectory_valid = false;
  expectStopped(policy.evaluate(trajectory), "TRAJECTORY_INVALID");
}

TEST(CommandPolicy, RejectsDesiredSearchAltitudeAboveFourMetres) {
  CommandPolicy policy(PolicyLimits{});
  PolicyInput input = safeInput();
  input.desired_position.z = 4.01;
  expectStopped(policy.evaluate(input), "HEIGHT_LIMIT");
}

TEST(CommandPolicy, AppliesPositionCorrectionBeforeWorldToFluRotation) {
  PolicyLimits limits;
  limits.max_lateral_speed = 2.0;
  CommandPolicy policy(limits);
  PolicyInput input = safeInput();
  input.current_position = Vec3{0.0, 0.0, 3.0};
  input.desired_position = Vec3{1.0, 0.0, 3.0};
  input.world_velocity = Vec3{0.4, 0.0, 0.0};
  input.current_yaw = 1.5707963267948966;
  input.desired_yaw = input.current_yaw;

  const auto output = policy.evaluate(input);

  EXPECT_TRUE(output.accepted);
  EXPECT_EQ("OK", output.fault_code);
  EXPECT_NEAR(0.0, output.forward, 1e-12);
  EXPECT_NEAR(-1.0, output.left, 1e-12);
  EXPECT_DOUBLE_EQ(0.0, output.up);
}

TEST(CommandPolicy, LargeYawErrorTurnsInPlace) {
  CommandPolicy policy(PolicyLimits{});
  PolicyInput input = safeInput();
  input.desired_yaw = 1.0;
  input.desired_yaw_rate = 0.2;

  const auto output = policy.evaluate(input);

  EXPECT_TRUE(output.accepted);
  EXPECT_EQ("YAW_ALIGNING", output.fault_code);
  EXPECT_DOUBLE_EQ(0.0, output.forward);
  EXPECT_DOUBLE_EQ(0.0, output.left);
  EXPECT_DOUBLE_EQ(0.0, output.up);
  EXPECT_DOUBLE_EQ(0.8, output.yaw_rate);
}

TEST(CommandPolicy, ClampsForwardLateralReverseVerticalAndYawRates) {
  CommandPolicy policy(PolicyLimits{});

  PolicyInput forward = safeInput();
  forward.world_velocity = Vec3{4.0, 2.0, 2.0};
  forward.desired_yaw_rate = 3.0;
  auto output = policy.evaluate(forward);
  EXPECT_DOUBLE_EQ(1.5, output.forward);
  EXPECT_DOUBLE_EQ(0.25, output.left);
  EXPECT_DOUBLE_EQ(0.5, output.up);
  EXPECT_DOUBLE_EQ(0.8, output.yaw_rate);

  PolicyInput reverse = safeInput();
  reverse.world_velocity = Vec3{-4.0, -2.0, -2.0};
  output = policy.evaluate(reverse);
  EXPECT_DOUBLE_EQ(-0.1, output.forward);
  EXPECT_DOUBLE_EQ(-0.25, output.left);
  EXPECT_DOUBLE_EQ(-0.5, output.up);
}

TEST(CommandPolicy, UnknownClearanceStopsOnlyTheAffectedDirection) {
  CommandPolicy policy(PolicyLimits{});
  PolicyInput input = safeInput();
  input.world_velocity = Vec3{1.0, 0.2, 0.1};
  input.clearance.forward.known = false;

  const auto output = policy.evaluate(input);

  EXPECT_TRUE(output.accepted);
  EXPECT_EQ("CLEARANCE_LIMITED", output.fault_code);
  EXPECT_DOUBLE_EQ(0.0, output.forward);
  EXPECT_DOUBLE_EQ(0.2, output.left);
  EXPECT_DOUBLE_EQ(0.1, output.up);
}

TEST(CommandPolicy, BrakingZoneScalesEachDirectionalComponentLinearly) {
  CommandPolicy policy(PolicyLimits{});
  PolicyInput input = safeInput();
  input.world_velocity = Vec3{1.0, -0.2, -0.4};
  input.clearance.forward.metres = 1.15;
  input.clearance.right.metres = 1.15;
  input.clearance.down.metres = 1.15;

  const auto output = policy.evaluate(input);

  EXPECT_TRUE(output.accepted);
  EXPECT_EQ("CLEARANCE_LIMITED", output.fault_code);
  EXPECT_NEAR(0.5, output.forward, 1e-12);
  EXPECT_NEAR(-0.1, output.left, 1e-12);
  EXPECT_NEAR(-0.2, output.up, 1e-12);
}

TEST(CommandPolicy, EmergencyClearanceStopsApproachInAllSigns) {
  CommandPolicy policy(PolicyLimits{});
  PolicyInput input = safeInput();
  input.world_velocity = Vec3{-1.0, 0.2, 0.4};
  input.clearance.backward.metres = 0.8;
  input.clearance.left.metres = 0.79;
  input.clearance.up.metres = 0.0;

  const auto output = policy.evaluate(input);

  EXPECT_TRUE(output.accepted);
  EXPECT_EQ("CLEARANCE_LIMITED", output.fault_code);
  EXPECT_DOUBLE_EQ(0.0, output.forward);
  EXPECT_DOUBLE_EQ(0.0, output.left);
  EXPECT_DOUBLE_EQ(0.0, output.up);
}

TEST(CommandPolicy, RejectsInvalidLimitConfiguration) {
  PolicyLimits limits;
  limits.emergency_clearance = limits.braking_clearance;
  EXPECT_THROW((void)CommandPolicy{limits}, std::invalid_argument);

  limits = PolicyLimits{};
  limits.command_timeout = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)CommandPolicy{limits}, std::invalid_argument);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
