#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "search_coordinator/coordinator.h"

namespace {

using search_coordinator::Coordinator;
using search_coordinator::CoordinatorInput;
using search_coordinator::State;
using search_coordinator::ValidationKind;
using search_coordinator::Vec3;

CoordinatorInput readyInput() {
  CoordinatorInput input;
  input.now = 1.0;
  input.ready = true;
  input.mission_active = true;
  input.map_healthy = true;
  input.has_odom = true;
  input.odom = Vec3{0.0, 0.0, 3.0};
  input.mux_selected = "/typhoon_h480_0/mux_inputs/navigator/cmd_vel";
  return input;
}

void setGoal(CoordinatorInput* input, double x, double y, double z,
             double stamp) {
  input->high_level_goal.available = true;
  input->high_level_goal.frame_id = "map";
  input->high_level_goal.position = Vec3{x, y, z};
  input->high_level_goal.stamp = stamp;
}

void setValidation(CoordinatorInput* input, std::uint64_t generation,
                   ValidationKind kind, bool valid) {
  input->validation.available = true;
  input->validation.generation = generation;
  input->validation.kind = kind;
  input->validation.valid = valid;
}

TEST(Coordinator, DoesNotPublishBeforeTakeoffAndMissionReady) {
  Coordinator coordinator;
  CoordinatorInput input = readyInput();
  input.ready = false;
  setGoal(&input, 10.0, 0.0, 3.0, 1.0);

  const auto output = coordinator.step(input);

  EXPECT_EQ(State::WAIT_READY, output.state);
  EXPECT_FALSE(output.publish_generation);
  EXPECT_FALSE(output.publish_ego_goal);
  EXPECT_FALSE(output.request_validation);

  input.now = 2.0;
  input.ready = true;
  input.high_level_goal.stamp = 2.0;
  const auto after_takeoff = coordinator.step(input);
  EXPECT_EQ(1u, after_takeoff.generation);
  EXPECT_TRUE(after_takeoff.publish_generation);
  EXPECT_TRUE(after_takeoff.request_validation);
}

TEST(Coordinator, ClampsGoalAndIgnoresEquivalentTenHertzUpdates) {
  Coordinator coordinator;
  CoordinatorInput input = readyInput();
  setGoal(&input, 20.0, 0.0, 5.0, 1.0);

  const auto requested = coordinator.step(input);
  EXPECT_EQ(1u, requested.generation);
  EXPECT_TRUE(requested.publish_generation);
  EXPECT_TRUE(requested.request_validation);
  EXPECT_EQ(ValidationKind::DIRECT, requested.validation_kind);
  EXPECT_NEAR(8.0, requested.validation_goal.x, 1e-12);
  EXPECT_NEAR(4.0, requested.validation_goal.z, 1e-12);
  EXPECT_FALSE(requested.publish_ego_goal);

  input.now = 1.1;
  input.high_level_goal.stamp = 1.1;
  setValidation(&input, 1u, ValidationKind::DIRECT, true);
  const auto accepted = coordinator.step(input);
  EXPECT_EQ(State::PLANNING, accepted.state);
  EXPECT_TRUE(accepted.publish_ego_goal);
  EXPECT_NEAR(8.0, accepted.goal.x, 1e-12);
  EXPECT_FALSE(accepted.publish_generation);

  input.now = 1.2;
  input.high_level_goal.stamp = 1.2;
  input.validation.available = false;
  const auto repeated = coordinator.step(input);
  EXPECT_EQ(1u, repeated.generation);
  EXPECT_FALSE(repeated.publish_generation);
  EXPECT_FALSE(repeated.publish_ego_goal);
}

TEST(Coordinator, UsesOnlyFreshValidatedFrontierAfterDirectRejection) {
  Coordinator coordinator;
  CoordinatorInput input = readyInput();
  setGoal(&input, 20.0, 0.0, 3.0, 1.0);
  coordinator.step(input);

  input.now = 1.1;
  input.frontier_goal.available = true;
  input.frontier_goal.frame_id = "map";
  input.frontier_goal.position = Vec3{4.0, 2.0, 3.0};
  input.frontier_goal.stamp = 1.05;
  setValidation(&input, 1u, ValidationKind::DIRECT, false);
  const auto fallback_request = coordinator.step(input);
  EXPECT_TRUE(fallback_request.request_validation);
  EXPECT_EQ(ValidationKind::FRONTIER, fallback_request.validation_kind);
  EXPECT_FALSE(fallback_request.publish_ego_goal);

  input.now = 1.2;
  setValidation(&input, 1u, ValidationKind::FRONTIER, true);
  const auto accepted = coordinator.step(input);
  EXPECT_EQ(State::PLANNING, accepted.state);
  EXPECT_TRUE(accepted.publish_ego_goal);
  EXPECT_NEAR(4.0, accepted.goal.x, 1e-12);
  EXPECT_NEAR(2.0, accepted.goal.y, 1e-12);
}

TEST(Coordinator, HoldsAndCancelsWhenNoKnownFreeGoalExists) {
  Coordinator coordinator;
  CoordinatorInput input = readyInput();
  setGoal(&input, 10.0, 0.0, 3.0, 1.0);
  coordinator.step(input);

  input.now = 1.1;
  setValidation(&input, 1u, ValidationKind::DIRECT, false);
  const auto output = coordinator.step(input);

  EXPECT_EQ(State::HOLD, output.state);
  EXPECT_EQ(2u, output.generation);
  EXPECT_TRUE(output.publish_generation);
  EXPECT_EQ("NO_KNOWN_FREE_GOAL", output.fault_code);
  EXPECT_FALSE(output.publish_ego_goal);
}

TEST(Coordinator, CancelsPlanningTimeoutAndRejectedTrajectory) {
  Coordinator timeout_coordinator;
  CoordinatorInput timeout = readyInput();
  setGoal(&timeout, 6.0, 0.0, 3.0, 1.0);
  timeout_coordinator.step(timeout);
  timeout.now = 1.1;
  setValidation(&timeout, 1u, ValidationKind::DIRECT, true);
  EXPECT_EQ(State::PLANNING, timeout_coordinator.step(timeout).state);
  timeout.validation.available = false;
  timeout.now = 2.11;
  const auto timed_out = timeout_coordinator.step(timeout);
  EXPECT_EQ(State::HOLD, timed_out.state);
  EXPECT_EQ(2u, timed_out.generation);
  EXPECT_TRUE(timed_out.publish_generation);
  EXPECT_EQ("PLANNING_TIMEOUT", timed_out.fault_code);

  Coordinator rejected_coordinator;
  CoordinatorInput rejected = readyInput();
  setGoal(&rejected, 6.0, 0.0, 3.0, 1.0);
  rejected_coordinator.step(rejected);
  rejected.now = 1.1;
  setValidation(&rejected, 1u, ValidationKind::DIRECT, true);
  rejected_coordinator.step(rejected);
  rejected.validation.available = false;
  rejected.adapter_status = "TRAJECTORY_REJECTED";
  const auto rejected_output = rejected_coordinator.step(rejected);
  EXPECT_EQ(State::OBSERVING, rejected_output.state);
  EXPECT_EQ(2u, rejected_output.generation);
  EXPECT_TRUE(rejected_output.publish_generation);
}

TEST(Coordinator, MissionDeactivationCancelsAnActiveGeneration) {
  Coordinator coordinator;
  CoordinatorInput input = readyInput();
  setGoal(&input, 6.0, 0.0, 3.0, 1.0);
  coordinator.step(input);
  input.now = 1.1;
  setValidation(&input, 1u, ValidationKind::DIRECT, true);
  EXPECT_EQ(State::PLANNING, coordinator.step(input).state);

  input.now = 1.2;
  input.validation.available = false;
  input.mission_active = false;
  const auto stopped = coordinator.step(input);
  EXPECT_EQ(State::WAIT_READY, stopped.state);
  EXPECT_EQ(2u, stopped.generation);
  EXPECT_TRUE(stopped.publish_generation);
  EXPECT_FALSE(stopped.publish_ego_goal);
}

TEST(Coordinator, NonFiniteOdomCancelsAnActiveGeneration) {
  Coordinator coordinator;
  CoordinatorInput input = readyInput();
  setGoal(&input, 6.0, 0.0, 3.0, 1.0);
  coordinator.step(input);
  input.now = 1.1;
  setValidation(&input, 1u, ValidationKind::DIRECT, true);
  EXPECT_EQ(State::PLANNING, coordinator.step(input).state);

  input.validation.available = false;
  input.odom.x = std::numeric_limits<double>::quiet_NaN();
  const auto stopped = coordinator.step(input);
  EXPECT_EQ(State::HOLD, stopped.state);
  EXPECT_EQ(2u, stopped.generation);
  EXPECT_TRUE(stopped.publish_generation);
  EXPECT_EQ("INVALID_INPUT", stopped.fault_code);
}

TEST(Coordinator, ExecutesMatchingSplineAndReevaluatesAfterLocalArrival) {
  Coordinator coordinator;
  CoordinatorInput input = readyInput();
  setGoal(&input, 20.0, 0.0, 3.0, 1.0);
  coordinator.step(input);
  input.now = 1.1;
  setValidation(&input, 1u, ValidationKind::DIRECT, true);
  coordinator.step(input);

  input.validation.available = false;
  input.adapter_status = "EXECUTING:1:";
  EXPECT_EQ(State::PLANNING, coordinator.step(input).state);

  input.adapter_status = "EXECUTING:1:9";
  EXPECT_EQ(State::EXECUTING, coordinator.step(input).state);

  input.now = 1.2;
  input.adapter_status.clear();
  input.odom = Vec3{7.2, 0.0, 3.0};
  const auto arrived = coordinator.step(input);
  EXPECT_EQ(State::OBSERVING, arrived.state);
  EXPECT_EQ(2u, arrived.generation);
  EXPECT_TRUE(arrived.publish_generation);
  EXPECT_TRUE(arrived.request_validation);
  EXPECT_NEAR(15.2, arrived.validation_goal.x, 1e-12);
}

TEST(Coordinator, TrackingTakeoverInvalidatesOnceAndRejoinsFromCurrentOdom) {
  Coordinator coordinator;
  CoordinatorInput input = readyInput();
  setGoal(&input, 20.0, 0.0, 3.0, 1.0);
  coordinator.step(input);

  input.now = 1.1;
  input.tracking_candidate = true;
  const auto candidate = coordinator.step(input);
  EXPECT_EQ(State::CANDIDATE_HOLD, candidate.state);
  EXPECT_EQ(2u, candidate.generation);
  EXPECT_TRUE(candidate.publish_generation);

  input.now = 1.2;
  input.tracking_candidate = false;
  input.tracking_active = true;
  input.mux_selected = "/typhoon_h480_0/mux_inputs/external/cmd_vel";
  const auto tracking = coordinator.step(input);
  EXPECT_EQ(State::TRACKING_EXTERNAL, tracking.state);
  EXPECT_EQ(2u, tracking.generation);
  EXPECT_FALSE(tracking.publish_generation);

  input.now = 1.3;
  input.tracking_active = false;
  EXPECT_EQ(State::TRACKING_EXTERNAL, coordinator.step(input).state);

  input.now = 1.4;
  input.odom = Vec3{5.0, 1.0, 3.0};
  input.mux_selected = "/typhoon_h480_0/mux_inputs/navigator/cmd_vel";
  const auto rejoining = coordinator.step(input);
  EXPECT_EQ(State::REJOINING, rejoining.state);
  EXPECT_EQ(3u, rejoining.generation);
  EXPECT_TRUE(rejoining.publish_generation);
  EXPECT_TRUE(rejoining.request_validation);
  EXPECT_NEAR(8.0,
              std::hypot(rejoining.validation_goal.x - input.odom.x,
                         rejoining.validation_goal.y - input.odom.y),
              1e-12);
  EXPECT_NEAR(-1.0 / 15.0,
              (rejoining.validation_goal.y - input.odom.y) /
                  (rejoining.validation_goal.x - input.odom.x),
              1e-12);

  input.now = 1.5;
  setValidation(&input, 2u, ValidationKind::DIRECT, true);
  const auto late = coordinator.step(input);
  EXPECT_EQ(State::REJOINING, late.state);
  EXPECT_FALSE(late.publish_ego_goal);

  input.now = 1.6;
  setValidation(&input, 3u, ValidationKind::DIRECT, true);
  const auto resumed = coordinator.step(input);
  EXPECT_EQ(State::PLANNING, resumed.state);
  EXPECT_TRUE(resumed.publish_ego_goal);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
