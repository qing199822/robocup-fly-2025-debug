#include "search_coordinator/coordinator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace search_coordinator {
namespace {

bool finite(double value) { return std::isfinite(value); }

bool finite(const Vec3& value) {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

double distance(const Vec3& first, const Vec3& second) {
  return std::hypot(std::hypot(second.x - first.x, second.y - first.y),
                    second.z - first.z);
}

double horizontalDistance(const Vec3& first, const Vec3& second) {
  return std::hypot(second.x - first.x, second.y - first.y);
}

}  // namespace

Coordinator::Coordinator(const CoordinatorConfig& config) : config_(config) {
  const bool valid =
      finite(config_.goal_position_epsilon) &&
      config_.goal_position_epsilon >= 0.0 &&
      finite(config_.goal_altitude_epsilon) &&
      config_.goal_altitude_epsilon >= 0.0 &&
      finite(config_.planning_timeout) && config_.planning_timeout > 0.0 &&
      finite(config_.local_arrival_tolerance) &&
      config_.local_arrival_tolerance > 0.0 &&
      finite(config_.max_local_goal_distance) &&
      config_.max_local_goal_distance > 0.0 &&
      finite(config_.min_search_altitude) &&
      finite(config_.max_search_altitude) &&
      config_.max_search_altitude > config_.min_search_altitude &&
      finite(config_.frontier_max_age) && config_.frontier_max_age > 0.0 &&
      !config_.navigator_topic.empty();
  if (!valid) {
    throw std::invalid_argument("invalid search coordinator configuration");
  }
}

bool Coordinator::equivalentGoal(const Vec3& first, const Vec3& second) const {
  return std::hypot(second.x - first.x, second.y - first.y) <=
             config_.goal_position_epsilon &&
         std::abs(second.z - first.z) <= config_.goal_altitude_epsilon;
}

bool Coordinator::observeHighLevelGoal(const StampedGoal& goal) {
  if (!goal.available || goal.frame_id != "map" || !finite(goal.position) ||
      !finite(goal.stamp)) {
    return false;
  }
  if (!has_high_level_goal_) {
    high_level_goal_ = goal;
    has_high_level_goal_ = true;
    high_level_goal_needs_processing_ = true;
    return true;
  }
  const bool changed = !equivalentGoal(high_level_goal_.position, goal.position);
  high_level_goal_ = goal;
  if (changed) {
    high_level_goal_needs_processing_ = true;
  }
  return changed;
}

Vec3 Coordinator::localGoal(const Vec3& current,
                            const Vec3& high_level) const {
  const double delta_x = high_level.x - current.x;
  const double delta_y = high_level.y - current.y;
  const double horizontal = std::hypot(delta_x, delta_y);
  const double scale = horizontal > config_.max_local_goal_distance
                           ? config_.max_local_goal_distance / horizontal
                           : 1.0;
  return Vec3{current.x + delta_x * scale, current.y + delta_y * scale,
              std::min(config_.max_search_altitude,
                       std::max(config_.min_search_altitude, high_level.z))};
}

bool Coordinator::frontierUsable(const CoordinatorInput& input) const {
  if (!input.frontier_goal.available ||
      input.frontier_goal.frame_id != "map" ||
      !finite(input.frontier_goal.position) ||
      !finite(input.frontier_goal.stamp)) {
    return false;
  }
  const double age = input.now - input.frontier_goal.stamp;
  return age >= 0.0 && age <= config_.frontier_max_age &&
         input.frontier_goal.position.z >= config_.min_search_altitude &&
         input.frontier_goal.position.z <= config_.max_search_altitude &&
         horizontalDistance(input.odom, input.frontier_goal.position) <=
             config_.max_local_goal_distance;
}

void Coordinator::incrementGeneration(CoordinatorOutput* output) {
  if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("search coordinator generation exhausted");
  }
  ++generation_;
  output->generation = generation_;
  output->publish_generation = true;
}

void Coordinator::clearActiveTask() {
  has_local_goal_ = false;
  validation_pending_ = false;
  validation_kind_ = ValidationKind::NONE;
  validation_generation_ = 0u;
  replan_requested_ = false;
}

void Coordinator::requestValidation(ValidationKind kind, const Vec3& goal,
                                    State waiting_state,
                                    CoordinatorOutput* output) {
  validation_pending_ = true;
  validation_kind_ = kind;
  validation_generation_ = generation_;
  validation_goal_ = goal;
  state_ = waiting_state;
  output->state = state_;
  output->generation = generation_;
  output->request_validation = true;
  output->validation_kind = kind;
  output->validation_goal = goal;
  output->fault_code = kind == ValidationKind::DIRECT
                           ? "VALIDATING_DIRECT"
                           : "VALIDATING_FRONTIER";
}

void Coordinator::requestDirectFromCurrent(const CoordinatorInput& input,
                                           State waiting_state,
                                           CoordinatorOutput* output) {
  requestValidation(ValidationKind::DIRECT,
                    localGoal(input.odom, high_level_goal_.position),
                    waiting_state, output);
}

void Coordinator::cancelTo(State state, const std::string& fault,
                           CoordinatorOutput* output) {
  clearActiveTask();
  incrementGeneration(output);
  state_ = state;
  output->state = state_;
  output->fault_code = fault;
}

bool Coordinator::activeTask() const {
  return validation_pending_ || has_local_goal_ || tracking_session_ ||
         state_ == State::PLANNING || state_ == State::EXECUTING ||
         state_ == State::CANDIDATE_HOLD ||
         state_ == State::TRACKING_EXTERNAL || state_ == State::REJOINING;
}

bool Coordinator::matchingExecutionStatus(const std::string& status) const {
  const std::string prefix = "EXECUTING:";
  if (status.compare(0u, prefix.size(), prefix) != 0) {
    return false;
  }
  const std::size_t separator = status.find(':', prefix.size());
  if (separator == std::string::npos || separator == prefix.size() ||
      separator + 1u >= status.size()) {
    return false;
  }
  try {
    std::size_t generation_parsed = 0u;
    const std::string generation_text =
        status.substr(prefix.size(), separator - prefix.size());
    const unsigned long long value =
        std::stoull(generation_text, &generation_parsed);
    std::size_t trajectory_parsed = 0u;
    const std::string trajectory_text = status.substr(separator + 1u);
    std::stoll(trajectory_text, &trajectory_parsed);
    return generation_parsed == generation_text.size() &&
           trajectory_parsed == trajectory_text.size() &&
           value == generation_;
  } catch (const std::exception&) {
    return false;
  }
}

CoordinatorOutput Coordinator::step(const CoordinatorInput& input) {
  CoordinatorOutput output;
  output.state = state_;
  output.generation = generation_;

  if (!finite(input.now) || !finite(input.odom)) {
    if (activeTask()) {
      cancelTo(State::HOLD, "INVALID_INPUT", &output);
    } else {
      state_ = State::HOLD;
      output.state = state_;
      output.fault_code = "INVALID_INPUT";
    }
    return output;
  }

  observeHighLevelGoal(input.high_level_goal);
  if (!input.ready || !input.mission_active || !input.has_odom) {
    if (activeTask()) {
      clearActiveTask();
      tracking_session_ = false;
      high_level_goal_needs_processing_ = has_high_level_goal_;
      incrementGeneration(&output);
    }
    state_ = State::WAIT_READY;
    output.state = state_;
    output.fault_code = "NOT_READY";
    return output;
  }

  const bool navigator_selected =
      input.mux_selected == config_.navigator_topic;
  const bool takeover = input.tracking_candidate || input.tracking_active ||
                        !navigator_selected;
  if (takeover && !tracking_session_) {
    clearActiveTask();
    incrementGeneration(&output);
    tracking_session_ = true;
  }
  if (tracking_session_) {
    if (input.tracking_active || !navigator_selected) {
      state_ = State::TRACKING_EXTERNAL;
      output.state = state_;
      output.fault_code = "TRACKING_EXTERNAL";
      return output;
    }
    if (input.tracking_candidate) {
      state_ = State::CANDIDATE_HOLD;
      output.state = state_;
      output.fault_code = "TRACKING_CANDIDATE";
      return output;
    }

    tracking_session_ = false;
    clearActiveTask();
    incrementGeneration(&output);
    state_ = State::REJOINING;
    output.state = state_;
    if (input.map_healthy && has_high_level_goal_) {
      high_level_goal_needs_processing_ = false;
      requestDirectFromCurrent(input, State::REJOINING, &output);
    } else {
      output.fault_code = "REJOIN_WAITING_FOR_MAP";
    }
    return output;
  }

  if (!input.map_healthy) {
    if (state_ == State::PLANNING || state_ == State::EXECUTING ||
        validation_pending_) {
      cancelTo(State::HOLD, "MAP_UNHEALTHY", &output);
    } else {
      state_ = State::HOLD;
      output.state = state_;
      output.fault_code = "MAP_UNHEALTHY";
    }
    return output;
  }

  if (high_level_goal_needs_processing_) {
    high_level_goal_needs_processing_ = false;
    clearActiveTask();
    incrementGeneration(&output);
    requestDirectFromCurrent(input, State::OBSERVING, &output);
    return output;
  }

  if (replan_requested_ && has_high_level_goal_) {
    replan_requested_ = false;
    requestDirectFromCurrent(input, State::OBSERVING, &output);
    return output;
  }

  if (validation_pending_ && input.validation.available &&
      input.validation.generation == validation_generation_ &&
      input.validation.kind == validation_kind_) {
    if (input.validation.valid) {
      validation_pending_ = false;
      has_local_goal_ = true;
      local_goal_ = validation_goal_;
      state_ = State::PLANNING;
      planning_started_at_ = input.now;
      output.state = state_;
      output.publish_ego_goal = true;
      output.goal = local_goal_;
      output.fault_code = validation_kind_ == ValidationKind::DIRECT
                              ? "DIRECT_GOAL_ACCEPTED"
                              : "FRONTIER_GOAL_ACCEPTED";
      validation_kind_ = ValidationKind::NONE;
      return output;
    }
    if (validation_kind_ == ValidationKind::DIRECT &&
        frontierUsable(input)) {
      requestValidation(ValidationKind::FRONTIER,
                        input.frontier_goal.position, state_, &output);
      return output;
    }
    cancelTo(State::HOLD, "NO_KNOWN_FREE_GOAL", &output);
    return output;
  }

  if (input.adapter_status == "TRAJECTORY_REJECTED" &&
      (state_ == State::PLANNING || state_ == State::EXECUTING)) {
    cancelTo(State::OBSERVING, "TRAJECTORY_REJECTED", &output);
    replan_requested_ = true;
    return output;
  }

  if (state_ == State::PLANNING &&
      input.now - planning_started_at_ > config_.planning_timeout) {
    cancelTo(State::HOLD, "PLANNING_TIMEOUT", &output);
    return output;
  }

  if (state_ == State::PLANNING &&
      matchingExecutionStatus(input.adapter_status)) {
    state_ = State::EXECUTING;
    output.state = state_;
    output.fault_code = "EXECUTING";
    return output;
  }

  if (state_ == State::EXECUTING && has_local_goal_ &&
      distance(input.odom, local_goal_) <= config_.local_arrival_tolerance) {
    clearActiveTask();
    incrementGeneration(&output);
    state_ = State::OBSERVING;
    output.state = state_;
    if (has_high_level_goal_ &&
        distance(input.odom, high_level_goal_.position) >
            config_.local_arrival_tolerance) {
      requestDirectFromCurrent(input, State::OBSERVING, &output);
    } else {
      output.fault_code = "HIGH_LEVEL_GOAL_REACHED";
    }
    return output;
  }

  if (!has_high_level_goal_ && state_ == State::WAIT_READY) {
    state_ = State::OBSERVING;
    output.state = state_;
    output.fault_code = "NO_HIGH_LEVEL_GOAL";
  }
  return output;
}

}  // namespace search_coordinator
