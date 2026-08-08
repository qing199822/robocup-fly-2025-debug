#pragma once

#include <cstdint>
#include <string>

namespace search_coordinator {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class State {
  WAIT_READY,
  OBSERVING,
  PLANNING,
  EXECUTING,
  HOLD,
  CANDIDATE_HOLD,
  TRACKING_EXTERNAL,
  REJOINING,
};

enum class ValidationKind { NONE, DIRECT, FRONTIER };

struct StampedGoal {
  bool available{false};
  std::string frame_id;
  Vec3 position;
  double stamp{0.0};
};

struct ValidationResult {
  bool available{false};
  std::uint64_t generation{0u};
  ValidationKind kind{ValidationKind::NONE};
  bool valid{false};
};

struct CoordinatorInput {
  double now{0.0};
  bool ready{false};
  bool mission_active{false};
  bool tracking_candidate{false};
  bool tracking_active{false};
  bool map_healthy{false};
  bool has_odom{false};
  std::string mux_selected;
  Vec3 odom;
  StampedGoal high_level_goal;
  StampedGoal frontier_goal;
  std::string adapter_status;
  ValidationResult validation;
};

struct CoordinatorOutput {
  State state{State::WAIT_READY};
  std::uint64_t generation{0u};
  bool publish_generation{false};
  bool publish_ego_goal{false};
  Vec3 goal;
  std::string fault_code;
  bool request_validation{false};
  ValidationKind validation_kind{ValidationKind::NONE};
  Vec3 validation_goal;
};

struct CoordinatorConfig {
  double goal_position_epsilon{0.20};
  double goal_altitude_epsilon{0.10};
  double planning_timeout{1.0};
  double local_arrival_tolerance{1.0};
  double max_local_goal_distance{8.0};
  double min_search_altitude{2.0};
  double max_search_altitude{4.0};
  double frontier_max_age{0.50};
  std::string navigator_topic{
      "/typhoon_h480_0/mux_inputs/navigator/cmd_vel"};
};

class Coordinator {
 public:
  explicit Coordinator(const CoordinatorConfig& config = CoordinatorConfig{});

  CoordinatorOutput step(const CoordinatorInput& input);

 private:
  bool observeHighLevelGoal(const StampedGoal& goal);
  bool equivalentGoal(const Vec3& first, const Vec3& second) const;
  bool frontierUsable(const CoordinatorInput& input) const;
  Vec3 localGoal(const Vec3& current, const Vec3& high_level) const;
  void incrementGeneration(CoordinatorOutput* output);
  void clearActiveTask();
  void requestValidation(ValidationKind kind, const Vec3& goal,
                         State waiting_state, CoordinatorOutput* output);
  void requestDirectFromCurrent(const CoordinatorInput& input,
                                State waiting_state,
                                CoordinatorOutput* output);
  void cancelTo(State state, const std::string& fault,
                CoordinatorOutput* output);
  bool activeTask() const;
  bool matchingExecutionStatus(const std::string& status) const;

  CoordinatorConfig config_;
  State state_{State::WAIT_READY};
  std::uint64_t generation_{0u};
  bool has_high_level_goal_{false};
  bool high_level_goal_needs_processing_{false};
  StampedGoal high_level_goal_;
  bool has_local_goal_{false};
  Vec3 local_goal_;
  bool validation_pending_{false};
  ValidationKind validation_kind_{ValidationKind::NONE};
  std::uint64_t validation_generation_{0u};
  Vec3 validation_goal_;
  double planning_started_at_{0.0};
  bool tracking_session_{false};
  bool replan_requested_{false};
};

}  // namespace search_coordinator
