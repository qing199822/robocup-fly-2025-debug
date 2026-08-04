#pragma once

#include <cstddef>
#include <stdexcept>

namespace task_manager {

enum class MissionPhase { ENTERING, PATROLLING };

class MissionProgress {
 public:
  MissionProgress(std::size_t entry_count, std::size_t patrol_count);

  MissionPhase phase() const;
  std::size_t index() const;
  bool paused() const;
  void advance();
  void pause();
  void resume();

 private:
  std::size_t entry_count_;
  std::size_t patrol_count_;
  std::size_t entry_index_{0};
  std::size_t patrol_index_{0};
  MissionPhase phase_;
  MissionPhase phase_before_pause_;
  bool paused_{false};
};

}  // namespace task_manager
