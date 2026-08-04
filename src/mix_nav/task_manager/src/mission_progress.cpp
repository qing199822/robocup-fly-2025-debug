#include "task_manager/mission_progress.h"

namespace task_manager {

MissionProgress::MissionProgress(std::size_t entry_count,
                                 std::size_t patrol_count)
    : entry_count_(entry_count),
      patrol_count_(patrol_count),
      phase_(entry_count > 0 ? MissionPhase::ENTERING
                             : MissionPhase::PATROLLING),
      phase_before_pause_(phase_) {
  if (patrol_count == 0) {
    throw std::invalid_argument("patrol_count must be greater than zero");
  }
}

MissionPhase MissionProgress::phase() const { return phase_; }

std::size_t MissionProgress::index() const {
  return phase_ == MissionPhase::ENTERING ? entry_index_ : patrol_index_;
}

bool MissionProgress::paused() const { return paused_; }

void MissionProgress::advance() {
  if (phase_ == MissionPhase::ENTERING) {
    ++entry_index_;
    if (entry_index_ >= entry_count_) {
      phase_ = MissionPhase::PATROLLING;
      patrol_index_ = 0;
    }
    return;
  }

  patrol_index_ = (patrol_index_ + 1) % patrol_count_;
}

void MissionProgress::pause() {
  if (paused_) {
    throw std::logic_error("mission progress is already paused");
  }
  phase_before_pause_ = phase_;
  paused_ = true;
}

void MissionProgress::resume() {
  if (!paused_) {
    throw std::logic_error("mission progress is not paused");
  }
  phase_ = phase_before_pause_;
  paused_ = false;
}

}  // namespace task_manager
