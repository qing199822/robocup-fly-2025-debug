#include <stdexcept>

#include <gtest/gtest.h>

#include "task_manager/mission_progress.h"

using task_manager::MissionPhase;
using task_manager::MissionProgress;

TEST(MissionProgress, RunsEntryOnceThenLoopsPatrolOnly) {
  MissionProgress progress(2, 3);
  EXPECT_EQ(MissionPhase::ENTERING, progress.phase());
  EXPECT_EQ(0u, progress.index());
  progress.advance();
  EXPECT_EQ(MissionPhase::ENTERING, progress.phase());
  EXPECT_EQ(1u, progress.index());
  progress.advance();
  EXPECT_EQ(MissionPhase::PATROLLING, progress.phase());
  EXPECT_EQ(0u, progress.index());
  progress.advance();
  progress.advance();
  progress.advance();
  EXPECT_EQ(MissionPhase::PATROLLING, progress.phase());
  EXPECT_EQ(0u, progress.index());
}

TEST(MissionProgress, LegacyMissionStartsInPatrol) {
  MissionProgress progress(0, 3);
  EXPECT_EQ(MissionPhase::PATROLLING, progress.phase());
  EXPECT_EQ(0u, progress.index());
}

TEST(MissionProgress, ResumesEntryAtSameIndex) {
  MissionProgress progress(2, 3);
  progress.advance();
  progress.pause();
  EXPECT_TRUE(progress.paused());
  progress.resume();
  EXPECT_FALSE(progress.paused());
  EXPECT_EQ(MissionPhase::ENTERING, progress.phase());
  EXPECT_EQ(1u, progress.index());
}

TEST(MissionProgress, ResumesPatrolAtSameIndex) {
  MissionProgress progress(1, 3);
  progress.advance();
  progress.advance();
  progress.pause();
  progress.resume();
  EXPECT_EQ(MissionPhase::PATROLLING, progress.phase());
  EXPECT_EQ(1u, progress.index());
}

TEST(MissionProgress, RejectsInvalidCountsAndPauseTransitions) {
  EXPECT_THROW(MissionProgress(0, 0), std::invalid_argument);

  MissionProgress progress(1, 3);
  EXPECT_THROW(progress.resume(), std::logic_error);
  progress.pause();
  EXPECT_THROW(progress.pause(), std::logic_error);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
