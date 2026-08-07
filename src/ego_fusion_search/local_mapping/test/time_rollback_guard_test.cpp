#include <limits>

#include <gtest/gtest.h>

#include "local_mapping/time_rollback_guard.h"

namespace {

using local_mapping::TimeRollbackGuard;

TEST(TimeRollbackGuard, FirstEqualAndForwardTimesStayInCurrentEpoch) {
  TimeRollbackGuard guard;

  EXPECT_FALSE(guard.observe(100.0));
  EXPECT_FALSE(guard.observe(100.0));
  EXPECT_FALSE(guard.observe(101.0));
}

TEST(TimeRollbackGuard, StrictRollbackStartsOneNewEpoch) {
  TimeRollbackGuard guard;
  ASSERT_FALSE(guard.observe(100.0));

  EXPECT_TRUE(guard.observe(1.0));
  EXPECT_FALSE(guard.observe(1.0));
  EXPECT_FALSE(guard.observe(2.0));
}

TEST(TimeRollbackGuard, NonFiniteTimesFailClosedWithoutChangingEpoch) {
  TimeRollbackGuard guard;
  ASSERT_FALSE(guard.observe(100.0));

  EXPECT_FALSE(guard.observe(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(guard.observe(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(guard.observe(-std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(guard.observe(1.0));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
