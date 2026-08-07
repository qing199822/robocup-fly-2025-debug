#include <gtest/gtest.h>

#include "local_mapping/integration_rate_limiter.h"

namespace {

using local_mapping::IntegrationRateLimiter;

TEST(IntegrationRateLimiter, FirstFrameIsDue) {
  IntegrationRateLimiter limiter(5.0);

  EXPECT_TRUE(limiter.due(10.0));
}

TEST(IntegrationRateLimiter, FrameBeforePeriodIsNotDue) {
  IntegrationRateLimiter limiter(5.0);
  ASSERT_TRUE(limiter.due(10.0));
  limiter.markIntegrated(10.0);

  EXPECT_FALSE(limiter.due(10.199));
}

TEST(IntegrationRateLimiter, FrameAtPeriodBoundaryIsDue) {
  IntegrationRateLimiter limiter(5.0);
  limiter.markIntegrated(10.0);

  EXPECT_TRUE(limiter.due(10.2));
}

TEST(IntegrationRateLimiter, ClockRollbackFailsClosed) {
  IntegrationRateLimiter limiter(5.0);
  limiter.markIntegrated(10.0);

  EXPECT_FALSE(limiter.due(9.9));
}

TEST(IntegrationRateLimiter, ResetMakesNextFrameDueInNewEpoch) {
  IntegrationRateLimiter limiter(5.0);
  limiter.markIntegrated(100.0);

  limiter.reset();

  EXPECT_TRUE(limiter.due(1.0));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
