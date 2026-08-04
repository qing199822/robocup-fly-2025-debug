#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "tracking/broadcast_progress.h"

TEST(BroadcastProgressTest, RequiresContinuousHeartbeatsForConfirmation) {
    BroadcastProgress progress(15.0, 0.5, 20.0);
    progress.start(100.0);

    for (int tenth = 1; tenth <= 149; ++tenth) {
        const double stamp = 100.0 + tenth / 10.0;
        EXPECT_TRUE(progress.recordHeartbeat(stamp, stamp));
    }
    EXPECT_FALSE(progress.broadcastConfirmed());

    EXPECT_TRUE(progress.recordHeartbeat(115.2, 115.2));
    EXPECT_TRUE(progress.broadcastConfirmed());
}

TEST(BroadcastProgressTest, GapRestartsConfirmationWindow) {
    BroadcastProgress progress(15.0, 0.5, 20.0);
    progress.start(10.0);

    EXPECT_TRUE(progress.recordHeartbeat(10.1, 10.1));
    EXPECT_TRUE(progress.recordHeartbeat(10.7, 10.7));
    for (int tenth = 8; tenth <= 156; ++tenth) {
        const double stamp = 10.0 + tenth / 10.0;
        EXPECT_TRUE(progress.recordHeartbeat(stamp, stamp));
    }

    EXPECT_FALSE(progress.broadcastConfirmed());
}

TEST(BroadcastProgressTest, RejectsInvalidTimestampsAndTimesOutSession) {
    BroadcastProgress progress(15.0, 0.5, 20.0);
    progress.start(50.0);

    EXPECT_FALSE(progress.recordHeartbeat(0.0, 50.1));
    EXPECT_FALSE(progress.recordHeartbeat(50.2, 50.1));
    EXPECT_TRUE(progress.recordHeartbeat(50.1, 50.1));
    EXPECT_FALSE(progress.recordHeartbeat(50.0, 50.2));
    EXPECT_FALSE(progress.recordHeartbeat(
        std::numeric_limits<double>::quiet_NaN(), 50.2));
    EXPECT_FALSE(progress.sessionTimedOut(69.999));
    EXPECT_TRUE(progress.sessionTimedOut(70.0));
}

TEST(BroadcastProgressTest, ResetClearsAllSessionState) {
    BroadcastProgress progress(0.19, 0.11, 1.0);
    progress.start(1.0);
    progress.recordHeartbeat(1.1, 1.1);
    progress.recordHeartbeat(1.2, 1.2);
    progress.recordHeartbeat(1.3, 1.3);
    ASSERT_TRUE(progress.broadcastConfirmed());

    progress.reset();

    EXPECT_FALSE(progress.active());
    EXPECT_FALSE(progress.broadcastConfirmed());
    EXPECT_FALSE(progress.sessionTimedOut(100.0));
}

TEST(BroadcastProgressTest, RejectsInvalidConfiguration) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    EXPECT_THROW(BroadcastProgress(0.0, 0.1, 1.0), std::invalid_argument);
    EXPECT_THROW(BroadcastProgress(1.0, 0.0, 2.0), std::invalid_argument);
    EXPECT_THROW(BroadcastProgress(1.0, 0.1, 0.0), std::invalid_argument);
    EXPECT_THROW(BroadcastProgress(nan, 0.1, 1.0), std::invalid_argument);
    EXPECT_THROW(BroadcastProgress(1.0, infinity, 2.0), std::invalid_argument);
    EXPECT_THROW(BroadcastProgress(2.0, 0.1, 2.0), std::invalid_argument);
    EXPECT_THROW(BroadcastProgress(1.0, 1.0, 2.0), std::invalid_argument);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
