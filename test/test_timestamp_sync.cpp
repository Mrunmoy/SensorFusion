#include <gtest/gtest.h>

#include "TimestampSync.hpp"

TEST(TimestampSyncTest, StartsUnlockedAndPassThrough) {
    sf::TimestampSync sync;
    EXPECT_FALSE(sync.isLocked());
    EXPECT_EQ(sync.toRemoteTimeUs(12345), 12345u);
}

TEST(TimestampSyncTest, FirstAnchorLocksAndSetsOffset) {
    sf::TimestampSync sync;
    sync.observeAnchor(1000, 1600);
    EXPECT_TRUE(sync.isLocked());
    EXPECT_EQ(sync.offsetUs(), 600);
    EXPECT_EQ(sync.toRemoteTimeUs(2000), 2600u);
}

TEST(TimestampSyncTest, SmoothsSmallOffsetUpdates) {
    sf::TimestampSync::Config cfg{};
    cfg.alpha = 0.5f;
    sf::TimestampSync sync(cfg);

    sync.observeAnchor(1000, 2000); // offset = 1000
    sync.observeAnchor(2000, 3300); // measured 1300; delta 300; +150
    EXPECT_EQ(sync.offsetUs(), 1150);
    EXPECT_EQ(sync.toRemoteTimeUs(4000), 5150u);
}

TEST(TimestampSyncTest, LargeJumpsRelockImmediately) {
    sf::TimestampSync::Config cfg{};
    cfg.alpha = 0.5f;
    cfg.maxDriftStepUs = 100;
    sf::TimestampSync sync(cfg);

    sync.observeAnchor(1000, 1500); // offset = 500
    sync.observeAnchor(2000, 3000); // measured 1000; jump 500 > 100
    EXPECT_EQ(sync.offsetUs(), 1000);
}

TEST(TimestampSyncTest, NegativeMappedTimeClampsToZero) {
    sf::TimestampSync sync;
    sync.observeAnchor(1000, 100); // offset = -900
    EXPECT_EQ(sync.toRemoteTimeUs(100), 0u);
}
