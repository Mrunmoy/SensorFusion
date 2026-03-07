#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockI2CBus.hpp"
#include "MockDelayProvider.hpp"
#include "AT24CxxNvStore.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;
using ::testing::InSequence;

TEST(AT24CxxNvStoreTest, CapacityMatchesConfig) {
    MockI2CBus bus;
    MockDelayProvider delay;
    AT24CxxConfig cfg;
    cfg.totalBytes = 2048;
    AT24CxxNvStore nv(bus, delay, cfg);
    EXPECT_EQ(nv.capacity(), 2048u);
}

TEST(AT24CxxNvStoreTest, ReadUsesAddressPointerThenRawRead) {
    MockI2CBus bus;
    MockDelayProvider delay;
    AT24CxxNvStore nv(bus, delay);

    {
        InSequence seq;
        EXPECT_CALL(bus, rawWrite(0x50, _, 2))
            .WillOnce([](uint8_t, const uint8_t* data, size_t len) {
                return len == 2 && data[0] == 0x01 && data[1] == 0x23;
            });
        EXPECT_CALL(bus, rawRead(0x50, _, 4))
            .WillOnce([](uint8_t, uint8_t* data, size_t len) {
                if (len != 4) return false;
                data[0] = 1; data[1] = 2; data[2] = 3; data[3] = 4;
                return true;
            });
    }

    uint8_t out[4] = {};
    ASSERT_TRUE(nv.read(0x0123, out, 4));
    EXPECT_EQ(out[0], 1);
    EXPECT_EQ(out[3], 4);
}

TEST(AT24CxxNvStoreTest, ReadOutOfBoundsFails) {
    MockI2CBus bus;
    MockDelayProvider delay;
    AT24CxxConfig cfg;
    cfg.totalBytes = 64;
    AT24CxxNvStore nv(bus, delay, cfg);

    EXPECT_CALL(bus, rawWrite(_, _, _)).Times(0);
    EXPECT_CALL(bus, rawRead(_, _, _)).Times(0);

    uint8_t out[4] = {};
    EXPECT_FALSE(nv.read(62, out, 4));
}

TEST(AT24CxxNvStoreTest, WriteSplitsAcrossPageBoundary) {
    MockI2CBus bus;
    MockDelayProvider delay;
    AT24CxxConfig cfg;
    cfg.pageSize = 16;
    AT24CxxNvStore nv(bus, delay, cfg);

    const uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};

    {
        InSequence seq;
        EXPECT_CALL(bus, rawWrite(0x50, _, 4))
            .WillOnce([](uint8_t, const uint8_t* p, size_t len) {
                return len == 4 &&
                       p[0] == 0x00 && p[1] == 0x0E &&
                       p[2] == 0xAA && p[3] == 0xBB;
            });
        EXPECT_CALL(delay, delayMs(5));
        EXPECT_CALL(bus, rawWrite(0x50, _, 4))
            .WillOnce([](uint8_t, const uint8_t* p, size_t len) {
                return len == 4 &&
                       p[0] == 0x00 && p[1] == 0x10 &&
                       p[2] == 0xCC && p[3] == 0xDD;
            });
        EXPECT_CALL(delay, delayMs(5));
    }

    EXPECT_TRUE(nv.write(14, data, 4));
}

TEST(AT24CxxNvStoreTest, WriteOutOfBoundsFails) {
    MockI2CBus bus;
    MockDelayProvider delay;
    AT24CxxConfig cfg;
    cfg.totalBytes = 64;
    AT24CxxNvStore nv(bus, delay, cfg);

    const uint8_t data[4] = {1, 2, 3, 4};
    EXPECT_FALSE(nv.write(63, data, 4));
}
