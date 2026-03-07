#include <algorithm>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <vector>

#include "INvStore.hpp"
#include "WearLevelingNvStore.hpp"

using namespace sf;

namespace {

class TracingNvStore : public INvStore {
public:
    explicit TracingNvStore(size_t size) : mem(size, 0xFF) {}

    bool read(uint32_t address, uint8_t* buf, size_t len) override {
        if (!buf) return false;
        if (static_cast<size_t>(address) + len > mem.size()) return false;
        std::memcpy(buf, mem.data() + address, len);
        return true;
    }

    bool write(uint32_t address, const uint8_t* data, size_t len) override {
        if (!data) return false;
        if (static_cast<size_t>(address) + len > mem.size()) return false;
        writeAddrs.push_back(address);
        std::memcpy(mem.data() + address, data, len);
        return true;
    }

    size_t capacity() const override { return mem.size(); }

    std::vector<uint8_t> mem;
    std::vector<uint32_t> writeAddrs;
};

} // namespace

TEST(WearLevelingNvStoreTest, CapacityMatchesLogicalSpace) {
    TracingNvStore raw(512);
    WearLevelingNvStoreConfig cfg;
    cfg.logicalPageSize = 16;
    cfg.logicalPageCount = 3;
    cfg.rotationDepth = 2;

    WearLevelingNvStore wl(raw, cfg);
    EXPECT_EQ(wl.capacity(), 48u);
}

TEST(WearLevelingNvStoreTest, WriteReadRoundTrip) {
    TracingNvStore raw(512);
    WearLevelingNvStoreConfig cfg;
    cfg.logicalPageSize = 16;
    cfg.logicalPageCount = 2;
    cfg.rotationDepth = 2;

    WearLevelingNvStore wl(raw, cfg);

    const uint8_t in[5] = {1, 2, 3, 4, 5};
    ASSERT_TRUE(wl.write(3, in, sizeof(in)));

    uint8_t out[5] = {};
    ASSERT_TRUE(wl.read(3, out, sizeof(out)));
    EXPECT_EQ(0, std::memcmp(in, out, sizeof(in)));
}

TEST(WearLevelingNvStoreTest, RepeatedWritesRotateSlots) {
    TracingNvStore raw(512);
    WearLevelingNvStoreConfig cfg;
    cfg.logicalPageSize = 8;
    cfg.logicalPageCount = 1;
    cfg.rotationDepth = 3;
    cfg.baseAddress = 16;

    WearLevelingNvStore wl(raw, cfg);
    uint8_t value = 0x10;
    ASSERT_TRUE(wl.write(0, &value, 1));
    value = 0x20;
    ASSERT_TRUE(wl.write(0, &value, 1));
    value = 0x30;
    ASSERT_TRUE(wl.write(0, &value, 1));

    const size_t recSize = WearLevelingNvStore::HEADER_SIZE + cfg.logicalPageSize;
    const std::vector<uint32_t> expected = {
        cfg.baseAddress + static_cast<uint32_t>(recSize * 0),
        cfg.baseAddress + static_cast<uint32_t>(recSize * 1),
        cfg.baseAddress + static_cast<uint32_t>(recSize * 2),
    };
    ASSERT_GE(raw.writeAddrs.size(), expected.size());
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), raw.writeAddrs.begin()));
}

TEST(WearLevelingNvStoreTest, CorruptedLatestSlotFallsBackToPrevious) {
    TracingNvStore raw(512);
    WearLevelingNvStoreConfig cfg;
    cfg.logicalPageSize = 8;
    cfg.logicalPageCount = 1;
    cfg.rotationDepth = 2;
    cfg.baseAddress = 0;

    WearLevelingNvStore wl(raw, cfg);

    uint8_t oldVal = 0x11;
    uint8_t newVal = 0x22;
    ASSERT_TRUE(wl.write(0, &oldVal, 1));
    ASSERT_TRUE(wl.write(0, &newVal, 1));

    const size_t recSize = WearLevelingNvStore::HEADER_SIZE + cfg.logicalPageSize;
    const size_t latestPayload = recSize + WearLevelingNvStore::HEADER_SIZE;
    raw.mem[latestPayload] ^= 0x5A;

    uint8_t out = 0;
    ASSERT_TRUE(wl.read(0, &out, 1));
    EXPECT_EQ(out, oldVal);
}

TEST(WearLevelingNvStoreTest, WriteAcrossPagesWorks) {
    TracingNvStore raw(1024);
    WearLevelingNvStoreConfig cfg;
    cfg.logicalPageSize = 8;
    cfg.logicalPageCount = 2;
    cfg.rotationDepth = 2;

    WearLevelingNvStore wl(raw, cfg);

    const uint8_t in[6] = {9, 8, 7, 6, 5, 4};
    ASSERT_TRUE(wl.write(6, in, sizeof(in)));

    uint8_t out[6] = {};
    ASSERT_TRUE(wl.read(6, out, sizeof(out)));
    EXPECT_EQ(0, std::memcmp(in, out, sizeof(in)));
}

TEST(WearLevelingNvStoreTest, OutOfBoundsRejected) {
    TracingNvStore raw(256);
    WearLevelingNvStoreConfig cfg;
    cfg.logicalPageSize = 8;
    cfg.logicalPageCount = 2;
    cfg.rotationDepth = 2;

    WearLevelingNvStore wl(raw, cfg);
    uint8_t b = 0x01;
    EXPECT_FALSE(wl.write(15, &b, 2));
    EXPECT_FALSE(wl.read(15, &b, 2));
}
