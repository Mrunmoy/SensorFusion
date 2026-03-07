#include <array>
#include <cstring>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "AT24CxxNvStore.hpp"
#include "CalibrationStore.hpp"
#include "MockDelayProvider.hpp"
#include "MockI2CBus.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;

namespace {

struct EepromBusModel {
    std::array<uint8_t, 256> mem{};
    uint16_t pointer = 0;

    EepromBusModel() { mem.fill(0xFF); }

    bool onWrite(const uint8_t* data, size_t len) {
        if (!data || len < 2) return false;
        const uint16_t addr = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
        if (len == 2) {
            if (addr >= mem.size()) return false;
            pointer = addr;
            return true;
        }

        const size_t payloadLen = len - 2;
        if (static_cast<size_t>(addr) + payloadLen > mem.size()) return false;
        std::memcpy(mem.data() + addr, data + 2, payloadLen);
        pointer = static_cast<uint16_t>(addr + payloadLen);
        return true;
    }

    bool onRead(uint8_t* data, size_t len) {
        if (!data) return false;
        if (static_cast<size_t>(pointer) + len > mem.size()) return false;
        std::memcpy(data, mem.data() + pointer, len);
        pointer = static_cast<uint16_t>(pointer + len);
        return true;
    }
};

} // namespace

TEST(CalibrationStoreAT24CxxTest, SaveAndLoadRoundTrip) {
    MockI2CBus bus;
    MockDelayProvider delay;
    EepromBusModel model;

    ON_CALL(bus, rawWrite(0x50, _, _))
        .WillByDefault([&](uint8_t, const uint8_t* data, size_t len) {
            return model.onWrite(data, len);
        });
    ON_CALL(bus, rawRead(0x50, _, _))
        .WillByDefault([&](uint8_t, uint8_t* data, size_t len) {
            return model.onRead(data, len);
        });

    AT24CxxConfig cfg;
    cfg.totalBytes = model.mem.size();
    cfg.pageSize = 16;
    AT24CxxNvStore nv(bus, delay, cfg);
    CalibrationStore store(nv, 32);

    CalibrationData in;
    in.offsetX = 1.2f;
    in.offsetY = -0.7f;
    in.scaleX = 1.01f;
    in.scaleY = 0.98f;

    ASSERT_TRUE(store.save(SensorId::ACCEL, in));

    CalibrationData out;
    ASSERT_TRUE(store.load(SensorId::ACCEL, out));
    EXPECT_FLOAT_EQ(out.offsetX, in.offsetX);
    EXPECT_FLOAT_EQ(out.offsetY, in.offsetY);
    EXPECT_FLOAT_EQ(out.scaleX, in.scaleX);
    EXPECT_FLOAT_EQ(out.scaleY, in.scaleY);
}

TEST(CalibrationStoreAT24CxxTest, CorruptedPayloadFailsLoad) {
    MockI2CBus bus;
    MockDelayProvider delay;
    EepromBusModel model;

    ON_CALL(bus, rawWrite(0x50, _, _))
        .WillByDefault([&](uint8_t, const uint8_t* data, size_t len) {
            return model.onWrite(data, len);
        });
    ON_CALL(bus, rawRead(0x50, _, _))
        .WillByDefault([&](uint8_t, uint8_t* data, size_t len) {
            return model.onRead(data, len);
        });

    AT24CxxConfig cfg;
    cfg.totalBytes = model.mem.size();
    cfg.pageSize = 16;
    AT24CxxNvStore nv(bus, delay, cfg);
    CalibrationStore store(nv, 32);

    CalibrationData in;
    in.offsetZ = 0.44f;
    ASSERT_TRUE(store.save(SensorId::GYRO, in));

    const uint32_t slot = 32u + static_cast<uint32_t>(SensorId::GYRO) * CalibrationStore::SLOT_SIZE;
    model.mem[slot + 8] ^= 0x5A;

    CalibrationData out;
    EXPECT_FALSE(store.load(SensorId::GYRO, out));
}
