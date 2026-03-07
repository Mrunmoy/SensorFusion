#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <limits>
#include "MockNvStore.hpp"
#include "CalibrationStore.hpp"

using namespace sf;
using namespace sf::test;
using ::testing::_;
using ::testing::Return;

class CalibrationStoreTest : public ::testing::Test {
protected:
    MockNvStore nv;

    void SetUp() override {
        nv.useBackingStore(1024);
    }
};

TEST_F(CalibrationStoreTest, SaveAndLoadRoundTrip) {
    CalibrationStore store(nv);

    CalibrationData cal;
    cal.offsetX = 0.5f;
    cal.offsetY = -0.3f;
    cal.offsetZ = 0.1f;
    cal.scaleX  = 1.01f;
    cal.scaleY  = 0.99f;
    cal.scaleZ  = 1.0f;

    EXPECT_TRUE(store.save(SensorId::ACCEL, cal));

    CalibrationData loaded;
    EXPECT_TRUE(store.load(SensorId::ACCEL, loaded));
    EXPECT_FLOAT_EQ(loaded.offsetX, 0.5f);
    EXPECT_FLOAT_EQ(loaded.offsetY, -0.3f);
    EXPECT_FLOAT_EQ(loaded.scaleX, 1.01f);
    EXPECT_FLOAT_EQ(loaded.scaleY, 0.99f);
}

TEST_F(CalibrationStoreTest, DifferentSensorsIndependent) {
    CalibrationStore store(nv);

    CalibrationData accelCal;
    accelCal.offsetX = 1.0f;
    EXPECT_TRUE(store.save(SensorId::ACCEL, accelCal));

    CalibrationData gyroCal;
    gyroCal.offsetX = 2.0f;
    EXPECT_TRUE(store.save(SensorId::GYRO, gyroCal));

    CalibrationData loadedAccel, loadedGyro;
    EXPECT_TRUE(store.load(SensorId::ACCEL, loadedAccel));
    EXPECT_TRUE(store.load(SensorId::GYRO, loadedGyro));
    EXPECT_FLOAT_EQ(loadedAccel.offsetX, 1.0f);
    EXPECT_FLOAT_EQ(loadedGyro.offsetX, 2.0f);
}

TEST_F(CalibrationStoreTest, BaroSlotRoundTripForSeaLevelReference) {
    CalibrationStore store(nv);

    CalibrationData baroCal;
    baroCal.offsetX = 1013.25f; // sea-level pressure reference
    baroCal.scaleX  = 1.0f;
    EXPECT_TRUE(store.save(SensorId::BARO, baroCal));

    CalibrationData loaded;
    EXPECT_TRUE(store.load(SensorId::BARO, loaded));
    EXPECT_FLOAT_EQ(loaded.offsetX, 1013.25f);
}

TEST_F(CalibrationStoreTest, LoadFromEmptyStoreFails) {
    CalibrationStore store(nv);
    CalibrationData data;
    EXPECT_FALSE(store.load(SensorId::ACCEL, data));
}

TEST_F(CalibrationStoreTest, IsValidReturnsTrueAfterSave) {
    CalibrationStore store(nv);
    EXPECT_FALSE(store.isValid(SensorId::MAG));

    CalibrationData cal;
    EXPECT_TRUE(store.save(SensorId::MAG, cal));
    EXPECT_TRUE(store.isValid(SensorId::MAG));
}

TEST_F(CalibrationStoreTest, ResetInvalidatesSlot) {
    CalibrationStore store(nv);

    CalibrationData cal;
    EXPECT_TRUE(store.save(SensorId::ACCEL, cal));
    EXPECT_TRUE(store.isValid(SensorId::ACCEL));

    EXPECT_TRUE(store.reset(SensorId::ACCEL));
    EXPECT_FALSE(store.isValid(SensorId::ACCEL));
}

TEST_F(CalibrationStoreTest, CorruptedCrcDetected) {
    CalibrationStore store(nv);

    CalibrationData cal;
    cal.offsetX = 42.0f;
    EXPECT_TRUE(store.save(SensorId::ACCEL, cal));

    // Corrupt one byte in the stored data
    uint8_t corrupt = 0xAA;
    nv.write(5, &corrupt, 1); // Corrupt inside calibration data

    CalibrationData loaded;
    EXPECT_FALSE(store.load(SensorId::ACCEL, loaded));
}

TEST_F(CalibrationStoreTest, Crc32KnownVector) {
    // "123456789" → CRC-32 = 0xCBF43926
    const uint8_t data[] = "123456789";
    uint32_t crc = CalibrationStore::crc32(data, 9);
    EXPECT_EQ(crc, 0xCBF43926u);
}

TEST_F(CalibrationStoreTest, ApplyCalibrationToAccel) {
    CalibrationStore store(nv);

    CalibrationData cal;
    cal.offsetX = 0.1f;
    cal.offsetY = 0.0f;
    cal.offsetZ = -0.05f;
    cal.scaleX  = 1.0f;
    cal.scaleY  = 1.0f;
    cal.scaleZ  = 1.0f;

    AccelData a{1.1f, 0.0f, 0.95f};
    store.apply(cal, a);
    EXPECT_NEAR(a.x, 1.0f, 0.001f);
    EXPECT_NEAR(a.z, 1.0f, 0.001f);
}

TEST_F(CalibrationStoreTest, ApplyCalibrationToMag) {
    CalibrationStore store(nv);

    CalibrationData cal;
    cal.offsetX = 10.0f;
    cal.scaleX  = 0.5f;
    cal.scaleY  = 1.0f;
    cal.scaleZ  = 1.0f;

    MagData m{30.0f, 0.0f, 0.0f};
    store.apply(cal, m);
    EXPECT_NEAR(m.x, 10.0f, 0.001f); // (30-10)*0.5 = 10
}

TEST_F(CalibrationStoreTest, NvWriteFailure) {
    MockNvStore failNv;
    EXPECT_CALL(failNv, write(_, _, _)).WillOnce(Return(false));

    CalibrationStore store(failNv);
    CalibrationData cal;
    EXPECT_FALSE(store.save(SensorId::ACCEL, cal));
}

TEST_F(CalibrationStoreTest, InvalidScaleRejectedOnSave) {
    CalibrationStore store(nv);

    CalibrationData invalid;
    invalid.scaleX = 0.0f; // invalid
    EXPECT_FALSE(store.save(SensorId::ACCEL, invalid));
}

TEST_F(CalibrationStoreTest, NonFiniteValueRejectedOnSave) {
    CalibrationStore store(nv);

    CalibrationData invalid;
    invalid.offsetY = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(store.save(SensorId::GYRO, invalid));
}

TEST_F(CalibrationStoreTest, LoadOrDefaultUsesStoredCalibrationWhenValid) {
    CalibrationStore store(nv);
    CalibrationData cal;
    cal.offsetX = 0.25f;
    EXPECT_TRUE(store.save(SensorId::MAG, cal));

    CalibrationData out;
    CalibrationData defaults;
    defaults.offsetX = -1.0f;

    EXPECT_TRUE(store.loadOrDefault(SensorId::MAG, out, defaults));
    EXPECT_FLOAT_EQ(out.offsetX, 0.25f);
}

TEST_F(CalibrationStoreTest, LoadOrDefaultFallsBackWhenCorrupt) {
    CalibrationStore store(nv);
    CalibrationData cal;
    cal.offsetX = 0.4f;
    EXPECT_TRUE(store.save(SensorId::ACCEL, cal));

    // Corrupt saved bytes to force load failure.
    uint8_t corrupt = 0x00;
    nv.write(6, &corrupt, 1);

    CalibrationData out;
    CalibrationData defaults;
    defaults.offsetX = -0.75f;
    defaults.scaleX = 1.2f;

    EXPECT_FALSE(store.loadOrDefault(SensorId::ACCEL, out, defaults));
    EXPECT_FLOAT_EQ(out.offsetX, defaults.offsetX);
    EXPECT_FLOAT_EQ(out.scaleX, defaults.scaleX);
}
