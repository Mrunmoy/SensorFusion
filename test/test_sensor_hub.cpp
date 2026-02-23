#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "SensorHub.hpp"

using namespace sf;

// --- Mock sensors ---
class MockAccelGyro : public IAccelGyroSensor {
public:
    MOCK_METHOD(bool, readAccel, (AccelData&), (override));
    MOCK_METHOD(bool, readGyro, (GyroData&), (override));
    MOCK_METHOD(bool, readTemperature, (float&), (override));
};

class MockMag : public IMagSensor {
public:
    MOCK_METHOD(bool, readMag, (MagData&), (override));
};

class MockBaro : public IBaroSensor {
public:
    MOCK_METHOD(bool, readPressureHPa, (float&), (override));
};

// --- Tests ---

TEST(SensorHubTest, NothingRegisteredByDefault) {
    SensorHub hub;
    EXPECT_FALSE(hub.hasIMU());
    EXPECT_FALSE(hub.hasMag());
    EXPECT_FALSE(hub.hasBaro());
    EXPECT_FALSE(hub.hasECG());
}

TEST(SensorHubTest, ReadAccelWithNoIMUReturnsFalse) {
    SensorHub hub;
    AccelData a;
    EXPECT_FALSE(hub.readAccel(a));
}

TEST(SensorHubTest, ReadGyroWithNoIMUReturnsFalse) {
    SensorHub hub;
    GyroData g;
    EXPECT_FALSE(hub.readGyro(g));
}

TEST(SensorHubTest, ReadMagWithNoMagReturnsFalse) {
    SensorHub hub;
    MagData m;
    EXPECT_FALSE(hub.readMag(m));
}

TEST(SensorHubTest, ReadPressureWithNoBaroReturnsFalse) {
    SensorHub hub;
    float p;
    EXPECT_FALSE(hub.readPressure(p));
}

TEST(SensorHubTest, ReadECGWithNoECGReturnsFalse) {
    SensorHub hub;
    ECGSample s;
    EXPECT_FALSE(hub.readECG(s));
}

TEST(SensorHubTest, RegisterIMU) {
    SensorHub hub;
    MockAccelGyro imu;
    hub.setIMU(&imu);
    EXPECT_TRUE(hub.hasIMU());
}

TEST(SensorHubTest, RegisterMag) {
    SensorHub hub;
    MockMag mag;
    hub.setMag(&mag);
    EXPECT_TRUE(hub.hasMag());
}

TEST(SensorHubTest, RegisterBaro) {
    SensorHub hub;
    MockBaro baro;
    hub.setBaro(&baro);
    EXPECT_TRUE(hub.hasBaro());
}

TEST(SensorHubTest, ReadAccelDelegates) {
    SensorHub hub;
    MockAccelGyro imu;
    hub.setIMU(&imu);

    EXPECT_CALL(imu, readAccel(::testing::_))
        .WillOnce([](AccelData& out) {
            out = {1.0f, 2.0f, 3.0f};
            return true;
        });

    AccelData a;
    EXPECT_TRUE(hub.readAccel(a));
    EXPECT_FLOAT_EQ(a.x, 1.0f);
    EXPECT_FLOAT_EQ(a.y, 2.0f);
    EXPECT_FLOAT_EQ(a.z, 3.0f);
}

TEST(SensorHubTest, ReadGyroDelegates) {
    SensorHub hub;
    MockAccelGyro imu;
    hub.setIMU(&imu);

    EXPECT_CALL(imu, readGyro(::testing::_))
        .WillOnce([](GyroData& out) {
            out = {10.0f, 20.0f, 30.0f};
            return true;
        });

    GyroData g;
    EXPECT_TRUE(hub.readGyro(g));
    EXPECT_FLOAT_EQ(g.x, 10.0f);
}

TEST(SensorHubTest, ReadMagDelegates) {
    SensorHub hub;
    MockMag mag;
    hub.setMag(&mag);

    EXPECT_CALL(mag, readMag(::testing::_))
        .WillOnce([](MagData& out) {
            out = {50.0f, -25.0f, 10.0f};
            return true;
        });

    MagData m;
    EXPECT_TRUE(hub.readMag(m));
    EXPECT_FLOAT_EQ(m.x, 50.0f);
}

TEST(SensorHubTest, ReadPressureDelegates) {
    SensorHub hub;
    MockBaro baro;
    hub.setBaro(&baro);

    EXPECT_CALL(baro, readPressureHPa(::testing::_))
        .WillOnce([](float& hPa) {
            hPa = 1013.25f;
            return true;
        });

    float p;
    EXPECT_TRUE(hub.readPressure(p));
    EXPECT_FLOAT_EQ(p, 1013.25f);
}

TEST(SensorHubTest, IMUReadFailurePropagates) {
    SensorHub hub;
    MockAccelGyro imu;
    hub.setIMU(&imu);

    EXPECT_CALL(imu, readAccel(::testing::_)).WillOnce(::testing::Return(false));
    AccelData a;
    EXPECT_FALSE(hub.readAccel(a));
}

TEST(SensorHubTest, UnregisterSensor) {
    SensorHub hub;
    MockAccelGyro imu;
    hub.setIMU(&imu);
    EXPECT_TRUE(hub.hasIMU());

    hub.setIMU(nullptr);
    EXPECT_FALSE(hub.hasIMU());
}

TEST(SensorHubTest, MixedSensorConfiguration) {
    SensorHub hub;
    MockAccelGyro imu;
    MockBaro baro;
    hub.setIMU(&imu);
    hub.setBaro(&baro);

    EXPECT_TRUE(hub.hasIMU());
    EXPECT_FALSE(hub.hasMag());
    EXPECT_TRUE(hub.hasBaro());
    EXPECT_FALSE(hub.hasECG());
}

TEST(SensorHubSizeTest, SmallFootprint) {
    // 5 pointers = 40 bytes on 64-bit, 20 on 32-bit
    EXPECT_LE(sizeof(SensorHub), 5 * sizeof(void*));
}
