#include "MPU6050Driver.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "MPU6050Driver";

// MPU6050 registers
static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
static constexpr uint8_t REG_CONFIG       = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_INT_PIN_CFG  = 0x37;
static constexpr uint8_t REG_INT_ENABLE   = 0x38;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_TEMP_OUT_H   = 0x41;
static constexpr uint8_t REG_GYRO_XOUT_H  = 0x43;
static constexpr uint8_t REG_USER_CTRL    = 0x6A;
static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t REG_WHO_AM_I     = 0x75;

// Bits
static constexpr uint8_t BIT_I2C_BYPASS_EN = 1 << 1; // INT_PIN_CFG[1]
static constexpr uint8_t BIT_I2C_MST_EN    = 1 << 5; // USER_CTRL[5]

// Gyro/Accel scale factors for default ranges (±2g, ±250 dps)
static constexpr float ACCEL_LSB_2G = 16384.0f;   // LSB/g
static constexpr float GYRO_LSB_250 = 131.0f;     // LSB/(°/s)

MPU6050Driver::MPU6050Driver(SensorI2CBus& bus, uint8_t addr)
: m_bus(bus), m_addr(addr) {}

bool MPU6050Driver::init()
{
    // WHO_AM_I should be 0x68 (or 0x69 depending on AD0)
    uint8_t who = 0;
    {
        uint8_t tmp;
        if (!m_bus.read(m_addr, REG_WHO_AM_I, &tmp, 1)) {
            ESP_LOGE(TAG, "Failed to read WHO_AM_I");
            return false;
        }
        who = tmp;
    }
    ESP_LOGI(TAG, "WHO_AM_I=0x%02X", who);

    // Wake device (clear sleep)
    if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
        ESP_LOGE(TAG, "Failed to wake device");
        return false;
    }

    // Basic config: DLPF, sample rate, ranges
    if (!writeRegister(REG_SMPLRT_DIV, 0x07)) return false;      // ~1 kHz/(1+7) = 125 Hz
    if (!writeRegister(REG_CONFIG,      0x03)) return false;     // DLPF=3 (approx 44Hz accel, 42Hz gyro)
    if (!writeRegister(REG_GYRO_CONFIG, 0x00)) return false;     // ±250 dps
    if (!writeRegister(REG_ACCEL_CONFIG,0x00)) return false;     // ±2 g
    if (!writeRegister(REG_INT_ENABLE,  0x00)) return false;     // no INTs

    return true;
}

bool MPU6050Driver::writeRegister(uint8_t reg, uint8_t value)
{
    return m_bus.write(m_addr, reg, &value, 1);
}

bool MPU6050Driver::readRawData(uint8_t start_reg, int16_t& x, int16_t& y, int16_t& z)
{
    uint8_t buf[6];
    if (!m_bus.read(m_addr, start_reg, buf, sizeof(buf))) {
        return false;
    }
    x = (int16_t)((buf[0] << 8) | buf[1]);
    y = (int16_t)((buf[2] << 8) | buf[3]);
    z = (int16_t)((buf[4] << 8) | buf[5]);
    return true;
}

bool MPU6050Driver::readAcceleration(float& ax, float& ay, float& az)
{
    int16_t x, y, z;
    if (!readRawData(REG_ACCEL_XOUT_H, x, y, z)) return false;
    ax = static_cast<float>(x) / ACCEL_LSB_2G * 9.80665f;
    ay = static_cast<float>(y) / ACCEL_LSB_2G * 9.80665f;
    az = static_cast<float>(z) / ACCEL_LSB_2G * 9.80665f;
    return true;
}

bool MPU6050Driver::readGyroscope(float& gx, float& gy, float& gz)
{
    int16_t x, y, z;
    if (!readRawData(REG_GYRO_XOUT_H, x, y, z)) return false;
    gx = static_cast<float>(x) / GYRO_LSB_250; // deg/s
    gy = static_cast<float>(y) / GYRO_LSB_250;
    gz = static_cast<float>(z) / GYRO_LSB_250;
    return true;
}

bool MPU6050Driver::readTemperature(float& temp)
{
    uint8_t buf[2];
    if (!m_bus.read(m_addr, REG_TEMP_OUT_H, buf, sizeof(buf))) return false;
    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    // From datasheet: Temp in °C = (raw / 340) + 36.53
    temp = (static_cast<float>(raw) / 340.0f) + 36.53f;
    return true;
}

void MPU6050Driver::addObserver(SensorObserver* observer)
{
    m_observers.push_back(observer);
}

void MPU6050Driver::notifyObservers()
{
    for (auto* o : m_observers) {
        o->onSensorUpdated();
    }
}

void MPU6050Driver::startPolling(uint32_t intervalMs)
{
    if (m_running) {
        ESP_LOGW(TAG, "Polling already running");
        return;
    }
    m_intervalMs = intervalMs;
    m_running = true;

    xTaskCreate(
        &MPU6050Driver::pollingTask,
        "MPU6050Task",
        4096,
        this,
        5,
        &m_taskHandle
    );
}

void MPU6050Driver::stopPolling()
{
    if (m_taskHandle) {
        m_running = false;
        while (m_taskHandle != nullptr) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

void MPU6050Driver::pollingTask(void* arg)
{
    auto* self = static_cast<MPU6050Driver*>(arg);
    while (self->m_running) {
        float ax, ay, az, gx, gy, gz, t;
        bool okA = self->readAcceleration(ax, ay, az);
        bool okG = self->readGyroscope(gx, gy, gz);
        bool okT = self->readTemperature(t);

        if (okA && okG && okT) {
            ESP_LOGD(TAG, "A(%.2f,%.2f,%.2f) G(%.2f,%.2f,%.2f) T=%.2f",
                     ax, ay, az, gx, gy, gz, t);
            self->notifyObservers();
        } else {
            ESP_LOGW(TAG, "Read failed (A:%d G:%d T:%d)", okA, okG, okT);
        }
        vTaskDelay(pdMS_TO_TICKS(self->m_intervalMs));
    }
    self->m_taskHandle = nullptr;
    vTaskDelete(nullptr);
}

bool MPU6050Driver::setBypass(bool enable)
{
    // Make sure internal I2C master is OFF when enabling bypass
    if (enable && !writeRegister(REG_USER_CTRL, 0x00)) {
        return false;
    }

    uint8_t cfg = 0;
    if (!m_bus.read(m_addr, REG_INT_PIN_CFG, &cfg, 1))
        return false;
    if (enable)
        cfg |=  BIT_I2C_BYPASS_EN;
    else
        cfg &= ~BIT_I2C_BYPASS_EN;

    ESP_LOGI(TAG, "I2C bypass %s (INT_PIN_CFG=0x%02X)", (enable ? "enabled" : "disabled"), cfg);

    return writeRegister(REG_INT_PIN_CFG, cfg);
}
