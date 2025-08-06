#include "MPU6050Driver.hpp"
#include "esp_log.h"

static const char* TAG = "MPU6050Driver";

MPU6050Driver::MPU6050Driver(SensorI2CBus& bus, uint8_t addr)
    : m_bus(bus), m_addr(addr)
{}

bool MPU6050Driver::init()
{
    // Wake up the MPU6050Driver
    return writeRegister(0x6B, 0x00); // PWR_MGMT_1 register
}

bool MPU6050Driver::readAcceleration(float& ax, float& ay, float& az)
{
    int16_t raw_ax, raw_ay, raw_az;
    if (!readRawData(0x3B, raw_ax, raw_ay, raw_az)) return false;

    constexpr float scale = 16384.0f;
    ax = raw_ax / scale;
    ay = raw_ay / scale;
    az = raw_az / scale;
    return true;
}

bool MPU6050Driver::readGyroscope(float& gx, float& gy, float& gz)
{
    int16_t raw_gx, raw_gy, raw_gz;
    if (!readRawData(0x43, raw_gx, raw_gy, raw_gz)) return false;

    constexpr float scale = 131.0f;
    gx = raw_gx / scale;
    gy = raw_gy / scale;
    gz = raw_gz / scale;
    return true;
}

bool MPU6050Driver::readTemperature(float& temp)
{
    uint8_t data[2];
    if (!m_bus.read(m_addr, 0x41, data, 2)) {
        return false;
    }

    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    temp = (raw / 340.0) + 36.53;
    return true;
}

// --- Private helpers ---

bool MPU6050Driver::writeRegister(uint8_t reg, uint8_t value)
{
    return m_bus.write(m_addr, reg, &value, 1);
}

bool MPU6050Driver::readRawData(uint8_t start_reg, int16_t& x, int16_t& y, int16_t& z)
{
    uint8_t data[6];
    if (!m_bus.read(m_addr, start_reg, data, sizeof(data))) {
        return false;
    }

    x = (int16_t)((data[0] << 8) | data[1]);
    y = (int16_t)((data[2] << 8) | data[3]);
    z = (int16_t)((data[4] << 8) | data[5]);
    return true;
}


void MPU6050Driver::addObserver(SensorObserver* observer) {
    m_observers.push_back(observer);
}

void MPU6050Driver::startPolling(uint32_t intervalMs)
{
    if (m_running) {
        ESP_LOGW(TAG, "Polling already started");
        return;
    }
    m_running = true;
    m_intervalMs = intervalMs;

    xTaskCreate(
        &MPU6050Driver::pollingTask,
        "MPU6050Task",
        4096,
        this,
        5,
        &m_taskHandle
    );
}

void MPU6050Driver::pollingTask(void* arg)
{
    MPU6050Driver* self = static_cast<MPU6050Driver*>(arg);
    while (self->m_running) {
        float ax, ay, az, gx, gy, gz, temp;
        if (self->readAcceleration(ax, ay, az)) {
            ESP_LOGI(TAG, "Accel: ax=%.2f, ay=%.2f, az=%.2f", ax, ay, az);
        } else {
            ESP_LOGW(TAG, "Failed to read acceleration");
        }

        if (self->readGyroscope(gx, gy, gz)) {
            ESP_LOGI(TAG, "Gyro: gx=%.2f, gy=%.2f, gz=%.2f", gx, gy, gz);
        } else {
            ESP_LOGW(TAG, "Failed to read gyroscope");
        }

        if (self->readTemperature(temp)) {
            ESP_LOGI(TAG, "Temp: %.2f °C", temp);
        } else {
            ESP_LOGW(TAG, "Failed to read temperature");
        }
        self->notifyObservers();  // notify registered listeners

        vTaskDelay(pdMS_TO_TICKS(self->m_intervalMs));
    }
}

void MPU6050Driver::stopPolling() {
    if (m_taskHandle) {
        m_running = false;
        // Task will delete itself after next delay
        while (m_taskHandle != nullptr) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void MPU6050Driver::notifyObservers() {
    for (auto* obs : m_observers) {
        obs->onSensorUpdated(); // fire callback
    }
}
