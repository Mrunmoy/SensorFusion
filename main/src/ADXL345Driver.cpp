
#include "ADXL345Driver.hpp"
#include "esp_log.h"

static const char* TAG = "ADXL345Driver";

ADXL345Driver::ADXL345Driver(SensorI2CBus& bus, uint8_t address)
    : m_bus(bus), m_addr(address), m_taskHandle(nullptr), m_intervalMs(0)
{}

bool ADXL345Driver::init()
{
    ESP_LOGI(TAG, "Starting ADXL345 init...");

    // 1. Check DEVID (0x00) — should return 0xE5
    uint8_t devid = 0;
    if (!m_bus.read(m_addr, 0x00, &devid, 1)) {
        ESP_LOGE(TAG, "Failed to read DEVID register");
        return false;
    }

    ESP_LOGI(TAG, "DEVID = 0x%02X", devid);
    if (devid != 0xE5) {
        ESP_LOGE(TAG, "Unexpected DEVID value, expected 0xE5");
        return false;
    }

    // 2. POWER_CTL (0x2D): Enable measurement
    uint8_t powerCtl = 0x08;
    if (!m_bus.write(m_addr, 0x2D, &powerCtl, 1)) {
        ESP_LOGE(TAG, "Failed to write POWER_CTL");
        return false;
    }

    // 3. DATA_FORMAT (0x31): Full resolution, ±4g
    uint8_t dataFmt = 0x01 | (1 << 3);  // range = ±4g, FULL_RES = 1
    if (!m_bus.write(m_addr, 0x31, &dataFmt, 1)) {
        ESP_LOGE(TAG, "Failed to write DATA_FORMAT");
        return false;
    }

    // 4. BW_RATE (0x2C): 100 Hz
    uint8_t bwRate = 0x0A;
    if (!m_bus.write(m_addr, 0x2C, &bwRate, 1)) {
        ESP_LOGE(TAG, "Failed to write BW_RATE");
        return false;
    }

    ESP_LOGI(TAG, "ADXL345 initialized successfully");
    return true;
}


bool ADXL345Driver::readAcceleration(float& ax, float& ay, float& az)
{
    uint8_t buffer[6];
    if (!m_bus.read(m_addr, 0x32, buffer, 6)) // DATAX0 register
        return false;

    int16_t x = (buffer[1] << 8) | buffer[0];
    int16_t y = (buffer[3] << 8) | buffer[2];
    int16_t z = (buffer[5] << 8) | buffer[4];

    // Scale factor: 4 mg/LSB (±2g mode)
    ax = x * 0.004f * 9.80665f;
    ay = y * 0.004f * 9.80665f;
    az = z * 0.004f * 9.80665f;
    return true;
}

void ADXL345Driver::startPolling(uint32_t intervalMs)
{
    if (m_running)
    {
        ESP_LOGW(TAG, "Polling already started");
        return;
    }
    m_running = true;
    m_intervalMs = intervalMs;

    xTaskCreate(
        &ADXL345Driver::pollingTask,
        "ADXL345Task",
        4096,
        this,
        5,
        &m_taskHandle
    );
}

void ADXL345Driver::stopPolling()
{
    if (m_taskHandle) {
        m_running = false;
        // Task will delete itself after next delay
        while (m_taskHandle != nullptr) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void ADXL345Driver::pollingTask(void* arg)
{
    ADXL345Driver* self = static_cast<ADXL345Driver*>(arg);
    while (self->m_running) {
        float ax, ay, az;
        if (self->readAcceleration(ax, ay, az)) {
            ESP_LOGI(TAG, "Accel: ax=%.2f ay=%.2f az=%.2f", ax, ay, az);
            self->notifyObservers();
        } else {
            ESP_LOGW(TAG, "Failed to read acceleration data");
        }

        vTaskDelay(pdMS_TO_TICKS(self->m_intervalMs));
    }
}

void ADXL345Driver::notifyObservers() {
    for (auto* obs : m_observers) {
        obs->onSensorUpdated(); // fire callback
    }
}
