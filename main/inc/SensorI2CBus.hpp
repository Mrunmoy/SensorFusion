#pragma once

#include "driver/i2c.h"
#include "freertos/semphr.h"
#include <cstdint>

class SensorI2CBus
{
public:
    SensorI2CBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freqHz = 400000);
    bool init();

    bool write(uint8_t deviceAddr, uint8_t regAddr, const uint8_t* data, size_t len, TickType_t timeout = pdMS_TO_TICKS(100));
    bool read(uint8_t deviceAddr, uint8_t regAddr, uint8_t* data, size_t len, TickType_t timeout = pdMS_TO_TICKS(100));

private:
    i2c_port_t m_port;
    gpio_num_t m_sda;
    gpio_num_t m_scl;
    uint32_t m_freqHz;

    SemaphoreHandle_t m_mutex = nullptr;
};
