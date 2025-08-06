#pragma once

#include "SensorI2CBus.hpp"
#include "SensorObserver.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <vector>

class MPU6050Driver {
public:
    MPU6050Driver(SensorI2CBus& bus, uint8_t addr);

    bool init();
    bool readAcceleration(float& ax, float& ay, float& az);
    bool readGyroscope(float& gx, float& gy, float& gz);
    bool readTemperature(float& temp);

    void addObserver(SensorObserver* observer);
    void startPolling(uint32_t intervalMs = 1000);
    void stopPolling();

private:
    static void pollingTask(void* arg);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRawData(uint8_t start_reg, int16_t& x, int16_t& y, int16_t& z);
    void notifyObservers();

    SensorI2CBus& m_bus;
    uint8_t m_addr;
    uint32_t m_intervalMs = 100;
    TaskHandle_t m_taskHandle = nullptr;
    std::atomic_bool m_running = false; 
    std::vector<SensorObserver*> m_observers;
};
