#pragma once

#include "SensorI2CBus.hpp"
#include "SensorObserver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
#include <cstdint>
#include <vector>

class ADXL345Driver {
public:
    ADXL345Driver(SensorI2CBus& bus, uint8_t addr = 0x53);

    bool init();
    bool readAcceleration(float& x, float& y, float& z);

    void addObserver(SensorObserver* observer);
    void startPolling(uint32_t intervalMs);
    void stopPolling();

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* data, size_t len);

    void notifyObservers();

    static void pollingTask(void* arg);

    SensorI2CBus& m_bus;
    uint8_t m_addr;
    TaskHandle_t m_taskHandle = nullptr;
    uint32_t m_intervalMs = 1000;
    std::atomic_bool m_running = false; 
    std::vector<SensorObserver*> m_observers;
};