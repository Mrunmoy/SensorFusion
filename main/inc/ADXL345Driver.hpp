#pragma once

#include "SensorI2CBus.hpp"
#include "SensorObserver.hpp"
#include "freertos/FreeRTOS.h"
#include <atomic>
#include <cstdint>
#include <vector>

class ADXL345Driver
{
  public:
	ADXL345Driver(SensorI2CBus &bus, uint8_t addr = 0x53);

	bool init();
	bool readAcceleration(float &x, float &y, float &z);

	void addObserver(SensorObserver *observer);

  private:
	bool writeRegister(uint8_t reg, uint8_t value);
	bool readRegisters(uint8_t reg, uint8_t *data, size_t len);

	void notifyObservers();

	SensorI2CBus &m_bus;
	uint8_t m_addr;
	std::vector<SensorObserver *> m_observers;
};