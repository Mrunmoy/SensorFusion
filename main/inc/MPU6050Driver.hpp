#pragma once

#include "SensorI2CBus.hpp"
#include "SensorObserver.hpp"
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <vector>

class MPU6050Driver
{
  public:
	MPU6050Driver(SensorI2CBus &bus, uint8_t addr);

	bool init();
	bool readAcceleration(float &ax, float &ay, float &az);
	bool readGyroscope(float &gx, float &gy, float &gz);
	bool readTemperature(float &temp);
	bool sample(float &ax, float &ay, float &az, float &gx, float &gy,
				float &gz);

	void addObserver(SensorObserver *observer);
	// ---- Enable BYPASS so the magnetometer behind AUX I2C is visible on main
	// I2C ----
	bool setBypass(bool enable);

  private:
	bool writeRegister(uint8_t reg, uint8_t value);
	bool readRawData(uint8_t start_reg, int16_t &x, int16_t &y, int16_t &z);
	void notifyObservers();

	SensorI2CBus &m_bus;
	uint8_t m_addr;
	std::vector<SensorObserver *> m_observers;
};
