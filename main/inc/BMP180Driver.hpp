#pragma once
#include "SensorI2CBus.hpp"
#include "SensorObserver.hpp"
#include <atomic>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>

class BMP180Driver
{
  public:
	static constexpr uint8_t DEFAULT_ADDR = 0x77;

	enum class Oversampling : uint8_t
	{
		ULTRA_LOW_POWER = 0, // 4.5ms
		STANDARD = 1,		 // 7.5ms
		HIGH_RES = 2,		 // 13.5ms
		ULTRA_HIGH_RES = 3,	 // 25.5ms
	};

	BMP180Driver(SensorI2CBus &bus, uint8_t addr = DEFAULT_ADDR,
				 Oversampling oss = Oversampling::STANDARD);

	bool init();						  // reads calibration & checks chip id
	bool readTemperatureC(float &temp_c); // °C
	bool readPressurePa(int32_t &pressure_pa); // Pa
	bool readTempAndPressure(float &temp_c, int32_t &pressure_pa);

	// Polling API (like your other drivers)
	void startPolling(uint32_t intervalMs = 1000);
	void stopPolling();
	void addObserver(SensorObserver *obs);

	// Utility
	static float pressureToAltitudeMeters(int32_t pressure_pa,
										  int32_t p0_pa = 101325);

  private:
	// Registers / commands / IDs
	static constexpr uint8_t REG_CALIB_START = 0xAA; // 0xAA..0xBF
	static constexpr uint8_t REG_CHIP_ID = 0xD0;	 // expect 0x55
	static constexpr uint8_t REG_SOFT_RESET = 0xE0;
	static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
	static constexpr uint8_t REG_OUT_MSB = 0xF6; // F6..F8
	static constexpr uint8_t CMD_TEMP = 0x2E;
	static constexpr uint8_t CMD_PRESS_BASE = 0x34;
	static constexpr uint8_t CHIP_ID_EXPECT = 0x55;

	struct Calib
	{
		int16_t AC1;
		int16_t AC2;
		int16_t AC3;
		uint16_t AC4;
		uint16_t AC5;
		uint16_t AC6;
		int16_t B1;
		int16_t B2;
		int16_t MB;
		int16_t MC;
		int16_t MD;
		bool valid{false};
	} calib_{};

	// I2C helpers
	bool readU8(uint8_t reg, uint8_t &val);
	bool writeU8(uint8_t reg, uint8_t val);
	bool readBytes(uint8_t reg, uint8_t *buf, size_t len);

	// Raw conversions
	bool readUncompensatedTemp(int32_t &UT);
	bool readUncompensatedPressure(int32_t &UP);

	// Compensation
	bool computeTrueTempC(int32_t UT, float &temp_c, int32_t &B5);
	bool computeTruePressurePa(int32_t UP, int32_t B5, int32_t &P);

	// Polling
	static void pollingTask(void *arg);
	void notifyObservers();

	SensorI2CBus &m_bus;
	uint8_t m_addr;
	Oversampling m_oss;

	// Polling state
	std::atomic_bool m_running{false};
	uint32_t m_intervalMs{1000};
	TaskHandle_t m_taskHandle{nullptr};
	std::vector<SensorObserver *> m_observers;
};
