#pragma once

#include "SensorI2CBus.hpp"
#include "SensorObserver.hpp"
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>

class QMC5883LDriver
{
  public:
	static constexpr uint8_t DEFAULT_ADDR = 0x0D;

	enum class Oversample : uint8_t
	{
		OSR_512,
		OSR_256,
		OSR_128,
		OSR_64
	};
	enum class Range : uint8_t
	{
		GAUSS_2,
		GAUSS_8
	};
	enum class ODR : uint8_t
	{
		HZ_10,
		HZ_50,
		HZ_100,
		HZ_200
	};
	enum class Mode : uint8_t
	{
		STANDBY,
		CONTINUOUS
	};

	explicit QMC5883LDriver(SensorI2CBus &bus, uint8_t addr = DEFAULT_ADDR);

	// Init with defaults (OSR512, 8G, 200Hz, continuous)
	bool init();

	// Optional reconfiguration at runtime
	bool configure(Oversample osr, Range rng, ODR odr, Mode mode);

	// Read APIs
	bool readRaw(int16_t &x, int16_t &y, int16_t &z,
				 uint32_t timeoutMs = 50) const;
	bool readMicroTesla(float &x_uT, float &y_uT, float &z_uT,
						uint32_t timeoutMs = 50) const;
	bool headingDegrees(float &heading_deg,
						uint32_t timeoutMs = 50) const; // level-only

	// Polling + observers (matches your MPU/BMP style)
	void startPolling(uint32_t intervalMs = 100);
	void stopPolling();
	void addObserver(SensorObserver *obs);

  private:
	// I2C helpers
	bool writeReg(uint8_t reg, uint8_t val) const;
	bool readReg(uint8_t reg, uint8_t *buf, size_t len) const;

	// Polling helpers
	static void pollingTask(void *arg);
	void notifyObservers();

	// Current configuration (for scaling)
	Oversample m_osr = Oversample::OSR_512;
	Range m_rng = Range::GAUSS_8;
	ODR m_odr = ODR::HZ_200;
	Mode m_mode = Mode::CONTINUOUS;

	SensorI2CBus &m_bus;
	uint8_t m_addr;

	// Polling state
	std::atomic_bool m_running{false};
	uint32_t m_intervalMs{100};
	TaskHandle_t m_taskHandle{nullptr};
	std::vector<SensorObserver *> m_observers;
};
