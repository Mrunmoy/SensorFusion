#pragma once

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdint>

/**
 * Thread-safe I2C master wrapper with per-transaction mutex,
 * short timeouts, retry-once on transient errors, and basic recovery.
 *
 */
class SensorI2CBus
{
  public:
	// freqHz: 100'000 or 400'000 typical
	SensorI2CBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
				 uint32_t freqHz = 400000);

	// Initialize driver and create mutex. Returns false on failure.
	bool init();

	// Optional explicit cleanup (driver_delete). Safe to call multiple times.
	void deinit();

	// Basic register write: [dev][reg]=data[0..len-1]
	bool write(uint8_t deviceAddr, uint8_t regAddr, const uint8_t *data,
			   size_t len, TickType_t timeout = pdMS_TO_TICKS(10));

	// Basic register read: [dev][reg] -> read len bytes
	bool read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t len,
			  TickType_t timeout = pdMS_TO_TICKS(10));

	// Combined write-then-read without a register (rarely needed)
	bool writeRead(uint8_t deviceAddr, const uint8_t *wdata, size_t wlen,
				   uint8_t *rdata, size_t rlen,
				   TickType_t timeout = pdMS_TO_TICKS(10));

	// Convenience helpers
	bool write8(uint8_t deviceAddr, uint8_t regAddr, uint8_t value,
				TickType_t timeout = pdMS_TO_TICKS(10));
	bool read8(uint8_t deviceAddr, uint8_t regAddr, uint8_t &out,
			   TickType_t timeout = pdMS_TO_TICKS(10));
	bool readBurst(uint8_t deviceAddr, uint8_t startReg, uint8_t *data,
				   size_t len, TickType_t timeout = pdMS_TO_TICKS(10));

	// Optional: quick scan utility (returns true if device ACKs address)
	bool ping(uint8_t deviceAddr, TickType_t timeout = pdMS_TO_TICKS(5));

  private:
	// Internal helpers
	bool doWrite(uint8_t deviceAddr, uint8_t regAddr, const uint8_t *data,
				 size_t len, TickType_t timeout);
	bool doRead(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, size_t len,
				TickType_t timeout);
	bool doWriteRead(uint8_t deviceAddr, const uint8_t *wdata, size_t wlen,
					 uint8_t *rdata, size_t rlen, TickType_t timeout);

	bool busRecover(); // basic recovery: delete+reinstall driver

	static bool isTransient(esp_err_t err);

  private:
	i2c_port_t m_port;
	gpio_num_t m_sda;
	gpio_num_t m_scl;
	uint32_t m_freqHz;

	i2c_config_t m_cfg{}; // saved config for recover()
	SemaphoreHandle_t m_mutex{nullptr};
	bool m_inited{false};
};
