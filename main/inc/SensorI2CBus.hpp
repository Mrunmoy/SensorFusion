#pragma once

#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/i2c.h"
#include "esp_log.h"

// Simple I2C bus wrapper using the legacy driver/i2c.h API.
// Thread-safe via a bus-wide recursive mutex and RAII BusLock.
// Public API matches your earlier working code.

class SensorI2CBus
{
  public:
	SensorI2CBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
				 uint32_t freq_hz = 400000)
		: m_port(port), m_sda(sda), m_scl(scl), m_freq(freq_hz)
	{
	}

	~SensorI2CBus()
	{
		deinit();
	}

	bool init();
	void deinit();

	// Register-based transactions (your drivers already use these)
	bool read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len,
			  TickType_t timeout = pdMS_TO_TICKS(50));
	bool write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len,
			   TickType_t timeout = pdMS_TO_TICKS(50));

	// Small conveniences (optional)
	bool read8(uint8_t addr, uint8_t reg, uint8_t &val,
			   TickType_t timeout = pdMS_TO_TICKS(50))
	{
		return read(addr, reg, &val, 1, timeout);
	}

	bool write8(uint8_t addr, uint8_t reg, uint8_t val,
				TickType_t timeout = pdMS_TO_TICKS(50))
	{
		return write(addr, reg, &val, 1, timeout);
	}

	// Optional: very lightweight probe (no table allocation)
	bool probe(uint8_t addr, TickType_t timeout = pdMS_TO_TICKS(50));

	// Optional: try to recover the bus (called internally on hard errors)
	bool busRecover();

  private:
	static constexpr const char *TAG = "SensorI2CBus";

	// RAII lock for the bus-wide mutex
	class BusLock
	{
	  public:
		BusLock(SensorI2CBus &b, TickType_t to = portMAX_DELAY)
			: m_bus(b), m_locked(false)
		{
			if (m_bus.m_mutex)
			{
				m_locked =
					(xSemaphoreTakeRecursive(m_bus.m_mutex, to) == pdTRUE);
			}
		}
		~BusLock()
		{
			if (m_locked && m_bus.m_mutex)
			{
				xSemaphoreGiveRecursive(m_bus.m_mutex);
			}
		}
		BusLock(const BusLock &) = delete;
		BusLock &operator=(const BusLock &) = delete;

		bool locked() const
		{
			return m_locked;
		}

	  private:
		SensorI2CBus &m_bus;
		bool m_locked;
	};

	// One place to actually submit an I2C command
	esp_err_t cmd_begin(i2c_cmd_handle_t cmd, TickType_t timeout);

	i2c_port_t m_port;
	gpio_num_t m_sda;
	gpio_num_t m_scl;
	uint32_t m_freq;
	SemaphoreHandle_t m_mutex{nullptr};
	bool m_inited{false};
};
