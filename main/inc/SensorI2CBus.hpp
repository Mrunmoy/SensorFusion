#pragma once

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstddef>
#include <cstdint>

/**
 * Thin wrapper around ESP-IDF v5 I2C master (bus+device) API.
 * - Normal I2C ops rely on the driver's own thread safety (no extra app lock).
 * - A small recursive mutex only guards: device-handle table edits & bus reset.
 * - Register helpers provided for convenience.
 */
class SensorI2CBus
{
  public:
	SensorI2CBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
				 uint32_t default_hz = 400000);

	bool init();
	void deinit();

	// Optionally pre-register a device at a specific speed (Hz). 0 => default
	// bus speed.
	bool addDevice(uint8_t addr, uint32_t scl_speed_hz = 0);

	// Register-based I/O (8-bit register address first)
	bool write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len,
			   TickType_t timeout = pdMS_TO_TICKS(50));
	bool read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len,
			  TickType_t timeout = pdMS_TO_TICKS(50));

	// Raw I/O (no register prefix)
	bool write_raw(uint8_t addr, const uint8_t *data, size_t len,
				   TickType_t timeout = pdMS_TO_TICKS(50));
	bool read_raw(uint8_t addr, uint8_t *data, size_t len,
				  TickType_t timeout = pdMS_TO_TICKS(50));

	// Convenience helpers
	bool write8(uint8_t addr, uint8_t reg, uint8_t val,
				TickType_t to = pdMS_TO_TICKS(50));
	bool read8(uint8_t addr, uint8_t reg, uint8_t &out,
			   TickType_t to = pdMS_TO_TICKS(50));
	bool write16be(uint8_t addr, uint8_t reg, uint16_t val,
				   TickType_t to = pdMS_TO_TICKS(50));
	bool write16le(uint8_t addr, uint8_t reg, uint16_t val,
				   TickType_t to = pdMS_TO_TICKS(50));
	bool read16be(uint8_t addr, uint8_t reg, uint16_t &out,
				  TickType_t to = pdMS_TO_TICKS(50));
	bool read16le(uint8_t addr, uint8_t reg, uint16_t &out,
				  TickType_t to = pdMS_TO_TICKS(50));
	bool update_bits(uint8_t addr, uint8_t reg, uint8_t mask, uint8_t set_bits,
					 TickType_t to = pdMS_TO_TICKS(50));

	bool probe(uint8_t addr, TickType_t timeout = pdMS_TO_TICKS(20));

  private:
	struct Dev
	{
		uint8_t addr = 0;
		uint32_t speed = 0; // Hz
		i2c_master_dev_handle_t handle = nullptr;
	};

	static constexpr size_t MAX_DEV = 8;

	Dev m_devs[MAX_DEV];
	size_t m_num = 0;

	Dev *find(uint8_t addr);
	Dev *ensure(uint8_t addr, uint32_t scl_speed_hz);
	bool reset_bus();

	static inline int ticks_to_ms(TickType_t t)
	{
		if (t == portMAX_DELAY)
			return -1;
		return static_cast<int>((uint64_t)t * 1000 / configTICK_RATE_HZ);
	}

	i2c_master_bus_handle_t m_bus = nullptr;
	i2c_port_t m_port;
	gpio_num_t m_sda;
	gpio_num_t m_scl;
	uint32_t m_default_speed;

	// Recursive mutex used ONLY for device table edits and bus reset.
	SemaphoreHandle_t m_mutex = nullptr;
};
