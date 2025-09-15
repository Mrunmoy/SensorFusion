#include "SensorI2CBus.hpp"

#include "esp_log.h"
#include <cstring> // memcpy

static const char *TAG = "SensorI2CBus";

// --------------------------- small helpers ---------------------------

// RAII guard for the recursive mutex that protects the device table and bus
// reset.
struct RecMutexGuard
{
	SemaphoreHandle_t mtx{nullptr};
	bool locked{false};

	RecMutexGuard(SemaphoreHandle_t m, TickType_t to)
		: mtx(m)
	{
		if (mtx)
			locked = (xSemaphoreTakeRecursive(mtx, to) == pdTRUE);
	}
	~RecMutexGuard()
	{
		if (locked)
			xSemaphoreGiveRecursive(mtx);
	}
	explicit operator bool() const
	{
		return locked;
	}
};

// Max payload we’ll stack-pack as [reg|data...] for a single transmit()
static constexpr size_t STACK_PACK_MAX = 128;

// --------------------------------------------------------------------

SensorI2CBus::SensorI2CBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
						   uint32_t default_hz)
	: m_port(port), m_sda(sda), m_scl(scl), m_default_speed(default_hz)
{
}

bool SensorI2CBus::init()
{
	m_mutex = xSemaphoreCreateRecursiveMutex();
	if (!m_mutex)
	{
		ESP_LOGE(TAG, "Failed to create bus mutex");
		return false;
	}

	i2c_master_bus_config_t bus_cfg = {};
	bus_cfg.i2c_port = m_port;
	bus_cfg.sda_io_num = m_sda;
	bus_cfg.scl_io_num = m_scl;
	bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
	bus_cfg.intr_priority = 0;
	bus_cfg.trans_queue_depth = 0; // default
	bus_cfg.glitch_ignore_cnt = 7; // small glitch filter

// Depending on IDF minor version, one of these flags may exist.
#ifdef I2C_BUS_FLAG_ENABLE_INTERNAL_PULLUP
	bus_cfg.flags |= I2C_BUS_FLAG_ENABLE_INTERNAL_PULLUP;
#elif defined(I2C_BUS_FLAG_ENABLE_PULLUP)
	bus_cfg.flags |= I2C_BUS_FLAG_ENABLE_PULLUP;
#endif

	esp_err_t err = i2c_new_master_bus(&bus_cfg, &m_bus);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
		vSemaphoreDelete(m_mutex);
		m_mutex = nullptr;
		return false;
	}

	ESP_LOGI(TAG, "I2C ready on port %d (SDA=%d, SCL=%d, %lu Hz)", (int)m_port,
			 (int)m_sda, (int)m_scl, (unsigned long)m_default_speed);
	return true;
}

void SensorI2CBus::deinit()
{
	// Guard device removals & bus deletion
	{
		RecMutexGuard lock(m_mutex, pdMS_TO_TICKS(200));
		if (lock)
		{
			for (size_t i = 0; i < m_num; ++i)
			{
				if (m_devs[i].handle)
				{
					i2c_master_bus_rm_device(m_devs[i].handle);
					m_devs[i].handle = nullptr;
				}
			}
			m_num = 0;

			if (m_bus)
			{
				i2c_del_master_bus(m_bus);
				m_bus = nullptr;
			}
		}
	}
	if (m_mutex)
	{
		vSemaphoreDelete(m_mutex);
		m_mutex = nullptr;
	}
}

SensorI2CBus::Dev *SensorI2CBus::find(uint8_t addr)
{
	for (size_t i = 0; i < m_num; ++i)
	{
		if (m_devs[i].addr == addr)
			return &m_devs[i];
	}
	return nullptr;
}

SensorI2CBus::Dev *SensorI2CBus::ensure(uint8_t addr, uint32_t scl_speed_hz)
{
	if (!m_bus)
	{
		ESP_LOGE(TAG, "ensure(0x%02X): bus not initialized", addr);
		return nullptr;
	}

	RecMutexGuard lock(m_mutex, pdMS_TO_TICKS(100));
	if (!lock)
	{
		ESP_LOGW(TAG, "ensure(0x%02X): device table busy", addr);
		return nullptr;
	}

	// 1) already present?
	if (Dev *d = find(addr))
		return d;

	// 2) capacity
	if (m_num >= MAX_DEV)
	{
		ESP_LOGE(TAG, "ensure(0x%02X): device table full (MAX_DEV=%u)", addr,
				 (unsigned)MAX_DEV);
		return nullptr;
	}

	// 3) add device
	i2c_device_config_t cfg = {};
	cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	cfg.device_address = addr;
	cfg.scl_speed_hz = (scl_speed_hz ? scl_speed_hz : m_default_speed);

	i2c_master_dev_handle_t handle = nullptr;
	esp_err_t err = i2c_master_bus_add_device(m_bus, &cfg, &handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "ensure(0x%02X): add_device failed: %s", addr,
				 esp_err_to_name(err));
		return nullptr;
	}

	// 4) cache & return stable pointer
	m_devs[m_num] = Dev{addr, cfg.scl_speed_hz, handle};
	return &m_devs[m_num++];
}

bool SensorI2CBus::addDevice(uint8_t addr, uint32_t scl_speed_hz)
{
	return ensure(addr, scl_speed_hz) != nullptr;
}

bool SensorI2CBus::reset_bus()
{
	if (!m_bus)
		return false;

	// Try non-blocking to avoid fighting with table edits
	RecMutexGuard lock(m_mutex, 0);
	if (!lock)
	{
		ESP_LOGW(TAG, "I2C bus reset skipped (device table busy)");
		return false;
	}

	esp_err_t err = i2c_master_bus_reset(m_bus);
	if (err != ESP_OK)
	{
		ESP_LOGW(TAG, "bus_reset failed: %s", esp_err_to_name(err));
		return false;
	}
	ESP_LOGW(TAG, "I2C bus reset performed");
	return true;
}

bool SensorI2CBus::write(uint8_t addr, uint8_t reg, const uint8_t *data,
						 size_t len, TickType_t timeout)
{
	auto *dev = ensure(addr, 0);
	if (!dev)
		return false;

	const int ms = ticks_to_ms(timeout);

	// Common case: small register write fits on stack as [reg | payload...]
	if (len + 1 <= STACK_PACK_MAX)
	{
		uint8_t buf[STACK_PACK_MAX];
		buf[0] = reg;
		if (len)
			std::memcpy(buf + 1, data, len);

		esp_err_t err = i2c_master_transmit(dev->handle, buf, len + 1, ms);
		if (err == ESP_ERR_TIMEOUT)
		{
			ESP_LOGW(TAG,
					 "I2C write timeout dev 0x%02X reg 0x%02X, resetting bus",
					 addr, reg);
			if (reset_bus())
				err = i2c_master_transmit(dev->handle, buf, len + 1, ms);
		}
		if (err != ESP_OK)
		{
			ESP_LOGW(TAG, "I2C write dev 0x%02X reg 0x%02X failed: %s", addr,
					 reg, esp_err_to_name(err));
			return false;
		}
		return true;
	}

	ESP_LOGE(
		TAG,
		"I2C write too large (%u bytes). Increase STACK_PACK_MAX or split.",
		(unsigned)len);
	return false;
}

bool SensorI2CBus::read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len,
						TickType_t timeout)
{
	auto *dev = ensure(addr, 0);
	if (!dev)
		return false;

	const int ms = ticks_to_ms(timeout);
	esp_err_t err =
		i2c_master_transmit_receive(dev->handle, &reg, 1, data, len, ms);
	if (err == ESP_ERR_TIMEOUT)
	{
		ESP_LOGW(TAG, "I2C read timeout dev 0x%02X reg 0x%02X, resetting bus",
				 addr, reg);
		if (reset_bus())
		{
			err = i2c_master_transmit_receive(dev->handle, &reg, 1, data, len,
											  ms);
		}
	}
	if (err != ESP_OK)
	{
		ESP_LOGW(TAG, "I2C read dev 0x%02X reg 0x%02X failed: %s", addr, reg,
				 esp_err_to_name(err));
		return false;
	}
	return true;
}

bool SensorI2CBus::write_raw(uint8_t addr, const uint8_t *data, size_t len,
							 TickType_t timeout)
{
	auto *dev = ensure(addr, 0);
	if (!dev)
		return false;

	const int ms = ticks_to_ms(timeout);
	esp_err_t err =
		i2c_master_transmit(dev->handle, const_cast<uint8_t *>(data), len, ms);
	if (err == ESP_ERR_TIMEOUT)
	{
		ESP_LOGW(TAG, "I2C tx timeout dev 0x%02X, resetting bus", addr);
		if (reset_bus())
		{
			err = i2c_master_transmit(dev->handle, const_cast<uint8_t *>(data),
									  len, ms);
		}
	}
	if (err != ESP_OK)
	{
		ESP_LOGW(TAG, "I2C tx dev 0x%02X failed: %s", addr,
				 esp_err_to_name(err));
		return false;
	}
	return true;
}

bool SensorI2CBus::read_raw(uint8_t addr, uint8_t *data, size_t len,
							TickType_t timeout)
{
	auto *dev = ensure(addr, 0);
	if (!dev)
		return false;

	const int ms = ticks_to_ms(timeout);
	esp_err_t err = i2c_master_receive(dev->handle, data, len, ms);
	if (err == ESP_ERR_TIMEOUT)
	{
		ESP_LOGW(TAG, "I2C rx timeout dev 0x%02X, resetting bus", addr);
		if (reset_bus())
		{
			err = i2c_master_receive(dev->handle, data, len, ms);
		}
	}
	if (err != ESP_OK)
	{
		ESP_LOGW(TAG, "I2C rx dev 0x%02X failed: %s", addr,
				 esp_err_to_name(err));
		return false;
	}
	return true;
}

// ---------------------- convenience helpers ----------------------

bool SensorI2CBus::write8(uint8_t addr, uint8_t reg, uint8_t val, TickType_t to)
{
	return write(addr, reg, &val, 1, to);
}

bool SensorI2CBus::read8(uint8_t addr, uint8_t reg, uint8_t &out, TickType_t to)
{
	return read(addr, reg, &out, 1, to);
}

bool SensorI2CBus::write16be(uint8_t addr, uint8_t reg, uint16_t val,
							 TickType_t to)
{
	uint8_t b[2] = {static_cast<uint8_t>((val >> 8) & 0xFF),
					static_cast<uint8_t>(val & 0xFF)};
	return write(addr, reg, b, 2, to);
}

bool SensorI2CBus::write16le(uint8_t addr, uint8_t reg, uint16_t val,
							 TickType_t to)
{
	uint8_t b[2] = {static_cast<uint8_t>(val & 0xFF),
					static_cast<uint8_t>((val >> 8) & 0xFF)};
	return write(addr, reg, b, 2, to);
}

bool SensorI2CBus::read16be(uint8_t addr, uint8_t reg, uint16_t &out,
							TickType_t to)
{
	uint8_t b[2];
	if (!read(addr, reg, b, 2, to))
		return false;
	out = (static_cast<uint16_t>(b[0]) << 8) | b[1];
	return true;
}

bool SensorI2CBus::read16le(uint8_t addr, uint8_t reg, uint16_t &out,
							TickType_t to)
{
	uint8_t b[2];
	if (!read(addr, reg, b, 2, to))
		return false;
	out = (static_cast<uint16_t>(b[1]) << 8) | b[0];
	return true;
}

bool SensorI2CBus::update_bits(uint8_t addr, uint8_t reg, uint8_t mask,
							   uint8_t set_bits, TickType_t to)
{
	uint8_t cur = 0;
	if (!read(addr, reg, &cur, 1, to))
		return false;
	uint8_t nxt = static_cast<uint8_t>((cur & ~mask) | (set_bits & mask));
	if (nxt == cur)
		return true; // no change
	return write(addr, reg, &nxt, 1, to);
}

bool SensorI2CBus::probe(uint8_t addr, TickType_t timeout)
{
	if (!m_bus)
		return false;
	const int ms = ticks_to_ms(timeout);
	esp_err_t err = i2c_master_probe(m_bus, addr, ms);
	return (err == ESP_OK);
}
