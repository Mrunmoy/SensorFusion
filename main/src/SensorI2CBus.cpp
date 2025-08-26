#include "SensorI2CBus.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "SensorI2CBus";
namespace
{
// RAII lock for the bus mutex
class BusLock
{
  public:
	BusLock(SemaphoreHandle_t m, TickType_t to)
		: m_(m)
	{
		ok_ = (m_ != nullptr) && xSemaphoreTake(m_, to);
	}
	~BusLock()
	{
		if (ok_)
			xSemaphoreGive(m_);
	}
	explicit operator bool() const
	{
		return ok_;
	}

  private:
	SemaphoreHandle_t m_;
	bool ok_{false};
};
} // namespace

SensorI2CBus::SensorI2CBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
						   uint32_t freqHz)
	: m_port(port), m_sda(sda), m_scl(scl), m_freqHz(freqHz)
{
}

bool SensorI2CBus::init()
{
	// Save config so we can reinstall on recover()
	std::memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.mode = I2C_MODE_MASTER;
	m_cfg.sda_io_num = m_sda;
	m_cfg.scl_io_num = m_scl;
	m_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
	m_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
	m_cfg.master.clk_speed = m_freqHz;

	esp_err_t err = i2c_param_config(m_port, &m_cfg);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
		return false;
	}
	err = i2c_driver_install(m_port, m_cfg.mode, 0, 0, 0);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
		return false;
	}

	if (!m_mutex)
	{
		m_mutex = xSemaphoreCreateMutex();
		if (!m_mutex)
		{
			ESP_LOGE(TAG, "Failed to create mutex");
			i2c_driver_delete(m_port);
			return false;
		}
	}
	m_inited = true;
	ESP_LOGI(TAG, "I2C ready on port %d (SDA=%d, SCL=%d, %lu Hz)", m_port,
			 (int)m_sda, (int)m_scl, (unsigned long)m_freqHz);
	return true;
}

void SensorI2CBus::deinit()
{
	if (m_inited)
	{
		i2c_driver_delete(m_port);
		m_inited = false;
	}
}

bool SensorI2CBus::isTransient(esp_err_t err)
{
	// TIMEOUT / FAIL usually indicate bus contention or device not ready yet
	return (err == ESP_ERR_TIMEOUT) || (err == ESP_FAIL);
}

bool SensorI2CBus::write(uint8_t deviceAddr, uint8_t regAddr,
						 const uint8_t *data, size_t len, TickType_t timeout)
{
	// Retry-at-most-once on transient error
	for (int attempt = 0; attempt < 2; ++attempt)
	{
		if (doWrite(deviceAddr, regAddr, data, len, timeout))
			return true;
		if (!busRecover())
			break; // non-transient or recover failed
	}
	return false;
}

bool SensorI2CBus::read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data,
						size_t len, TickType_t timeout)
{
	for (int attempt = 0; attempt < 2; ++attempt)
	{
		if (doRead(deviceAddr, regAddr, data, len, timeout))
			return true;
		if (!busRecover())
			break;
	}
	return false;
}

bool SensorI2CBus::writeRead(uint8_t deviceAddr, const uint8_t *wdata,
							 size_t wlen, uint8_t *rdata, size_t rlen,
							 TickType_t timeout)
{
	for (int attempt = 0; attempt < 2; ++attempt)
	{
		if (doWriteRead(deviceAddr, wdata, wlen, rdata, rlen, timeout))
			return true;
		if (!busRecover())
			break;
	}
	return false;
}

bool SensorI2CBus::write8(uint8_t deviceAddr, uint8_t regAddr, uint8_t value,
						  TickType_t timeout)
{
	return write(deviceAddr, regAddr, &value, 1, timeout);
}

bool SensorI2CBus::read8(uint8_t deviceAddr, uint8_t regAddr, uint8_t &out,
						 TickType_t timeout)
{
	return read(deviceAddr, regAddr, &out, 1, timeout);
}

bool SensorI2CBus::readBurst(uint8_t deviceAddr, uint8_t startReg,
							 uint8_t *data, size_t len, TickType_t timeout)
{
	return read(deviceAddr, startReg, data, len, timeout);
}

bool SensorI2CBus::ping(uint8_t deviceAddr, TickType_t timeout)
{
	if (!m_inited)
		return false;

	BusLock lk(m_mutex, timeout);
	if (!lk)
		return false;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	// Write phase: only the address is sent; some devices require a register ->
	// using 0x00 is usually harmless
	i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);
	esp_err_t err = i2c_master_cmd_begin(m_port, cmd, timeout);
	i2c_cmd_link_delete(cmd);
	return (err == ESP_OK);
}

// ======== Internal ops (single attempt) ========

bool SensorI2CBus::doWrite(uint8_t dev, uint8_t reg, const uint8_t *data,
						   size_t len, TickType_t timeout)
{
	if (!m_inited)
		return false;

	BusLock lk(m_mutex, timeout);
	if (!lk)
		return false;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	if (len && data)
	{
		i2c_master_write(cmd, const_cast<uint8_t *>(data), len, true);
	}
	i2c_master_stop(cmd);

	esp_err_t err = i2c_master_cmd_begin(m_port, cmd, timeout);
	i2c_cmd_link_delete(cmd);

	if (err != ESP_OK)
	{
		ESP_LOGW(TAG, "I2C write dev 0x%02X reg 0x%02X failed: %s", dev, reg,
				 esp_err_to_name(err));
		if (isTransient(err))
			return false;
		return false;
	}
	return true;
}

bool SensorI2CBus::doRead(uint8_t dev, uint8_t reg, uint8_t *data, size_t len,
						  TickType_t timeout)
{
	if (!m_inited || !data || len == 0)
		return false;

	BusLock lk(m_mutex, timeout);
	if (!lk)
		return false;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Write register
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);

	// Repeated start -> read payload
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
	if (len > 1)
	{
		i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	esp_err_t err = i2c_master_cmd_begin(m_port, cmd, timeout);
	i2c_cmd_link_delete(cmd);

	if (err != ESP_OK)
	{
		ESP_LOGW(TAG, "I2C read dev 0x%02X reg 0x%02X failed: %s", dev, reg,
				 esp_err_to_name(err));
		if (isTransient(err))
			return false;
		return false;
	}
	return true;
}

bool SensorI2CBus::doWriteRead(uint8_t dev, const uint8_t *wdata, size_t wlen,
							   uint8_t *rdata, size_t rlen, TickType_t timeout)
{
	if (!m_inited)
		return false;

	BusLock lk(m_mutex, timeout);
	if (!lk)
		return false;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Write portion
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
	if (wlen && wdata)
	{
		i2c_master_write(cmd, const_cast<uint8_t *>(wdata), wlen, true);
	}

	// Read portion (repeated start)
	if (rlen && rdata)
	{
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
		if (rlen > 1)
		{
			i2c_master_read(cmd, rdata, rlen - 1, I2C_MASTER_ACK);
		}
		i2c_master_read_byte(cmd, rdata + rlen - 1, I2C_MASTER_NACK);
	}
	i2c_master_stop(cmd);

	esp_err_t err = i2c_master_cmd_begin(m_port, cmd, timeout);
	i2c_cmd_link_delete(cmd);

	if (err != ESP_OK)
	{
		ESP_LOGW(TAG, "I2C writeRead dev 0x%02X failed: %s", dev,
				 esp_err_to_name(err));
		if (isTransient(err))
			return false;
		return false;
	}
	return true;
}

bool SensorI2CBus::busRecover()
{
	// Basic recovery: delete and reinstall the driver with saved config
	// This helps clear bus state after timeouts/NACK storms.
	i2c_driver_delete(m_port);
	esp_err_t err = i2c_param_config(m_port, &m_cfg);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Recover: param_config failed: %s", esp_err_to_name(err));
		return false;
	}
	err = i2c_driver_install(m_port, m_cfg.mode, 0, 0, 0);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Recover: driver_install failed: %s",
				 esp_err_to_name(err));
		return false;
	}
	ESP_LOGW(TAG, "I2C bus recovered on port %d", m_port);
	return true;
}
