#include "SensorI2CBus.hpp"

#include "esp_check.h"
#include "freertos/task.h"

bool SensorI2CBus::init()
{
	if (m_inited)
		return true;

	// Create recursive mutex once
	if (!m_mutex)
	{
		m_mutex = xSemaphoreCreateRecursiveMutex();
		if (!m_mutex)
		{
			ESP_LOGE(TAG, "Failed to create bus mutex");
			return false;
		}
	}

	// Configure legacy I2C driver (IDF prints a migration warning, which is OK
	// for now)
	i2c_config_t cfg = {};
	cfg.mode = I2C_MODE_MASTER;
	cfg.sda_io_num = m_sda;
	cfg.scl_io_num = m_scl;
	cfg.sda_pullup_en = GPIO_PULLUP_ENABLE; // OK even if you have externals
	cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
	cfg.master.clk_speed = static_cast<uint32_t>(m_freq);

	ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_param_config(m_port, &cfg));
	esp_err_t err = i2c_driver_install(m_port, cfg.mode, 0, 0, 0);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "i2c_driver_install(%d) failed: %s", m_port,
				 esp_err_to_name(err));
		return false;
	}

	m_inited = true;
	ESP_LOGI(TAG, "I2C ready on port %d (SDA=%d, SCL=%d, %u Hz)",
			 static_cast<int>(m_port), static_cast<int>(m_sda),
			 static_cast<int>(m_scl), static_cast<unsigned>(m_freq));
	return true;
}

void SensorI2CBus::deinit()
{
	if (!m_inited)
		return;
	{
		BusLock lock(*this, pdMS_TO_TICKS(1000));
		(void)lock;
		i2c_driver_delete(m_port);
	}
	m_inited = false;
}

esp_err_t SensorI2CBus::cmd_begin(i2c_cmd_handle_t cmd, TickType_t timeout)
{
	// Submit and delete the command link in all cases
	esp_err_t err = i2c_master_cmd_begin(m_port, cmd, timeout);
	i2c_cmd_link_delete(cmd);
	return err;
}

bool SensorI2CBus::read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len,
						TickType_t timeout)
{
	if (!m_inited || !buf || len == 0)
		return false;

	BusLock lock(*this, timeout);
	if (!lock.locked())
		return false;

	auto do_xfer = [&]() -> esp_err_t
	{
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		if (!cmd)
			return ESP_ERR_NO_MEM;

		// Write register address
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, reg, true);

		// Repeated start, then read 'len' bytes
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
		if (len > 1)
		{
			i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
		}
		i2c_master_read_byte(cmd, buf + (len - 1), I2C_MASTER_NACK);
		i2c_master_stop(cmd);

		return cmd_begin(cmd, timeout);
	};

	esp_err_t err = do_xfer();
	if (err == ESP_OK)
		return true;

	// Log and try a single recovery+retry on hard errors
	ESP_LOGW(TAG, "I2C read dev 0x%02X reg 0x%02X failed: %s", addr, reg,
			 esp_err_to_name(err));
	if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL ||
		err == ESP_ERR_INVALID_STATE)
	{
		ESP_LOGW(TAG, "I2C bus reset (read error)");
		if (busRecover())
		{
			esp_err_t err2 = do_xfer();
			if (err2 == ESP_OK)
				return true;
			ESP_LOGE(TAG, "I2C read retry dev 0x%02X reg 0x%02X failed: %s",
					 addr, reg, esp_err_to_name(err2));
		}
		else
		{
			ESP_LOGE(TAG, "I2C bus reset failed");
		}
	}

	return false;
}

bool SensorI2CBus::write(uint8_t addr, uint8_t reg, const uint8_t *data,
						 size_t len, TickType_t timeout)
{
	if (!m_inited)
		return false;

	BusLock lock(*this, timeout);
	if (!lock.locked())
		return false;

	auto do_xfer = [&]() -> esp_err_t
	{
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		if (!cmd)
			return ESP_ERR_NO_MEM;

		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, reg, true);
		if (data && len)
		{
			i2c_master_write(cmd, const_cast<uint8_t *>(data), len, true);
		}
		i2c_master_stop(cmd);

		return cmd_begin(cmd, timeout);
	};

	esp_err_t err = do_xfer();
	if (err == ESP_OK)
		return true;

	ESP_LOGW(TAG, "I2C write dev 0x%02X reg 0x%02X failed: %s", addr, reg,
			 esp_err_to_name(err));
	if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL ||
		err == ESP_ERR_INVALID_STATE)
	{
		ESP_LOGW(TAG, "I2C bus reset (write error)");
		if (busRecover())
		{
			esp_err_t err2 = do_xfer();
			if (err2 == ESP_OK)
				return true;
			ESP_LOGE(TAG, "I2C write retry dev 0x%02X reg 0x%02X failed: %s",
					 addr, reg, esp_err_to_name(err2));
		}
		else
		{
			ESP_LOGE(TAG, "I2C bus reset failed");
		}
	}

	return false;
}

bool SensorI2CBus::probe(uint8_t addr, TickType_t timeout)
{
	if (!m_inited)
		return false;

	BusLock lock(*this, timeout);
	if (!lock.locked())
		return false;

	// Do a "dummy" write of zero bytes (just address + stop) to see if device
	// ACKs.
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	if (!cmd)
		return false;

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);

	esp_err_t err = cmd_begin(cmd, timeout);
	return (err == ESP_OK);
}

bool SensorI2CBus::busRecover()
{
	// Minimalistic recovery: delete and re-install the legacy driver.
	// We hold the mutex, so we're the only party touching the bus here.
	if (!m_inited)
		return false;

	i2c_driver_delete(m_port);
	vTaskDelay(pdMS_TO_TICKS(2)); // settle a bit

	i2c_config_t cfg = {};
	cfg.mode = I2C_MODE_MASTER;
	cfg.sda_io_num = m_sda;
	cfg.scl_io_num = m_scl;
	cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
	cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
	cfg.master.clk_speed = static_cast<uint32_t>(m_freq);

	if (i2c_param_config(m_port, &cfg) != ESP_OK)
		return false;
	if (i2c_driver_install(m_port, cfg.mode, 0, 0, 0) != ESP_OK)
		return false;

	ESP_LOGW(TAG, "I2C bus reset performed");
	return true;
}
