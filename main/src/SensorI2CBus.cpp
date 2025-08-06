#include "SensorI2CBus.hpp"
#include "esp_log.h"

static const char* TAG = "SensorI2CBus";

SensorI2CBus::SensorI2CBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freqHz)
    : m_port(port), m_sda(sda), m_scl(scl), m_freqHz(freqHz)
{}

bool SensorI2CBus::init()
{
    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = m_sda;
    config.scl_io_num = m_scl;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = m_freqHz;

    esp_err_t err = i2c_param_config(m_port, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(m_port, config.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    m_mutex = xSemaphoreCreateMutex();
    if (!m_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }

    ESP_LOGI(TAG, "I2C port %d ready. SDA=%d, SCL=%d", m_port, m_sda, m_scl);
    return true;
}

bool SensorI2CBus::write(uint8_t deviceAddr, uint8_t regAddr, const uint8_t* data, size_t len, TickType_t timeout)
{
    if (!xSemaphoreTake(m_mutex, timeout)) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(m_port, cmd, timeout);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(m_mutex);
    return err == ESP_OK;
}

bool SensorI2CBus::read(uint8_t deviceAddr, uint8_t regAddr, uint8_t* data, size_t len, TickType_t timeout)
{
    if (!xSemaphoreTake(m_mutex, timeout)) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);
    i2c_master_start(cmd); // repeated start
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(m_port, cmd, timeout);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(m_mutex);
    return err == ESP_OK;
}
