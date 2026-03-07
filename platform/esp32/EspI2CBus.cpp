#include "EspI2CBus.hpp"
#include "freertos/FreeRTOS.h"

namespace sf {

EspI2CBus::EspI2CBus(i2c_port_t port, uint32_t timeoutMs)
    : port_(port), timeoutTicks_(pdMS_TO_TICKS(timeoutMs))
{}

bool EspI2CBus::readRegister(uint8_t devAddr, uint8_t reg, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    return i2c_master_write_read_device(port_, devAddr, &reg, 1, buf, len, timeoutTicks_) == ESP_OK;
}

bool EspI2CBus::writeRegister(uint8_t devAddr, uint8_t reg, const uint8_t* data, size_t len) {
    uint8_t scratch[1 + 32];
    if (len <= 32) {
        scratch[0] = reg;
        if (data && len > 0) {
            for (size_t i = 0; i < len; ++i) scratch[1 + i] = data[i];
        }
        return i2c_master_write_to_device(port_, devAddr, scratch, 1 + len, timeoutTicks_) == ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return false;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, static_cast<uint8_t>((devAddr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, reg, true);
    if (len > 0 && data) {
        i2c_master_write(cmd, const_cast<uint8_t*>(data), len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t rc = i2c_master_cmd_begin(port_, cmd, timeoutTicks_);
    i2c_cmd_link_delete(cmd);
    return rc == ESP_OK;
}

bool EspI2CBus::probe(uint8_t devAddr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return false;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, static_cast<uint8_t>((devAddr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_stop(cmd);
    esp_err_t rc = i2c_master_cmd_begin(port_, cmd, timeoutTicks_);
    i2c_cmd_link_delete(cmd);
    return rc == ESP_OK;
}

bool EspI2CBus::rawWrite(uint8_t devAddr, const uint8_t* data, size_t len) {
    if (!data || len == 0) return false;
    return i2c_master_write_to_device(port_, devAddr, data, len, timeoutTicks_) == ESP_OK;
}

bool EspI2CBus::rawRead(uint8_t devAddr, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    return i2c_master_read_from_device(port_, devAddr, buf, len, timeoutTicks_) == ESP_OK;
}

} // namespace sf
