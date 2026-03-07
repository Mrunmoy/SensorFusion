#include "EspSpiBus.hpp"
#include <cstring>

namespace sf {

EspSpiBus::EspSpiBus(spi_device_handle_t handle, uint8_t readMask)
    : handle_(handle), readMask_(readMask)
{}

bool EspSpiBus::readRegister(uint8_t reg, uint8_t* buf, size_t len) {
    if (!buf || len == 0 || !handle_) return false;

    uint8_t tx[1 + 256] = {};
    if (len > 256) return false;
    tx[0] = static_cast<uint8_t>(reg | readMask_);

    uint8_t rx[1 + 256] = {};
    spi_transaction_t t = {};
    t.length = static_cast<uint32_t>((1 + len) * 8);
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    if (spi_device_polling_transmit(handle_, &t) != ESP_OK) return false;
    std::memcpy(buf, rx + 1, len);
    return true;
}

bool EspSpiBus::writeRegister(uint8_t reg, const uint8_t* data, size_t len) {
    if (!handle_) return false;
    if (len > 256) return false;

    uint8_t tx[1 + 256] = {};
    tx[0] = reg;
    if (data && len > 0) {
        std::memcpy(tx + 1, data, len);
    }

    spi_transaction_t t = {};
    t.length = static_cast<uint32_t>((1 + len) * 8);
    t.tx_buffer = tx;
    return spi_device_polling_transmit(handle_, &t) == ESP_OK;
}

} // namespace sf
