#include "NrfTwimBus.hpp"

namespace sf {

bool NrfTwimBus::readRegister(uint8_t devAddr, uint8_t reg, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    if (nrfx_twim_tx(&twim_, devAddr, &reg, 1, true) != NRFX_SUCCESS) return false;
    return nrfx_twim_rx(&twim_, devAddr, buf, len) == NRFX_SUCCESS;
}

bool NrfTwimBus::writeRegister(uint8_t devAddr, uint8_t reg, const uint8_t* data, size_t len) {
    uint8_t tmp[1 + 32] = {};
    if (len <= 32) {
        tmp[0] = reg;
        for (size_t i = 0; i < len; ++i) tmp[1 + i] = data[i];
        return nrfx_twim_tx(&twim_, devAddr, tmp, 1 + len, false) == NRFX_SUCCESS;
    }
    if (nrfx_twim_tx(&twim_, devAddr, &reg, 1, true) != NRFX_SUCCESS) return false;
    return nrfx_twim_tx(&twim_, devAddr, data, len, false) == NRFX_SUCCESS;
}

bool NrfTwimBus::probe(uint8_t devAddr) {
    uint8_t b = 0;
    return nrfx_twim_rx(&twim_, devAddr, &b, 1) == NRFX_SUCCESS;
}

bool NrfTwimBus::rawWrite(uint8_t devAddr, const uint8_t* data, size_t len) {
    if (!data || len == 0) return false;
    return nrfx_twim_tx(&twim_, devAddr, data, len, false) == NRFX_SUCCESS;
}

bool NrfTwimBus::rawRead(uint8_t devAddr, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    return nrfx_twim_rx(&twim_, devAddr, buf, len) == NRFX_SUCCESS;
}

} // namespace sf
