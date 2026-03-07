#include "NrfSpimBus.hpp"

namespace sf {

bool NrfSpimBus::readRegister(uint8_t reg, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;

    uint8_t tx[1 + 256] = {};
    uint8_t rx[1 + 256] = {};
    if (len > 256) return false;
    tx[0] = static_cast<uint8_t>(reg | readMask_);

    nrfx_spim_xfer_desc_t d = NRFX_SPIM_XFER_TRX(tx, 1 + len, rx, 1 + len);
    if (nrfx_spim_xfer(&spim_, &d, 0) != NRFX_SUCCESS) return false;
    for (size_t i = 0; i < len; ++i) buf[i] = rx[i + 1];
    return true;
}

bool NrfSpimBus::writeRegister(uint8_t reg, const uint8_t* data, size_t len) {
    uint8_t tx[1 + 256] = {};
    if (len > 256) return false;

    tx[0] = reg;
    for (size_t i = 0; i < len; ++i) tx[i + 1] = data[i];

    nrfx_spim_xfer_desc_t d = NRFX_SPIM_XFER_TX(tx, 1 + len);
    return nrfx_spim_xfer(&spim_, &d, 0) == NRFX_SUCCESS;
}

} // namespace sf
