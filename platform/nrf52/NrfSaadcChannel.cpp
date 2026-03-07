#include "NrfSaadcChannel.hpp"

namespace sf {

bool NrfSaadcChannel::readRaw(int32_t& out) {
    nrf_saadc_value_t sample = 0;
    if (nrfx_saadc_sample_convert(channel_, &sample) != NRFX_SUCCESS) return false;
    out = static_cast<int32_t>(sample);
    return true;
}

bool NrfSaadcChannel::readMillivolts(int32_t& out) {
    int32_t raw = 0;
    if (!readRaw(raw)) return false;

    // Typical nRF52 SAADC single-ended, gain 1/6, reference 0.6 V => 3.6 V full-scale.
    out = (raw * 3600) / 1023;
    return true;
}

} // namespace sf
