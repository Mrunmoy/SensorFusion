#pragma once

#include "IAdcChannel.hpp"
#include "esp_adc/adc_oneshot.h"

namespace sf {

class EspAdcChannel : public IAdcChannel {
public:
    EspAdcChannel(adc_oneshot_unit_handle_t unit, adc_channel_t channel);

    bool readRaw(int32_t& out) override;
    bool readMillivolts(int32_t& out) override;

private:
    adc_oneshot_unit_handle_t unit_;
    adc_channel_t channel_;
};

} // namespace sf
