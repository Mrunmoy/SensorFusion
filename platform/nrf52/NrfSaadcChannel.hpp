#pragma once

#include "IAdcChannel.hpp"
#include "NrfSdk.hpp"

namespace sf {

class NrfSaadcChannel : public IAdcChannel {
public:
    explicit NrfSaadcChannel(uint8_t channelIndex) : channel_(channelIndex) {}

    bool readRaw(int32_t& out) override;
    bool readMillivolts(int32_t& out) override;

private:
    uint8_t channel_;
};

} // namespace sf
