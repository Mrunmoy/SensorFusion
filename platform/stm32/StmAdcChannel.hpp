#pragma once

#include "IAdcChannel.hpp"
#include "StmHal.hpp"

namespace sf {

class StmAdcChannel : public IAdcChannel {
public:
    explicit StmAdcChannel(ADC_HandleTypeDef* hadc) : hadc_(hadc) {}

    bool readRaw(int32_t& out) override;
    bool readMillivolts(int32_t& out) override;

private:
    ADC_HandleTypeDef* hadc_;
};

} // namespace sf
