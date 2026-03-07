#pragma once

#include "ISPIBus.hpp"
#include "StmHal.hpp"

namespace sf {

class StmSpiBus : public ISPIBus {
public:
    StmSpiBus(SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin,
              uint8_t readMask = 0x80, uint32_t timeoutMs = 100);

    bool readRegister(uint8_t reg, uint8_t* buf, size_t len) override;
    bool writeRegister(uint8_t reg, const uint8_t* data, size_t len) override;

private:
    void select();
    void deselect();

    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* csPort_;
    uint16_t csPin_;
    uint8_t readMask_;
    uint32_t timeoutMs_;
};

} // namespace sf
