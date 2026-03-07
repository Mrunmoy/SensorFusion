#pragma once

#include "II2CBus.hpp"
#include "StmHal.hpp"

namespace sf {

class StmI2CBus : public II2CBus {
public:
    explicit StmI2CBus(I2C_HandleTypeDef* handle, uint32_t timeoutMs = 100);

    bool readRegister(uint8_t devAddr, uint8_t reg, uint8_t* buf, size_t len) override;
    bool writeRegister(uint8_t devAddr, uint8_t reg, const uint8_t* data, size_t len) override;
    bool probe(uint8_t devAddr) override;
    bool rawWrite(uint8_t devAddr, const uint8_t* data, size_t len) override;
    bool rawRead(uint8_t devAddr, uint8_t* buf, size_t len) override;

private:
    I2C_HandleTypeDef* h_;
    uint32_t timeoutMs_;
};

} // namespace sf
