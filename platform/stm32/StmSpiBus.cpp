#include "StmSpiBus.hpp"

namespace sf {

StmSpiBus::StmSpiBus(SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin,
                     uint8_t readMask, uint32_t timeoutMs)
    : hspi_(spi), csPort_(csPort), csPin_(csPin), readMask_(readMask), timeoutMs_(timeoutMs)
{}

void StmSpiBus::select() {
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);
}

void StmSpiBus::deselect() {
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
}

bool StmSpiBus::readRegister(uint8_t reg, uint8_t* buf, size_t len) {
    if (!hspi_ || !csPort_ || !buf || len == 0) return false;

    uint8_t txReg = static_cast<uint8_t>(reg | readMask_);
    select();
    bool ok = HAL_SPI_Transmit(hspi_, &txReg, 1, timeoutMs_) == HAL_OK &&
              HAL_SPI_Receive(hspi_, buf, static_cast<uint16_t>(len), timeoutMs_) == HAL_OK;
    deselect();
    return ok;
}

bool StmSpiBus::writeRegister(uint8_t reg, const uint8_t* data, size_t len) {
    if (!hspi_ || !csPort_) return false;

    select();
    bool ok = HAL_SPI_Transmit(hspi_, &reg, 1, timeoutMs_) == HAL_OK;
    if (ok && data && len > 0) {
        ok = HAL_SPI_Transmit(hspi_, const_cast<uint8_t*>(data), static_cast<uint16_t>(len), timeoutMs_) == HAL_OK;
    }
    deselect();
    return ok;
}

} // namespace sf
