#include "StmI2CBus.hpp"

namespace sf {

StmI2CBus::StmI2CBus(I2C_HandleTypeDef* handle, uint32_t timeoutMs)
    : h_(handle), timeoutMs_(timeoutMs)
{}

bool StmI2CBus::readRegister(uint8_t devAddr, uint8_t reg, uint8_t* buf, size_t len) {
    if (!h_ || !buf || len == 0) return false;
    return HAL_I2C_Mem_Read(h_, static_cast<uint16_t>(devAddr << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, buf,
                            static_cast<uint16_t>(len), timeoutMs_) == HAL_OK;
}

bool StmI2CBus::writeRegister(uint8_t devAddr, uint8_t reg, const uint8_t* data, size_t len) {
    if (!h_) return false;
    return HAL_I2C_Mem_Write(h_, static_cast<uint16_t>(devAddr << 1), reg,
                             I2C_MEMADD_SIZE_8BIT,
                             const_cast<uint8_t*>(data),
                             static_cast<uint16_t>(len), timeoutMs_) == HAL_OK;
}

bool StmI2CBus::probe(uint8_t devAddr) {
    if (!h_) return false;
    return HAL_I2C_IsDeviceReady(h_, static_cast<uint16_t>(devAddr << 1), 2, timeoutMs_) == HAL_OK;
}

bool StmI2CBus::rawWrite(uint8_t devAddr, const uint8_t* data, size_t len) {
    if (!h_ || !data || len == 0) return false;
    return HAL_I2C_Master_Transmit(h_, static_cast<uint16_t>(devAddr << 1),
                                   const_cast<uint8_t*>(data),
                                   static_cast<uint16_t>(len), timeoutMs_) == HAL_OK;
}

bool StmI2CBus::rawRead(uint8_t devAddr, uint8_t* buf, size_t len) {
    if (!h_ || !buf || len == 0) return false;
    return HAL_I2C_Master_Receive(h_, static_cast<uint16_t>(devAddr << 1), buf,
                                  static_cast<uint16_t>(len), timeoutMs_) == HAL_OK;
}

} // namespace sf
