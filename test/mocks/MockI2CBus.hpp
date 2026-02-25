#pragma once

#include "II2CBus.hpp"
#include <gmock/gmock.h>

namespace sf { namespace test {

class MockI2CBus : public II2CBus {
public:
    MOCK_METHOD(bool, readRegister,
                (uint8_t devAddr, uint8_t reg, uint8_t* buf, size_t len),
                (override));
    MOCK_METHOD(bool, writeRegister,
                (uint8_t devAddr, uint8_t reg, const uint8_t* data, size_t len),
                (override));
    MOCK_METHOD(bool, probe, (uint8_t devAddr), (override));
    MOCK_METHOD(bool, rawWrite,
                (uint8_t devAddr, const uint8_t* data, size_t len),
                (override));
    MOCK_METHOD(bool, rawRead,
                (uint8_t devAddr, uint8_t* buf, size_t len),
                (override));
};

}} // namespace sf::test
