#pragma once

#include "ISPIBus.hpp"
#include <gmock/gmock.h>

namespace sf { namespace test {

class MockSPIBus : public ISPIBus {
public:
    MOCK_METHOD(bool, readRegister,
                (uint8_t reg, uint8_t* buf, size_t len),
                (override));
    MOCK_METHOD(bool, writeRegister,
                (uint8_t reg, const uint8_t* data, size_t len),
                (override));
};

}} // namespace sf::test
