#pragma once

#include "IGpioOutput.hpp"
#include <gmock/gmock.h>

namespace sf { namespace test {

class MockGpioOutput : public IGpioOutput {
public:
    MOCK_METHOD(bool, write, (bool level), (override));
};

}} // namespace sf::test
