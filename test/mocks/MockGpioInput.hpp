#pragma once

#include "IGpioInput.hpp"
#include <gmock/gmock.h>

namespace sf { namespace test {

class MockGpioInput : public IGpioInput {
public:
    MOCK_METHOD(bool, read, (bool& level), (override));
};

}} // namespace sf::test
