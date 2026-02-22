#pragma once

#include "IAdcChannel.hpp"
#include <gmock/gmock.h>

namespace sf { namespace test {

class MockAdcChannel : public IAdcChannel {
public:
    MOCK_METHOD(bool, readRaw, (int32_t& out), (override));
    MOCK_METHOD(bool, readMillivolts, (int32_t& out), (override));
};

}} // namespace sf::test
