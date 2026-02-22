#pragma once

#include "IDelayProvider.hpp"
#include <gmock/gmock.h>

namespace sf { namespace test {

class MockDelayProvider : public IDelayProvider {
public:
    MOCK_METHOD(void, delayMs, (uint32_t ms), (override));
    MOCK_METHOD(void, delayUs, (uint32_t us), (override));
    MOCK_METHOD(uint64_t, getTimestampUs, (), (override));
};

}} // namespace sf::test
