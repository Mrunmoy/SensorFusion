#pragma once

#include "IGpioInterrupt.hpp"
#include <gmock/gmock.h>

namespace sf { namespace test {

class MockGpioInterrupt : public IGpioInterrupt {
public:
    MOCK_METHOD(bool, enable, (GpioEdge edge, Callback cb, void* context), (override));
    MOCK_METHOD(bool, disable, (), (override));

    // Test helper: capture the callback so tests can simulate an interrupt.
    void captureCallback() {
        ON_CALL(*this, enable(::testing::_, ::testing::_, ::testing::_))
            .WillByDefault([this](GpioEdge, Callback cb, void* ctx) {
                capturedCb_ = cb;
                capturedCtx_ = ctx;
                return true;
            });
    }

    // Simulate the hardware interrupt firing.
    void fire() {
        if (capturedCb_) capturedCb_(capturedCtx_);
    }

private:
    Callback capturedCb_ = nullptr;
    void* capturedCtx_ = nullptr;
};

}} // namespace sf::test
