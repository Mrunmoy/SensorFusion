#pragma once

#include "IGpioInput.hpp"
#include "IGpioInterrupt.hpp"
#include "IGpioOutput.hpp"
#include "NrfSdk.hpp"

namespace sf {

class NrfGpioInput : public IGpioInput {
public:
    explicit NrfGpioInput(uint32_t pin) : pin_(pin) {}
    bool read(bool& level) override;

private:
    uint32_t pin_;
};

class NrfGpioOutput : public IGpioOutput {
public:
    explicit NrfGpioOutput(uint32_t pin) : pin_(pin) {}
    bool write(bool level) override;

private:
    uint32_t pin_;
};

class NrfGpioInterrupt : public IGpioInterrupt {
public:
    explicit NrfGpioInterrupt(uint32_t pin) : pin_(pin) {}

    bool enable(GpioEdge edge, Callback cb, void* context) override;
    bool disable() override;

    // Call from GPIOTE ISR path when pin changes.
    void handleIrq(uint32_t pin);

private:
    uint32_t pin_;
    Callback cb_ = nullptr;
    void* ctx_ = nullptr;
};

} // namespace sf
