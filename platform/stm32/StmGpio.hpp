#pragma once

#include "IGpioInput.hpp"
#include "IGpioInterrupt.hpp"
#include "IGpioOutput.hpp"
#include "StmHal.hpp"

namespace sf {

class StmGpioInput : public IGpioInput {
public:
    StmGpioInput(GPIO_TypeDef* port, uint16_t pin) : port_(port), pin_(pin) {}
    bool read(bool& level) override;

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

class StmGpioOutput : public IGpioOutput {
public:
    StmGpioOutput(GPIO_TypeDef* port, uint16_t pin) : port_(port), pin_(pin) {}
    bool write(bool level) override;

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

class StmGpioInterrupt : public IGpioInterrupt {
public:
    explicit StmGpioInterrupt(uint16_t pinMask) : pinMask_(pinMask) {}

    bool enable(GpioEdge edge, Callback cb, void* context) override;
    bool disable() override;

    // Call this from HAL_GPIO_EXTI_Callback(GPIO_Pin)
    void handleIrq(uint16_t pinMask);

private:
    uint16_t pinMask_;
    Callback cb_ = nullptr;
    void* ctx_ = nullptr;
};

} // namespace sf
