#pragma once

#include "IGpioInput.hpp"
#include "IGpioInterrupt.hpp"
#include "IGpioOutput.hpp"
#include "esp_attr.h"
#include "driver/gpio.h"

namespace sf {

class EspGpioInput : public IGpioInput {
public:
    explicit EspGpioInput(gpio_num_t pin) : pin_(pin) {}
    bool read(bool& level) override;

private:
    gpio_num_t pin_;
};

class EspGpioOutput : public IGpioOutput {
public:
    explicit EspGpioOutput(gpio_num_t pin) : pin_(pin) {}
    bool write(bool level) override;

private:
    gpio_num_t pin_;
};

class EspGpioInterrupt : public IGpioInterrupt {
public:
    explicit EspGpioInterrupt(gpio_num_t pin) : pin_(pin) {}
    bool enable(GpioEdge edge, Callback cb, void* context) override;
    bool disable() override;

private:
    static void isrTrampoline(void* arg);

    gpio_num_t pin_;
    Callback cb_ = nullptr;
    void* ctx_ = nullptr;
};

} // namespace sf
