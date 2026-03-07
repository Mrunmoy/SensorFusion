#include "NrfGpio.hpp"

namespace sf {

bool NrfGpioInput::read(bool& level) {
    level = nrf_gpio_pin_read(pin_) != 0;
    return true;
}

bool NrfGpioOutput::write(bool level) {
    if (level) nrf_gpio_pin_set(pin_);
    else nrf_gpio_pin_clear(pin_);
    return true;
}

bool NrfGpioInterrupt::enable(GpioEdge edge, Callback cb, void* context) {
    (void)edge; // Edge selection is configured in GPIOTE init.
    cb_ = cb;
    ctx_ = context;
    return cb_ != nullptr;
}

bool NrfGpioInterrupt::disable() {
    cb_ = nullptr;
    ctx_ = nullptr;
    return true;
}

void NrfGpioInterrupt::handleIrq(uint32_t pin) {
    if (pin == pin_ && cb_) {
        cb_(ctx_);
    }
}

} // namespace sf
