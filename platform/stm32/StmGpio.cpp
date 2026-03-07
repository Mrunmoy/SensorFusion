#include "StmGpio.hpp"

namespace sf {

bool StmGpioInput::read(bool& level) {
    if (!port_) return false;
    level = HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
    return true;
}

bool StmGpioOutput::write(bool level) {
    if (!port_) return false;
    HAL_GPIO_WritePin(port_, pin_, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

bool StmGpioInterrupt::enable(GpioEdge edge, Callback cb, void* context) {
    (void)edge; // Edge is configured by CubeMX/LL init for the pin.
    cb_ = cb;
    ctx_ = context;
    return cb_ != nullptr;
}

bool StmGpioInterrupt::disable() {
    cb_ = nullptr;
    ctx_ = nullptr;
    return true;
}

void StmGpioInterrupt::handleIrq(uint16_t pinMask) {
    if (pinMask == pinMask_ && cb_) {
        cb_(ctx_);
    }
}

} // namespace sf
