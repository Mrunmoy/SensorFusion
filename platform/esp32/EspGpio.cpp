#include "EspGpio.hpp"

namespace sf {

bool EspGpioInput::read(bool& level) {
    int v = gpio_get_level(pin_);
    level = (v != 0);
    return true;
}

bool EspGpioOutput::write(bool level) {
    return gpio_set_level(pin_, level ? 1 : 0) == ESP_OK;
}

bool EspGpioInterrupt::enable(GpioEdge edge, Callback cb, void* context) {
    gpio_int_type_t type = GPIO_INTR_DISABLE;
    if (edge == GpioEdge::RISING) type = GPIO_INTR_POSEDGE;
    else if (edge == GpioEdge::FALLING) type = GPIO_INTR_NEGEDGE;
    else type = GPIO_INTR_ANYEDGE;

    cb_ = cb;
    ctx_ = context;

    if (gpio_set_intr_type(pin_, type) != ESP_OK) return false;
    const esp_err_t isrSvcRc = gpio_install_isr_service(0);
    if (isrSvcRc != ESP_OK && isrSvcRc != ESP_ERR_INVALID_STATE) return false;
    if (gpio_isr_handler_add(pin_, &EspGpioInterrupt::isrTrampoline, this) != ESP_OK) return false;
    return true;
}

bool EspGpioInterrupt::disable() {
    cb_ = nullptr;
    ctx_ = nullptr;
    return gpio_isr_handler_remove(pin_) == ESP_OK;
}

void IRAM_ATTR EspGpioInterrupt::isrTrampoline(void* arg) {
    auto* self = static_cast<EspGpioInterrupt*>(arg);
    if (self && self->cb_) self->cb_(self->ctx_);
}

} // namespace sf
