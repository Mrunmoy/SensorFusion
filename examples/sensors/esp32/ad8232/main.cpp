#include "AD8232.hpp"
#include "EspAdcChannel.hpp"
#include "EspDelay.hpp"

extern adc_oneshot_unit_handle_t g_adc_unit;

extern "C" void app_main() {
    sf::EspAdcChannel adc(g_adc_unit, ADC_CHANNEL_6);
    sf::EspDelay delay;
    sf::AD8232 ecg(adc, delay);

    while (true) {
        sf::ECGSample sample{};
        ecg.sample(sample);
        delay.delayMs(4);
    }
}
