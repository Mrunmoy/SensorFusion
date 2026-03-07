#include "AD8232.hpp"
#include "StmAdcChannel.hpp"
#include "StmDelay.hpp"

extern ADC_HandleTypeDef hadc1;

int main() {
    sf::StmAdcChannel adc(&hadc1);
    sf::StmDelay delay;
    sf::AD8232 ecg(adc, delay);

    while (true) {
        sf::ECGSample sample{};
        ecg.sample(sample);
        delay.delayMs(4);
    }
}
