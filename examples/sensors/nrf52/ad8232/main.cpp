#include "AD8232.hpp"
#include "NrfDelay.hpp"
#include "NrfSaadcChannel.hpp"

int main() {
    sf::NrfSaadcChannel adc(0);
    sf::NrfDelay delay;
    sf::AD8232 ecg(adc, delay);

    while (true) {
        sf::ECGSample sample{};
        ecg.sample(sample);
        delay.delayMs(4);
    }
}
