#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"
#include "SGP40.hpp"

extern const nrfx_twim_t g_twim;

int main() {
    sf::NrfTwimBus i2c(g_twim);
    sf::NrfDelay delay;
    sf::SGP40 sgp(i2c, delay);
    if (!sgp.init()) return 1;

    while (true) {
        uint16_t voc = 0;
        sgp.readVocRaw(voc);
        delay.delayMs(1000);
    }
}
