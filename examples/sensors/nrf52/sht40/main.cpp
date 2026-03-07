#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"
#include "SHT40.hpp"

extern const nrfx_twim_t g_twim;

int main() {
    sf::NrfTwimBus i2c(g_twim);
    sf::NrfDelay delay;
    sf::SHT40 sht(i2c, delay);
    if (!sht.init()) return 1;

    while (true) {
        float t = 0.0f;
        float rh = 0.0f;
        sht.measure(t, rh);
        delay.delayMs(500);
    }
}
