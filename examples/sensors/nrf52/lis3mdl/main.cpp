#include "LIS3MDL.hpp"
#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"

extern const nrfx_twim_t g_twim;

int main() {
    sf::NrfTwimBus i2c(g_twim);
    sf::NrfDelay delay;
    sf::LIS3MDL mag(i2c, delay);
    if (!mag.init()) return 1;

    while (true) {
        sf::MagData m{};
        mag.readMag(m);
        delay.delayMs(20);
    }
}
