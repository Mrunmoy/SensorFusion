#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"
#include "QMC5883L.hpp"

extern const nrfx_twim_t g_twim;

int main() {
    sf::NrfTwimBus i2c(g_twim);
    sf::NrfDelay delay;
    sf::QMC5883L mag(i2c, delay);
    if (!mag.init()) return 1;

    while (true) {
        sf::MagData m{};
        mag.readMag(m);
        delay.delayMs(20);
    }
}
