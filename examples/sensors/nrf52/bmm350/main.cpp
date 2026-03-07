#include "BMM350.hpp"
#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"

extern const nrfx_twim_t g_twim;

int main() {
    sf::NrfTwimBus i2c(g_twim);
    sf::NrfDelay delay;
    sf::BMM350 mag(i2c, delay);
    if (!mag.init()) return 1;

    while (true) {
        sf::MagData m{};
        float t = 0.0f;
        mag.readMag(m);
        mag.readTemperature(t);
        delay.delayMs(20);
    }
}
