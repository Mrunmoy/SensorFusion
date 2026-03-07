#include "ADXL345.hpp"
#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"

extern const nrfx_twim_t g_twim;

int main() {
    sf::NrfTwimBus i2c(g_twim);
    sf::NrfDelay delay;
    sf::ADXL345 accel(i2c);
    if (!accel.init()) return 1;

    while (true) {
        sf::AccelData a{};
        accel.readAccel(a);
        delay.delayMs(10);
    }
}
