#include "EspDelay.hpp"
#include "EspI2CBus.hpp"
#include "LIS3MDL.hpp"

extern "C" void app_main() {
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;
    sf::LIS3MDL mag(i2c, delay);
    if (!mag.init()) return;

    while (true) {
        sf::MagData m{};
        mag.readMag(m);
        delay.delayMs(20);
    }
}
