#include "BMM350.hpp"
#include "EspDelay.hpp"
#include "EspI2CBus.hpp"

extern "C" void app_main() {
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;
    sf::BMM350 mag(i2c, delay);
    if (!mag.init()) return;

    while (true) {
        sf::MagData m{};
        float t = 0.0f;
        mag.readMag(m);
        mag.readTemperature(t);
        delay.delayMs(20);
    }
}
