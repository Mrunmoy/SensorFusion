#include "EspDelay.hpp"
#include "EspI2CBus.hpp"
#include "SHT40.hpp"

extern "C" void app_main() {
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;
    sf::SHT40 sht(i2c, delay);
    if (!sht.init()) return;

    while (true) {
        float t = 0.0f;
        float rh = 0.0f;
        sht.measure(t, rh);
        delay.delayMs(500);
    }
}
