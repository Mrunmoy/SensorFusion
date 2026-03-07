#include "EspDelay.hpp"
#include "EspI2CBus.hpp"
#include "SGP40.hpp"

extern "C" void app_main() {
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;
    sf::SGP40 sgp(i2c, delay);
    if (!sgp.init()) return;

    while (true) {
        uint16_t voc = 0;
        sgp.readVocRaw(voc);
        delay.delayMs(1000);
    }
}
