#include "ADXL345.hpp"
#include "EspDelay.hpp"
#include "EspI2CBus.hpp"

extern "C" void app_main() {
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;
    sf::ADXL345 accel(i2c);
    if (!accel.init()) return;

    while (true) {
        sf::AccelData a{};
        accel.readAccel(a);
        delay.delayMs(10);
    }
}
