#include "BMP180.hpp"
#include "EspDelay.hpp"
#include "EspI2CBus.hpp"

extern "C" void app_main() {
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;
    sf::BMP180 baro(i2c, delay);
    if (!baro.init()) return;

    while (true) {
        float hPa = 0.0f;
        float t = 0.0f;
        baro.readPressureHPa(hPa);
        baro.readTemperature(t);
        delay.delayMs(200);
    }
}
