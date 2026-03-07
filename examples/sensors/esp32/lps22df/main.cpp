#include "EspDelay.hpp"
#include "EspI2CBus.hpp"
#include "LPS22DF.hpp"

extern "C" void app_main() {
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;
    sf::LPS22DF baro(i2c, delay);
    if (!baro.init()) return;

    while (true) {
        float hPa = 0.0f;
        float t = 0.0f;
        baro.readPressure(hPa);
        baro.readTemperature(t);
        delay.delayMs(100);
    }
}
