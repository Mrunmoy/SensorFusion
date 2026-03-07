#include "BMP180.hpp"
#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"

extern const nrfx_twim_t g_twim;

int main() {
    sf::NrfTwimBus i2c(g_twim);
    sf::NrfDelay delay;
    sf::BMP180 baro(i2c, delay);
    if (!baro.init()) return 1;

    while (true) {
        float hPa = 0.0f;
        float t = 0.0f;
        baro.readPressureHPa(hPa);
        baro.readTemperature(t);
        delay.delayMs(200);
    }
}
