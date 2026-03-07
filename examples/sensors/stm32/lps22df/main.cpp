#include "LPS22DF.hpp"
#include "StmDelay.hpp"
#include "StmI2CBus.hpp"

extern I2C_HandleTypeDef hi2c1;

int main() {
    sf::StmI2CBus i2c(&hi2c1);
    sf::StmDelay delay;
    sf::LPS22DF baro(i2c, delay);
    if (!baro.init()) return 1;

    while (true) {
        float hPa = 0.0f;
        float t = 0.0f;
        baro.readPressure(hPa);
        baro.readTemperature(t);
        delay.delayMs(100);
    }
}
