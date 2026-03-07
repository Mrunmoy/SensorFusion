#include "BMM350.hpp"
#include "StmDelay.hpp"
#include "StmI2CBus.hpp"

extern I2C_HandleTypeDef hi2c1;

int main() {
    sf::StmI2CBus i2c(&hi2c1);
    sf::StmDelay delay;
    sf::BMM350 mag(i2c, delay);
    if (!mag.init()) return 1;

    while (true) {
        sf::MagData m{};
        float t = 0.0f;
        mag.readMag(m);
        mag.readTemperature(t);
        delay.delayMs(20);
    }
}
