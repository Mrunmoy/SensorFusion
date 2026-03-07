#include "LIS3MDL.hpp"
#include "StmDelay.hpp"
#include "StmI2CBus.hpp"

extern I2C_HandleTypeDef hi2c1;

int main() {
    sf::StmI2CBus i2c(&hi2c1);
    sf::StmDelay delay;
    sf::LIS3MDL mag(i2c, delay);
    if (!mag.init()) return 1;

    while (true) {
        sf::MagData m{};
        mag.readMag(m);
        delay.delayMs(20);
    }
}
