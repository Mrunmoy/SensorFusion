#include "ADXL345.hpp"
#include "StmDelay.hpp"
#include "StmI2CBus.hpp"

extern I2C_HandleTypeDef hi2c1;

int main() {
    sf::StmI2CBus i2c(&hi2c1);
    sf::StmDelay delay;
    sf::ADXL345 accel(i2c);
    if (!accel.init()) return 1;

    while (true) {
        sf::AccelData a{};
        accel.readAccel(a);
        delay.delayMs(10);
    }
}
