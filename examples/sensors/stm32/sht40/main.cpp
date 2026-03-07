#include "SHT40.hpp"
#include "StmDelay.hpp"
#include "StmI2CBus.hpp"

extern I2C_HandleTypeDef hi2c1;

int main() {
    sf::StmI2CBus i2c(&hi2c1);
    sf::StmDelay delay;
    sf::SHT40 sht(i2c, delay);
    if (!sht.init()) return 1;

    while (true) {
        float t = 0.0f;
        float rh = 0.0f;
        sht.measure(t, rh);
        delay.delayMs(500);
    }
}
