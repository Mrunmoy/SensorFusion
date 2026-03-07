#include "StmDelay.hpp"
#include "StmI2CBus.hpp"
#include "SHT40.hpp"

extern I2C_HandleTypeDef hi2c1;

int main() {
    sf::StmI2CBus i2c(&hi2c1);
    sf::StmDelay delay;

    sf::SHT40 sht(i2c, delay);
    sht.init();

    while (true) {
        float t = 0.0f;
        float h = 0.0f;
        sht.measure(t, h);
        delay.delayMs(200);
    }
}
