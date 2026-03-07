#include "SGP40.hpp"
#include "StmDelay.hpp"
#include "StmI2CBus.hpp"

extern I2C_HandleTypeDef hi2c1;

int main() {
    sf::StmI2CBus i2c(&hi2c1);
    sf::StmDelay delay;
    sf::SGP40 sgp(i2c, delay);
    if (!sgp.init()) return 1;

    while (true) {
        uint16_t voc = 0;
        sgp.readVocRaw(voc);
        delay.delayMs(1000);
    }
}
