#include "LSM6DSO.hpp"
#include "StmDelay.hpp"
#include "StmI2CBus.hpp"

extern I2C_HandleTypeDef hi2c1;

int main() {
    sf::StmI2CBus i2c(&hi2c1);
    sf::StmDelay delay;
    sf::LSM6DSO imu(i2c, delay);
    if (!imu.init()) return 1;

    while (true) {
        sf::AccelData a{};
        sf::GyroData g{};
        float t = 0.0f;
        imu.readAccel(a);
        imu.readGyro(g);
        imu.readTemperature(t);
        delay.delayMs(10);
    }
}
