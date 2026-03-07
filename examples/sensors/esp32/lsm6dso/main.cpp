#include "EspDelay.hpp"
#include "EspI2CBus.hpp"
#include "LSM6DSO.hpp"

extern "C" void app_main() {
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;
    sf::LSM6DSO imu(i2c, delay);
    if (!imu.init()) return;

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
