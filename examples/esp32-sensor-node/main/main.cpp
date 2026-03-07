#include "EspDelay.hpp"
#include "EspI2CBus.hpp"
#include "MPU6050.hpp"

extern "C" void app_main() {
    // Board-specific I2C init belongs in app startup; this only wires abstractions.
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;

    sf::MPU6050 imu(i2c, delay);
    imu.init();

    while (true) {
        sf::AccelData a{};
        sf::GyroData g{};
        imu.readAccel(a);
        imu.readGyro(g);
        delay.delayMs(10);
    }
}
