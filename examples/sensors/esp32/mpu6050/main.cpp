#include "EspDelay.hpp"
#include "EspI2CBus.hpp"
#include "MPU6050.hpp"

extern "C" void app_main() {
    sf::EspI2CBus i2c(I2C_NUM_0);
    sf::EspDelay delay;
    sf::MPU6050 imu(i2c, delay);
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
