#include "MPU6050.hpp"
#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"

extern const nrfx_twim_t g_twim;

int main() {
    sf::NrfTwimBus i2c(g_twim);
    sf::NrfDelay delay;
    sf::MPU6050 imu(i2c, delay);
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
