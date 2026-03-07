#include "MahonyAHRS.hpp"
#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"
#include "QMC5883L.hpp"

extern const nrfx_twim_t g_twim;

int main() {
    sf::NrfTwimBus i2c(g_twim);
    sf::NrfDelay delay;
    sf::QMC5883L mag(i2c, delay);

    mag.init();

    sf::MahonyAHRS ahrs;
    while (true) {
        sf::MagData m{};
        if (mag.readMag(m)) {
            sf::AccelData a{};
            sf::GyroData g{};
            ahrs.update(a, g, m, 0.01f);
        }
        delay.delayMs(10);
    }
}
