#include "BMM350.hpp"
#include "FrameCodec.hpp"
#include "LPS22DF.hpp"
#include "LSM6DSO.hpp"
#include "MocapBleTransport.hpp"
#include "MocapNodePipeline.hpp"
#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"

extern const nrfx_twim_t g_twim0; // Dedicated IMU bus
extern const nrfx_twim_t g_twim1; // Shared MAG + BARO bus

namespace {
constexpr uint8_t kNodeId = 1;

enum class PowerMode {
    PERFORMANCE,
    BATTERY,
};

constexpr PowerMode kPowerMode = PowerMode::PERFORMANCE;

extern "C" bool __attribute__((weak)) sf_mocap_ble_notify(const uint8_t* data, size_t len) {
    (void)data;
    (void)len;
    // Provide this symbol from your board BLE module.
    return false;
}

bool bleNotifyAdapter(const uint8_t* data, size_t len, void* context) {
    (void)context;
    return sf_mocap_ble_notify(data, len);
}
} // namespace

int main() {
    sf::NrfTwimBus imuBus(g_twim0);
    sf::NrfTwimBus envBus(g_twim1);
    sf::NrfDelay delay;

    sf::LSM6DSOConfig imuCfg{};
    sf::BMM350Config magCfg{};
    sf::LPS22DFConfig baroCfg{};
    sf::MocapNodePipeline::Config nodeCfg{};
    uint32_t outputPeriodUs = 20000; // 50 Hz

    if (kPowerMode == PowerMode::PERFORMANCE) {
        imuCfg.accelOdr = sf::LsmOdr::HZ_208;
        imuCfg.gyroOdr = sf::LsmOdr::HZ_208;
        magCfg.odr = sf::Bmm350Odr::HZ_100;
        baroCfg.odr = sf::LpsOdr::HZ_200;
        nodeCfg.dtSeconds = 1.0f / 50.0f;
        outputPeriodUs = 20000;
    } else {
        imuCfg.accelOdr = sf::LsmOdr::HZ_104;
        imuCfg.gyroOdr = sf::LsmOdr::HZ_104;
        magCfg.odr = sf::Bmm350Odr::HZ_50;
        baroCfg.odr = sf::LpsOdr::HZ_25;
        nodeCfg.dtSeconds = 1.0f / 40.0f;
        outputPeriodUs = 25000;
    }

    sf::LSM6DSO imu(imuBus, delay, imuCfg);

    sf::BMM350 mag(envBus, delay, magCfg);

    sf::LPS22DF baro(envBus, delay, baroCfg);

    if (!imu.init() || !mag.init() || !baro.init()) {
        while (true) delay.delayMs(1000);
    }

    nodeCfg.preferMag = true;
    sf::MocapNodePipeline pipeline(imu, &mag, &baro, nodeCfg);
    sf::MocapBleTransport::Config bleCfg{};
    bleCfg.attMtu = 185;
    bleCfg.maxRetries = 2;
    sf::MocapBleTransport bleTx(&bleNotifyAdapter, nullptr, bleCfg);

    uint64_t nextTickUs = delay.getTimestampUs();
    while (true) {
        const uint64_t nowUs = delay.getTimestampUs();
        if (nowUs < nextTickUs) {
            delay.delayMs(1);
            continue;
        }
        nextTickUs = nowUs + outputPeriodUs;

        sf::MocapNodeSample sample{};
        if (!pipeline.step(sample)) continue;

        (void)bleTx.sendQuaternion(kNodeId, nowUs, sample.orientation);
    }
}
