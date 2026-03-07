#include "BMM350.hpp"
#include "FrameCodec.hpp"
#include "LPS22DF.hpp"
#include "LSM6DSO.hpp"
#include "MocapNodePipeline.hpp"
#include "NrfDelay.hpp"
#include "NrfTwimBus.hpp"

extern const nrfx_twim_t g_twim0; // Dedicated IMU bus
extern const nrfx_twim_t g_twim1; // Shared MAG + BARO bus

namespace {
constexpr uint8_t kNodeId = 1;
constexpr uint32_t kOutputPeriodUs = 20000; // 50 Hz

bool bleSend(const uint8_t* data, size_t len) {
    (void)data;
    (void)len;
    // Integrate with your BLE transport (NUS/custom GATT) here.
    return true;
}
} // namespace

int main() {
    sf::NrfTwimBus imuBus(g_twim0);
    sf::NrfTwimBus envBus(g_twim1);
    sf::NrfDelay delay;

    sf::LSM6DSOConfig imuCfg{};
    imuCfg.accelOdr = sf::LsmOdr::HZ_208;
    imuCfg.gyroOdr = sf::LsmOdr::HZ_208;
    sf::LSM6DSO imu(imuBus, delay, imuCfg);

    sf::BMM350Config magCfg{};
    magCfg.odr = sf::Bmm350Odr::HZ_100;
    sf::BMM350 mag(envBus, delay, magCfg);

    sf::LPS22DFConfig baroCfg{};
    baroCfg.odr = sf::LpsOdr::HZ_200;
    sf::LPS22DF baro(envBus, delay, baroCfg);

    if (!imu.init() || !mag.init() || !baro.init()) {
        while (true) delay.delayMs(1000);
    }

    sf::MocapNodePipeline::Config nodeCfg{};
    nodeCfg.dtSeconds = 1.0f / 50.0f;
    nodeCfg.preferMag = true;
    sf::MocapNodePipeline pipeline(imu, &mag, &baro, nodeCfg);

    uint8_t frame[sf::FrameCodec::MAX_FRAME_SIZE] = {};
    uint64_t nextTickUs = delay.getTimestampUs();
    while (true) {
        const uint64_t nowUs = delay.getTimestampUs();
        if (nowUs < nextTickUs) {
            delay.delayMs(1);
            continue;
        }
        nextTickUs = nowUs + kOutputPeriodUs;

        sf::MocapNodeSample sample{};
        if (!pipeline.step(sample)) continue;

        const size_t frameLen = sf::FrameCodec::encodeQuaternion(
            kNodeId, nowUs, sample.orientation, frame, sizeof(frame));
        if (frameLen > 0) bleSend(frame, frameLen);
    }
}
