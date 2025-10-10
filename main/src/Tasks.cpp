#include "Tasks.hpp"

#include "Ad8232Driver.hpp"
#include "SensorAdcBus.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void EcgTask(void *pv)
{
    static const char *TAG = "EcgTask";
    auto *ctx = static_cast<EcgCtx *>(pv);
    if (!ctx) { vTaskDelete(nullptr); return; }

    // set up ADC bus + driver
    static SensorAdcBus adcBus;
    static Ad8232Driver ecg(adcBus);

    Ad8232Driver::Config cfg;
    cfg.unit      = ADC_UNIT_1;
    cfg.channel   = ADC_CHANNEL_0;                 // GPIO1 on ESP32-S3
    cfg.atten     = Ad8232Driver::Attenuation::DB_11;
    cfg.calibrate = true;

    if (!ecg.init(cfg))
    {
        ESP_LOGE(TAG, "ECG init failed");
        vTaskDelete(nullptr);
        return;
    }

    // simple fixed-rate loop (no pacer yet) ~250 Hz
    const TickType_t period = pdMS_TO_TICKS((TickType_t)(1000.0f / ctx->hz));
    TickType_t next = xTaskGetTickCount();

    int mv = -1;
    uint32_t printCount = 0;

    ESP_LOGI(TAG, "start @ %.1f Hz", ctx->hz);

    for (;;)
    {
        vTaskDelayUntil(&next, period);

        if (ecg.sample(mv))
        {
            // throttle logs (1 line per ~0.5s)
            if ((++printCount % (uint32_t)(ctx->hz / 2.0f)) == 0)
            {
                // note: mv == -1 means uncalibrated; still valid raw read occurred
                ESP_LOGI(TAG, "ECG: %d mV", mv);
            }
        }
        else
        {
            // occasional error message to avoid log spam
            static uint32_t errCount = 0;
            if ((++errCount % 100) == 0) ESP_LOGW(TAG, "read failed (x%u)", errCount);
        }
    }
}
