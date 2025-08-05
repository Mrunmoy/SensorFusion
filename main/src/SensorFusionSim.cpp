#include "SensorFusionSim.hpp"
#include "WebSocketServer.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include <cstdio>

static const char* TAG = "SensorFusionSim";

void SensorFusionSim::start() {
    xTaskCreate([](void*) {
        ESP_LOGI(TAG, "Starting simulated data stream...");

        while (true) {
            static int counter = 0;
            char buffer[256];

            // Generate fake values
            int yaw   = (counter * 3) % 360;
            int pitch = (counter * 2) % 180;
            int roll  = (counter * 5) % 360;

            float temperature = 25.0f + (counter % 10); // 25–34°C
            float humidity = 50.0f + (counter % 20);    // 50–69%
            int air_quality = 50 + (counter % 500);     // 50–549

            snprintf(buffer, sizeof(buffer),
                R"({"imu":{"yaw":%d,"pitch":%d,"roll":%d},"environment":{"temperature":%.1f,"humidity":%.1f,"air_quality":%d}})",
                yaw, pitch, roll,
                temperature, humidity, air_quality
            );

            WebSocketServer::broadcast(buffer);
            counter++;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }, "FusionSimTask", 4096, nullptr, 5, nullptr);
}
