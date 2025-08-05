#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "WifiManager.hpp"
#include "Heartbeat.hpp"

extern "C" void app_main()
{
    // Don't forget to set your Wi-Fi credentials in sdkconfig
    // CONFIG_ESP_WIFI_SSID="YourSSID"
    // CONFIG_ESP_WIFI_PASSWORD="YourPassword"
    ESP_LOGI("APP", "Starting Sensor Fusion App...");
    WifiManager::startSoftAP();
    Heartbeat::start();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
