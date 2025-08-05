#include "WifiManager.hpp"
#include "esp_log.h"

extern "C" void app_main()
{
    // CONFIG_ESP_WIFI_SSID="TheSamStone"
    // CONFIG_ESP_WIFI_PASSWORD="esp-32-s3"
    ESP_LOGI("APP", "Starting Sensor Fusion App...");
    WifiManager::startSoftAP();
}
