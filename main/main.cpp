#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "WifiManager.hpp"
#include "Heartbeat.hpp"
#include "WebSocketServer.hpp"
#include "SensorFusionSim.hpp"

extern "C" void app_main()
{
    // Don't forget to set your Wi-Fi credentials in sdkconfig
    // CONFIG_ESP_WIFI_SSID="YourSSID"
    // CONFIG_ESP_WIFI_PASSWORD="YourPassword"
    ESP_LOGI("APP", "Starting Sensor Fusion App...");

    // WifiManager::startSoftAP();
    WifiManager::startStation();
    // Optional: Wait for IP before starting web server
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    while (true)
    {
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0)
            break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    Heartbeat::start();
    WebSocketServer::start();
    SensorFusionSim::start();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
