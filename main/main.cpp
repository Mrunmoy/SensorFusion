#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// keep your network/infra
#include "WifiManager.hpp"
#include "WebSocketServer.hpp"
#include "Heartbeat.hpp"

// ECG-only task
#include "Tasks.hpp"

extern "C" void app_main()
{
    // Bring up Wi-Fi + HTTP/WS + heartbeat like before
    // ---- Wi-Fi ----
	WifiManager::startStation();
	esp_netif_ip_info_t ip_info;
	esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
	while (true)
	{
		if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
			ip_info.ip.addr != 0)
			break;
		vTaskDelay(pdMS_TO_TICKS(100));
	}
    WebSocketServer::start();
    Heartbeat::start();

    // Start ECG sampler @ 250 Hz (no fusion/IMU/MAG/BARO)
    static EcgCtx ecgCtx{ .hz = 250.0f };
    xTaskCreatePinnedToCore(
        EcgTask, "ECG", 4096, &ecgCtx,
        10, nullptr, tskNO_AFFINITY);

    // keep main alive
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
