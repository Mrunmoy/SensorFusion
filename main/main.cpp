#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "WifiManager.hpp"
#include "Heartbeat.hpp"
#include "WebSocketServer.hpp"
#include "SensorFusionSim.hpp"

#include "SensorI2CBus.hpp"
#include "MPU6050Driver.hpp"
#include "ADXL345Driver.hpp"

static const char* TAG = "MainApp";

// Define sensor drivers and buses
SensorI2CBus* i2cBus0 = nullptr; // MPU6050
SensorI2CBus* i2cBus1 = nullptr; // ADXL345

MPU6050Driver* mpu = nullptr;
ADXL345Driver* adxl = nullptr;

constexpr uint8_t MPU6050_ADDR = 0x68;  // Common default I2C address
constexpr uint8_t ADXL345_ADDR = 0x53;  // Common default I2C address


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

    // MPU6050: I2C_NUM_0 (GPIO21, GPIO22)
    i2cBus0 = new SensorI2CBus(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
    if (!i2cBus0->init()) {
        ESP_LOGE(TAG, "Failed to init I2C0 (MPU6050)");
        return;
    }

    // ADXL345: I2C_NUM_1 (GPIO17, GPIO16)
    i2cBus1 = new SensorI2CBus(I2C_NUM_1, GPIO_NUM_17, GPIO_NUM_16);
    if (!i2cBus1->init()) {
        ESP_LOGE(TAG, "Failed to init I2C1 (ADXL345)");
        return;
    }

    // Initialize MPU6050
    mpu = new MPU6050Driver(*i2cBus0, MPU6050_ADDR);
    if (!mpu->init()) {
        ESP_LOGE(TAG, "MPU6050 init failed");
    } else {
        ESP_LOGI(TAG, "MPU6050 OK");
        mpu->startPolling(1000);
    }

    // Initialize ADXL345
    adxl = new ADXL345Driver(*i2cBus1, ADXL345_ADDR);
    if (!adxl->init()) {
        ESP_LOGE(TAG, "ADXL345 init failed");
    } else {
        ESP_LOGI(TAG, "ADXL345 OK");
        adxl->startPolling(1000);
    }

    Heartbeat::start();
    WebSocketServer::start();
    SensorFusionSim::start();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
