#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Heartbeat.hpp"
#include "SensorFusionSim.hpp"
#include "WebSocketServer.hpp"
#include "WifiManager.hpp"

#include "BMP180Driver.hpp"
#include "MPU6050Driver.hpp"
#include "QMC5883LDriver.hpp"
#include "SensorI2CBus.hpp"

static const char *TAG = "MainApp";

constexpr uint8_t MPU6050_ADDR = 0x68; // Common default I2C address for MPU6050
constexpr uint8_t ADXL345_ADDR = 0x53; // Common default I2C address for ADXL345
constexpr uint8_t QMC5883L_ADDR =
	0x0D;							  // Common default I2C address for QMC5883L
constexpr uint8_t BME180_ADDR = 0x77; // Common default I2C address for BME280

void i2c_scan(SensorI2CBus &bus)
{
	for (uint8_t addr = 1; addr < 0x7F; ++addr)
	{
		uint8_t dummy;
		if (bus.read(addr, 0x00, &dummy, 1, pdMS_TO_TICKS(20)))
		{
			ESP_LOGI("I2C_SCAN", "Found device at 0x%02X", addr);
		}
	}
}

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
		if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
			ip_info.ip.addr != 0)
			break;
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	// Define sensor drivers and buses
	SensorI2CBus i2cBus0 = SensorI2CBus(I2C_NUM_0, GPIO_NUM_1, GPIO_NUM_2);
	if (!i2cBus0.init())
	{
		ESP_LOGE(TAG, "Failed to init I2C0 (MPU6050)");
		return;
	}

	// i2c_scan(i2cBus0);

	// Initialize MPU6050
	MPU6050Driver mpu = MPU6050Driver(i2cBus0, MPU6050_ADDR);
	if (!mpu.init())
	{
		ESP_LOGE(TAG, "MPU6050 init failed");
	}
	else
	{
		mpu.setBypass(true); // Enable bypass for magnetometer access
		// i2c_scan(i2cBus0);
		ESP_LOGI(TAG, "MPU6050 OK");
		// mpu->startPolling(1000);
	}

	vTaskDelay(pdMS_TO_TICKS(1000));

	QMC5883LDriver mag(i2cBus0, QMC5883L_ADDR);
	if (mag.init())
	{
		int16_t mx, my, mz;
		if (mag.readRaw(mx, my, mz))
		{
			ESP_LOGI("MAG", "Raw: X=%d Y=%d Z=%d", mx, my, mz);
			mag.startPolling(1000);
		}
		else
		{
			ESP_LOGE("MAG", "Failed to read magnetometer data");
		}
	}
	else
	{
		ESP_LOGE(TAG, "QMC5883L init failed");
	}

	// optional:
	// qmc.configure(QMC5883LDriver::Oversample::OSR_512,
	//               QMC5883LDriver::Range::GAUSS_8,
	//               QMC5883LDriver::ODR::HZ_200,
	//               QMC5883LDriver::Mode::CONTINUOUS);

	float x_uT, y_uT, z_uT;
	if (mag.readMicroTesla(x_uT, y_uT, z_uT))
	{
		ESP_LOGI("MAG", "MicroTesla: X=%.2f Y=%.2f Z=%.2f", x_uT, y_uT, z_uT);
		// use vectors, heading, fusion, etc.
	}

	BMP180Driver bmp(i2cBus0, BMP180Driver::DEFAULT_ADDR,
					 BMP180Driver::Oversampling::ULTRA_HIGH_RES);

	if (bmp.init())
	{
		float tC = 0.0f;
		int32_t pPa = 0;
		if (bmp.readTempAndPressure(tC, pPa))
		{
			float alt_m = BMP180Driver::pressureToAltitudeMeters(
				pPa); // sea-level 101325 Pa
			ESP_LOGI("BMP180", "T=%.2f C, P=%ld Pa, Alt=%.1f m", tC, (long)pPa,
					 alt_m);
			bmp.startPolling();
		}
		else
		{
			ESP_LOGE("BMP180", "Failed to read temperature and pressure");
		}
	}
	else
	{
		ESP_LOGE("BMP180", "BMP180 init failed");
	}

	Heartbeat::start();
	WebSocketServer::start();
	SensorFusionSim::start();

	while (true)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
