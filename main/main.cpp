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
#include "SFRegistry.hpp"
#include "SensorI2CBus.hpp"
#include "TaskRate.hpp"
#include "Tasks.hpp"

namespace
{

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

} // namespace

extern "C" void app_main()
{
	ESP_LOGI("APP", "Starting Sensor Fusion App...");

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

	// ---- I2C bus + drivers (locals that never go out of scope) ----
	SensorI2CBus i2cBus0(I2C_NUM_0, GPIO_NUM_1, GPIO_NUM_2);
	if (!i2cBus0.init())
	{
		ESP_LOGE(TAG, "Failed to init I2C0");
		return;
	}

	// i2c_scan(i2cBus0);

	MPU6050Driver mpu(i2cBus0, MPU6050_ADDR);
	if (!mpu.init())
	{
		ESP_LOGE(TAG, "MPU6050 init failed");
	}
	else
	{
		mpu.setBypass(true); // allow direct access to mag
		ESP_LOGI(TAG, "MPU6050 OK");
	}

	vTaskDelay(pdMS_TO_TICKS(200)); // small settle between inits

	QMC5883LDriver mag(i2cBus0, QMC5883L_ADDR);
	if (!mag.init())
	{
		ESP_LOGE(TAG, "QMC5883L init failed");
	}

	BMP180Driver bmp(i2cBus0, BMP180Driver::DEFAULT_ADDR,
					 BMP180Driver::Oversampling::ULTRA_HIGH_RES);
	if (!bmp.init())
	{
		ESP_LOGE(TAG, "BMP180 init failed");
	}

	// ---- Registry (local) ----
	SFReg reg;

	// ---- Contexts with per-task frequency (Hz) ----
	ImuCtx imuCtx{&reg, &mpu, 200.0f};	// 200 Hz
	MagCtx magCtx{&reg, &mag, 10.0f};	//  50 Hz
	BaroCtx baroCtx{&reg, &bmp, 10.0f}; // 20 Hz
	FusionCtx fusCtx{&reg, 200.0f};		// 200 Hz fusion loop

	// ---- Start tasks (pin cores/pri to taste) ----
	xTaskCreatePinnedToCore(ImuTask, "ImuTask", 4096, &imuCtx, 12, nullptr, 0);
	xTaskCreatePinnedToCore(MagTask, "MagTask", 4096, &magCtx, 10, nullptr, 0);
	xTaskCreatePinnedToCore(BaroTask, "BaroTask", 4096, &baroCtx, 9, nullptr,
							0);
	xTaskCreatePinnedToCore(FusionTask, "Fusion", 4096, &fusCtx, 11, nullptr,
							1);

	// ---- Infra/UI ----
	Heartbeat::start();
	WebSocketServer::start();

	// If you want to run ONLY real sensors, keep the sim off.
	// SensorFusionSim::start();

	// Keep app_main() alive (objects on its stack must not go out of scope)
	while (true)
		vTaskDelay(pdMS_TO_TICKS(1000));
}
