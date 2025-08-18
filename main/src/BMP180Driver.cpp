#include "BMP180Driver.hpp"
#include "esp_log.h"
#include <cmath>

static const char *TAG = "BMP180";

BMP180Driver::BMP180Driver(SensorI2CBus &bus, uint8_t addr, Oversampling oss)
	: m_bus(bus), m_addr(addr), m_oss(oss)
{
}

bool BMP180Driver::readU8(uint8_t reg, uint8_t &val)
{
	return m_bus.read(m_addr, reg, &val, 1);
}
bool BMP180Driver::writeU8(uint8_t reg, uint8_t val)
{
	return m_bus.write(m_addr, reg, &val, 1);
}
bool BMP180Driver::readBytes(uint8_t reg, uint8_t *b, size_t n)
{
	return m_bus.read(m_addr, reg, b, n);
}

bool BMP180Driver::init()
{
	uint8_t id = 0;
	if (!readU8(REG_CHIP_ID, id))
	{
		ESP_LOGE(TAG, "Failed to read CHIP_ID");
		return false;
	}
	if (id != CHIP_ID_EXPECT)
	{
		ESP_LOGE(TAG, "Unexpected CHIP_ID=0x%02X (expected 0x%02X)", id,
				 CHIP_ID_EXPECT);
		return false;
	}

	uint8_t buf[22];
	if (!readBytes(REG_CALIB_START, buf, sizeof(buf)))
	{
		ESP_LOGE(TAG, "Failed to read calibration");
		return false;
	}
	auto rdS16 = [&](int i) -> int16_t
	{ return int16_t((buf[i] << 8) | buf[i + 1]); };
	auto rdU16 = [&](int i) -> uint16_t
	{ return uint16_t((buf[i] << 8) | buf[i + 1]); };

	calib_.AC1 = rdS16(0);
	calib_.AC2 = rdS16(2);
	calib_.AC3 = rdS16(4);
	calib_.AC4 = rdU16(6);
	calib_.AC5 = rdU16(8);
	calib_.AC6 = rdU16(10);
	calib_.B1 = rdS16(12);
	calib_.B2 = rdS16(14);
	calib_.MB = rdS16(16);
	calib_.MC = rdS16(18);
	calib_.MD = rdS16(20);
	calib_.valid = true;

	ESP_LOGI(TAG,
			 "Calib OK: AC1=%d AC2=%d AC3=%d AC4=%u AC5=%u AC6=%u B1=%d B2=%d "
			 "MC=%d MD=%d",
			 calib_.AC1, calib_.AC2, calib_.AC3, calib_.AC4, calib_.AC5,
			 calib_.AC6, calib_.B1, calib_.B2, calib_.MC, calib_.MD);
	return true;
}

bool BMP180Driver::readUncompensatedTemp(int32_t &UT)
{
	if (!writeU8(REG_CTRL_MEAS, CMD_TEMP))
		return false;
	vTaskDelay(pdMS_TO_TICKS(5)); // typ 4.5ms
	uint8_t msb, lsb;
	if (!readU8(REG_OUT_MSB, msb))
		return false;
	if (!readU8(REG_OUT_MSB + 1, lsb))
		return false;
	UT = (int32_t)((msb << 8) | lsb);
	return true;
}

bool BMP180Driver::readUncompensatedPressure(int32_t &UP)
{
	uint8_t oss = static_cast<uint8_t>(m_oss);
	if (!writeU8(REG_CTRL_MEAS, uint8_t(CMD_PRESS_BASE | (oss << 6))))
		return false;

	static const uint8_t ms_table[4] = {5, 8, 14, 26};
	vTaskDelay(pdMS_TO_TICKS(ms_table[oss]));

	uint8_t msb, lsb, xlsb;
	if (!readU8(REG_OUT_MSB, msb))
		return false;
	if (!readU8(REG_OUT_MSB + 1, lsb))
		return false;
	if (!readU8(REG_OUT_MSB + 2, xlsb))
		return false;

	UP = (((int32_t)msb << 16) | ((int32_t)lsb << 8) | (int32_t)xlsb) >>
		 (8 - oss);
	return true;
}

bool BMP180Driver::computeTrueTempC(int32_t UT, float &temp_c, int32_t &B5)
{
	if (!calib_.valid)
		return false;
	int32_t X1 = ((UT - (int32_t)calib_.AC6) * (int32_t)calib_.AC5) >> 15;
	int32_t X2 = ((int32_t)calib_.MC << 11) / (X1 + calib_.MD);
	B5 = X1 + X2;
	int32_t T = (B5 + 8) >> 4; // 0.1 °C
	temp_c = T / 10.0f;
	return true;
}

bool BMP180Driver::computeTruePressurePa(int32_t UP, int32_t B5, int32_t &P)
{
	if (!calib_.valid)
		return false;
	int32_t oss = static_cast<uint8_t>(m_oss);
	int32_t B6 = B5 - 4000;

	int32_t X1 = ((int32_t)calib_.B2 * ((B6 * B6) >> 12)) >> 11;
	int32_t X2 = ((int32_t)calib_.AC2 * B6) >> 11;
	int32_t X3 = X1 + X2;
	int32_t B3 = ((((int32_t)calib_.AC1 * 4 + X3) << oss) + 2) >> 2;

	X1 = ((int32_t)calib_.AC3 * B6) >> 13;
	X2 = ((int32_t)calib_.B1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	uint32_t B4 = ((uint32_t)calib_.AC4 * (uint32_t)(X3 + 32768)) >> 15;
	uint32_t B7 = (uint32_t)(UP - B3) * (uint32_t)(50000 >> oss);

	int32_t p;
	if (B7 < 0x80000000UL)
		p = (int32_t)((B7 << 1) / B4);
	else
		p = (int32_t)((B7 / B4) << 1);

	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	p = p + ((X1 + X2 + 3791) >> 4);

	P = p;
	return true;
}

bool BMP180Driver::readTempAndPressure(float &temp_c, int32_t &pressure_pa)
{
	if (!calib_.valid)
	{
		ESP_LOGE(TAG, "Calibration data not valid");
		return false;
	}
	// Step 1: Read uncompensated temperature
	int32_t UT;
	if (!readUncompensatedTemp(UT))
		return false;

	// Step 2: Compensate temperature and get B5
	int32_t B5;
	if (!computeTrueTempC(UT, temp_c, B5))
		return false;

	// Step 3: Read uncompensated pressure
	int32_t UP;
	if (!readUncompensatedPressure(UP))
		return false;

	// Step 4: Compensate pressure using B5
	return computeTruePressurePa(UP, B5, pressure_pa);
}

// Separate functions could be wrappers for the combined function.
bool BMP180Driver::readTemperatureC(float &temp_c)
{
	int32_t pressure_pa;
	return readTempAndPressure(temp_c, pressure_pa);
}

bool BMP180Driver::readPressurePa(int32_t &pressure_pa)
{
	float temp_c;
	return readTempAndPressure(temp_c, pressure_pa);
}

float BMP180Driver::pressureToAltitudeMeters(int32_t pressure_pa, int32_t p0_pa)
{
	return 44330.0f *
		   (1.0f - powf((float)pressure_pa / (float)p0_pa, 0.19029495f));
}

// --- Polling + observer ---

void BMP180Driver::addObserver(SensorObserver *obs)
{
	m_observers.push_back(obs);
}
void BMP180Driver::notifyObservers()
{
	for (auto *o : m_observers)
		if (o)
			o->onSensorUpdated();
}

void BMP180Driver::startPolling(uint32_t intervalMs)
{
	if (m_running)
	{
		ESP_LOGW(TAG, "Polling already running");
		return;
	}
	m_intervalMs = intervalMs;
	m_running = true;

	xTaskCreate(
		[](void *arg)
		{
			auto *self = static_cast<BMP180Driver *>(arg);
			ESP_LOGI(TAG, "Polling started (interval=%ums)",
					 self->m_intervalMs);
			while (self->m_running)
			{
				float tC = 0.0f;
				int32_t pPa = 0;
				if (self->readTempAndPressure(tC, pPa))
				{
					float alt_m = BMP180Driver::pressureToAltitudeMeters(pPa);
					ESP_LOGI(TAG, "T=%.2f C, P=%ld Pa, Alt=%.1f m", tC,
							 (long)pPa, alt_m);
					self->notifyObservers();
				}
				else
				{
					ESP_LOGW(TAG, "Read failed");
				}
				vTaskDelay(pdMS_TO_TICKS(self->m_intervalMs));
			}
			self->m_taskHandle = nullptr;
			vTaskDelete(nullptr);
		},
		"BMP180Task", 4096, this, 5, &m_taskHandle);
}

void BMP180Driver::stopPolling()
{
	if (!m_taskHandle)
		return;
	m_running = false;
	// Wait for task to self-delete
	while (m_taskHandle != nullptr)
	{
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	ESP_LOGI(TAG, "Polling stopped");
}
