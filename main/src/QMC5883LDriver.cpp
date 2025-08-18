#include "QMC5883LDriver.hpp"
#include "esp_log.h"
#include <cmath>

static const char *TAG = "QMC5883L";

// --- Device specifics hidden in anon namespace ---
namespace
{
// Registers
constexpr uint8_t REG_X_LSB = 0x00;
constexpr uint8_t REG_STATUS = 0x06;
constexpr uint8_t REG_TOUT_L = 0x07; // (unused)
constexpr uint8_t REG_CTRL1 = 0x09;
constexpr uint8_t REG_CTRL2 = 0x0A;
constexpr uint8_t REG_SET_RST = 0x0B;

// STATUS bits
constexpr uint8_t STATUS_DRDY = 0x01; // data ready
constexpr uint8_t STATUS_OVL = 0x02;  // overflow
constexpr uint8_t STATUS_DOR = 0x04;  // data skipped (overrun)

// CTRL1 fields
constexpr uint8_t CTRL1_OSR_SHIFT = 6;
constexpr uint8_t CTRL1_RNG_SHIFT = 4;
constexpr uint8_t CTRL1_ODR_SHIFT = 2;
constexpr uint8_t CTRL1_MODE_SHIFT = 0;

// CTRL2 fields
constexpr uint8_t CTRL2_SOFT_RST = 0x80;

// enum -> bitfield helpers
constexpr uint8_t to_osr(QMC5883LDriver::Oversample osr)
{
	switch (osr)
	{
	case QMC5883LDriver::Oversample::OSR_512:
		return 0b00;
	case QMC5883LDriver::Oversample::OSR_256:
		return 0b01;
	case QMC5883LDriver::Oversample::OSR_128:
		return 0b10;
	case QMC5883LDriver::Oversample::OSR_64:
		return 0b11;
	}
	return 0b00;
}
constexpr uint8_t to_rng(QMC5883LDriver::Range rng)
{
	switch (rng)
	{
	case QMC5883LDriver::Range::GAUSS_2:
		return 0b00;
	case QMC5883LDriver::Range::GAUSS_8:
		return 0b01;
	}
	return 0b01;
}
constexpr uint8_t to_odr(QMC5883LDriver::ODR odr)
{
	switch (odr)
	{
	case QMC5883LDriver::ODR::HZ_10:
		return 0b00;
	case QMC5883LDriver::ODR::HZ_50:
		return 0b01;
	case QMC5883LDriver::ODR::HZ_100:
		return 0b10;
	case QMC5883LDriver::ODR::HZ_200:
		return 0b11;
	}
	return 0b11;
}
constexpr uint8_t to_mode(QMC5883LDriver::Mode m)
{
	switch (m)
	{
	case QMC5883LDriver::Mode::STANDBY:
		return 0b00;
	case QMC5883LDriver::Mode::CONTINUOUS:
		return 0b01;
	}
	return 0b01;
}

constexpr uint8_t make_ctrl1(QMC5883LDriver::Oversample osr,
							 QMC5883LDriver::Range rng, QMC5883LDriver::ODR odr,
							 QMC5883LDriver::Mode mode)
{
	return (to_osr(osr) << CTRL1_OSR_SHIFT) | (to_rng(rng) << CTRL1_RNG_SHIFT) |
		   (to_odr(odr) << CTRL1_ODR_SHIFT) |
		   (to_mode(mode) << CTRL1_MODE_SHIFT);
}

// Scale (rough; refine with calibration)
constexpr float LSB_PER_GAUSS_2G = 12000.0f;
constexpr float LSB_PER_GAUSS_8G = 3000.0f;
constexpr float GAUSS_TO_uT = 100.0f;

inline float lsb_to_uT(int16_t v, QMC5883LDriver::Range rng)
{
	const float lsb_per_gauss = (rng == QMC5883LDriver::Range::GAUSS_2)
									? LSB_PER_GAUSS_2G
									: LSB_PER_GAUSS_8G;
	return (static_cast<float>(v) / lsb_per_gauss) * GAUSS_TO_uT;
}
} // namespace

// --- Public API ---

QMC5883LDriver::QMC5883LDriver(SensorI2CBus &bus, uint8_t addr)
	: m_bus(bus), m_addr(addr)
{
}

bool QMC5883LDriver::init()
{
	// Soft reset
	if (!writeReg(REG_CTRL2, CTRL2_SOFT_RST))
	{
		ESP_LOGE(TAG, "Soft reset write failed");
		return false;
	}
	vTaskDelay(pdMS_TO_TICKS(10));

	// Set/Reset period (common values: 0x01 or 0x11)
	if (!writeReg(REG_SET_RST, 0x01))
	{
		ESP_LOGW(TAG, "SET/RESET write failed (continuing)");
	}

	// uint8_t id_A, id_B, id_C;
	// bool success = m_bus.read(m_addr, 0x0A, &id_A, 1);
	// success &= m_bus.read(m_addr, 0x0B, &id_B, 1);
	// success &= m_bus.read(m_addr, 0x0C, &id_C, 1);

	// if (success)
	// {
	// 	ESP_LOGI(TAG, "HMC5883L ID: %c%c%c (%d, %d, %d)", id_A, id_B, id_C,
	// 			 id_A, id_B, id_C);
	// 	// HMC5883L chip will return 'H', '4', '3'
	// }
	// uint8_t chip_id = 0;
	// constexpr uint8_t REG_CHIP_ID = 0x0F;
	// constexpr uint8_t CHIP_ID_EXPECTED = 0xFF; // As per datasheet
	// Section 6.5 if (!readReg(REG_CHIP_ID, &chip_id, 1))
	// {
	// 	ESP_LOGE(TAG, "Failed to read CHIP_ID");
	// 	return false;
	// }
	// if (chip_id != CHIP_ID_EXPECTED)
	// {
	// 	ESP_LOGE(TAG, "Unexpected CHIP_ID=0x%02X (expected 0x%02X)", chip_id,
	// 			 CHIP_ID_EXPECTED);
	// 	return false;
	// }
	// ESP_LOGI(TAG, "Chip ID check OK: 0x%02X", chip_id);

	// Defaults
	m_osr = Oversample::OSR_512;
	m_rng = Range::GAUSS_8;
	m_odr = ODR::HZ_200;
	m_mode = Mode::CONTINUOUS;

	const uint8_t ctrl1 = make_ctrl1(m_osr, m_rng, m_odr, m_mode);
	if (!writeReg(REG_CTRL1, ctrl1))
	{
		ESP_LOGE(TAG, "CTRL1 write failed");
		return false;
	}

	vTaskDelay(pdMS_TO_TICKS(20));

	uint8_t rb = 0;
	if (readReg(REG_CTRL1, &rb, 1))
	{
		ESP_LOGI(TAG, "CTRL1=0x%02X", rb);
	}

	ESP_LOGI(TAG, "QMC5883L init OK (addr 0x%02X)", m_addr);
	return true;
}

bool QMC5883LDriver::configure(Oversample osr, Range rng, ODR odr, Mode mode)
{
	const uint8_t ctrl1 = make_ctrl1(osr, rng, odr, mode);
	if (!writeReg(REG_CTRL1, ctrl1))
	{
		ESP_LOGE(TAG, "CTRL1 write failed");
		return false;
	}
	m_osr = osr;
	m_rng = rng;
	m_odr = odr;
	m_mode = mode;
	vTaskDelay(pdMS_TO_TICKS(10));
	return true;
}

bool QMC5883LDriver::readRaw(int16_t &x, int16_t &y, int16_t &z,
							 uint32_t timeoutMs) const
{
	const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
	uint8_t status = 0;

	while (xTaskGetTickCount() < deadline)
	{
		if (!readReg(REG_STATUS, &status, 1))
		{
			ESP_LOGW(TAG, "STATUS read failed");
			return false;
		}
		if (status & STATUS_OVL)
		{
			ESP_LOGW(TAG, "Overflow flagged; discarding sample");
			return false;
		}
		if (status & STATUS_DRDY)
			break;
		vTaskDelay(pdMS_TO_TICKS(2));
	}
	if (!(status & STATUS_DRDY))
	{
		ESP_LOGW(TAG, "DRDY timeout (status=0x%02X)", status);
		return false;
	}

	uint8_t buf[6];
	if (!readReg(REG_X_LSB, buf, sizeof(buf)))
	{
		ESP_LOGW(TAG, "XYZ read failed");
		return false;
	}

	x = (int16_t)((buf[1] << 8) | buf[0]);
	y = (int16_t)((buf[3] << 8) | buf[2]);
	z = (int16_t)((buf[5] << 8) | buf[4]);

	if (x == 0 && y == 0 && z == 0)
	{
		ESP_LOGW(TAG, "All-zero sample (sensor may be settling)");
		return false;
	}
	return true;
}

bool QMC5883LDriver::readMicroTesla(float &x_uT, float &y_uT, float &z_uT,
									uint32_t timeoutMs) const
{
	int16_t x, y, z;
	if (!readRaw(x, y, z, timeoutMs))
		return false;
	x_uT = lsb_to_uT(x, m_rng);
	y_uT = lsb_to_uT(y, m_rng);
	z_uT = lsb_to_uT(z, m_rng);
	return true;
}

bool QMC5883LDriver::headingDegrees(float &heading_deg,
									uint32_t timeoutMs) const
{
	float x_uT, y_uT, z_uT;
	if (!readMicroTesla(x_uT, y_uT, z_uT, timeoutMs))
		return false;

	// Level-only heading
	float heading = std::atan2f(y_uT, x_uT) * 180.0f / float(M_PI);
	if (heading < 0)
		heading += 360.0f;
	heading_deg = heading;
	return true;
}

// --- I2C helpers ---
bool QMC5883LDriver::writeReg(uint8_t reg, uint8_t val) const
{
	return m_bus.write(m_addr, reg, &val, 1);
}
bool QMC5883LDriver::readReg(uint8_t reg, uint8_t *buf, size_t len) const
{
	return m_bus.read(m_addr, reg, buf, len);
}

// --- Observers / Polling ---
void QMC5883LDriver::addObserver(SensorObserver *obs)
{
	m_observers.push_back(obs);
}
void QMC5883LDriver::notifyObservers()
{
	for (auto *o : m_observers)
		if (o)
			o->onSensorUpdated();
}

void QMC5883LDriver::startPolling(uint32_t intervalMs)
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
			auto *self = static_cast<QMC5883LDriver *>(arg);
			ESP_LOGI(TAG, "Polling started (%u ms)", self->m_intervalMs);

			while (self->m_running)
			{
				float x_uT = 0, y_uT = 0, z_uT = 0;
				float heading = 0;
				bool ok = self->readMicroTesla(x_uT, y_uT, z_uT, 50);
				if (ok)
					ok = self->headingDegrees(
						heading, 50); // uses last sample; 0ms timeout

				if (ok)
				{
					ESP_LOGI(TAG, "Mag: %.2f/%.2f/%.2f uT, Heading=%.1f°", x_uT,
							 y_uT, z_uT, heading);
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
		"QMC5883LTask", 4096, this, 5, &m_taskHandle);
}

void QMC5883LDriver::stopPolling()
{
	if (!m_taskHandle)
		return;
	m_running = false;
	while (m_taskHandle != nullptr)
	{
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	ESP_LOGI(TAG, "Polling stopped");
}
