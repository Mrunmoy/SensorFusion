#include "MPU6050Driver.hpp"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "MPU6050Driver";

// MPU6050 registers
static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
static constexpr uint8_t REG_CONFIG = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_INT_PIN_CFG = 0x37;
static constexpr uint8_t REG_INT_ENABLE = 0x38;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_TEMP_OUT_H = 0x41;
static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
static constexpr uint8_t REG_USER_CTRL = 0x6A;
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t REG_WHO_AM_I = 0x75;

// PWR_MGMT_1 bits
static constexpr uint8_t PWR1_DEVICE_RESET = 0x80;
static constexpr uint8_t PWR1_SLEEP = 0x40;
static constexpr uint8_t PWR1_CLKSEL_PLL_X = 0x01;

// CONFIG bits
static constexpr uint8_t CONFIG_DLPF_MASK = 0x07; // DLPF_CFG[2:0]
// Bits
static constexpr uint8_t BIT_I2C_BYPASS_EN = 1 << 1; // INT_PIN_CFG[1]
static constexpr uint8_t BIT_I2C_MST_EN = 1 << 5;	 // USER_CTRL[5]

// Gyro/Accel scale factors for default ranges (±2g, ±250 dps)
static constexpr float ACCEL_LSB_2G = 16384.0f; // LSB/g
static constexpr float GYRO_LSB_250 = 131.0f;	// LSB/(°/s)

float m_gyro_lsb_per_dps = 131.0f; // FS_SEL=0 → 131 LSB/(°/s)

static bool readAccelConfigLSBperG(SensorI2CBus &bus, uint8_t addr,
								   int &lsb_per_g, uint8_t &raw_cfg)
{
	uint8_t cfg = 0;
	if (!bus.read(addr, REG_ACCEL_CONFIG, &cfg, 1))
		return false;
	raw_cfg = cfg;
	const int sel = (cfg >> 3) & 0x03; // 0=±2g,1=±4g,2=±8g,3=±16g
	static const int sens[4] = {16384, 8192, 4096, 2048};
	lsb_per_g = sens[sel];
	return true;
}

MPU6050Driver::MPU6050Driver(SensorI2CBus &bus, uint8_t addr)
	: m_bus(bus), m_addr(addr)
{
}

bool MPU6050Driver::init()
{
	// WHO_AM_I should be 0x68 (or 0x69 depending on AD0)
	uint8_t who = 0;
	{
		uint8_t tmp;
		if (!m_bus.read(m_addr, REG_WHO_AM_I, &tmp, 1))
		{
			ESP_LOGE(TAG, "Failed to read WHO_AM_I");
			return false;
		}
		who = tmp;
	}
	ESP_LOGI(TAG, "WHO_AM_I=0x%02X", who);

	// --- Soft-reset, wake and clock source ---
	ESP_LOGI(TAG, "Resetting and waking MPU6050…");
	auto ok = writeRegister(REG_PWR_MGMT_1, PWR1_DEVICE_RESET);
	ESP_ERROR_CHECK_WITHOUT_ABORT(ok ? ESP_OK : ESP_FAIL);
	vTaskDelay(pdMS_TO_TICKS(100)); // allow reset to complete

	// Wake and select PLL with X-gyro (recommended clock)
	ok = writeRegister(REG_PWR_MGMT_1, PWR1_CLKSEL_PLL_X);
	ESP_ERROR_CHECK_WITHOUT_ABORT(ok ? ESP_OK : ESP_FAIL);
	vTaskDelay(pdMS_TO_TICKS(5));

	// --- Low-pass filter & sample rate ---
	// DLPF_CFG = 3 → ~44 Hz accel / 42 Hz gyro bandwidth, 1 kHz internal rate
	ok = modifyRegister(REG_CONFIG, CONFIG_DLPF_MASK, 3);
	ESP_ERROR_CHECK_WITHOUT_ABORT(ok ? ESP_OK : ESP_FAIL);

	// SMPLRT_DIV = 4 → Output rate = 1kHz / (1 + 4) = 200 Hz
	ok = writeRegister(REG_SMPLRT_DIV, 4);
	ESP_ERROR_CHECK_WITHOUT_ABORT(ok ? ESP_OK : ESP_FAIL);

	// Read back and log
	uint8_t pwr1 = 0, cfg = 0, div = 0;
	readRegister(REG_PWR_MGMT_1, pwr1);
	readRegister(REG_CONFIG, cfg);
	readRegister(REG_SMPLRT_DIV, div);
	ESP_LOGI(TAG,
			 "PWR_MGMT_1=0x%02X (sleep=%d, clksel=%u), CONFIG=0x%02X "
			 "(DLPF=%u), SMPLRT_DIV=%u",
			 pwr1, !!(pwr1 & PWR1_SLEEP), (unsigned)(pwr1 & 0x07), cfg,
			 (unsigned)(cfg & CONFIG_DLPF_MASK), (unsigned)div);

	// (Keep your existing ACCEL_CONFIG / GYRO_CONFIG setup and I2C bypass
	// enable)

	// Wake device (clear sleep)
	if (!writeRegister(REG_PWR_MGMT_1, 0x00))
	{
		ESP_LOGE(TAG, "Failed to wake device");
		return false;
	}

	// Basic config: DLPF, sample rate, ranges
	if (!writeRegister(REG_SMPLRT_DIV, 0x07))
		return false; // ~1 kHz/(1+7) = 125 Hz
	if (!writeRegister(REG_CONFIG, 0x03))
		return false; // DLPF=3 (approx 44Hz accel, 42Hz gyro)
	if (!writeRegister(REG_GYRO_CONFIG, 0x00))
		return false; // ±250 dps
	if (!writeRegister(REG_ACCEL_CONFIG, 0x00))
		return false; // ±2 g
	if (!writeRegister(REG_INT_ENABLE, 0x00))
		return false; // no INTs

	{
		int lsb_per_g = 0;
		uint8_t cfg = 0;
		if (readAccelConfigLSBperG(m_bus, m_addr, lsb_per_g, cfg))
		{
			ESP_LOGI(TAG, "ACCEL_CONFIG=0x%02X -> AFS_SEL=%d, %d LSB/g", cfg,
					 (cfg >> 3) & 0x03, lsb_per_g);
		}
		else
		{
			ESP_LOGW(TAG, "Failed to read back ACCEL_CONFIG");
		}
	}
	{
		// MPU6050Driver.cpp (inside init())
		uint8_t gyro_cfg = 0;
		if (!readRegister(REG_GYRO_CONFIG, gyro_cfg))
		{
			ESP_LOGW(TAG, "Failed to read GYRO_CONFIG");
		}
		else
		{
			const int fs_sel = (gyro_cfg >> 3) & 0x03;
			const char *fs_txt = "±250 dps";
			float lsb_per_dps = 131.0f;
			switch (fs_sel)
			{
			case 0:
				fs_txt = "±250 dps";
				lsb_per_dps = 131.0f;
				break;
			case 1:
				fs_txt = "±500 dps";
				lsb_per_dps = 65.5f;
				break;
			case 2:
				fs_txt = "±1000 dps";
				lsb_per_dps = 32.8f;
				break;
			case 3:
				fs_txt = "±2000 dps";
				lsb_per_dps = 16.4f;
				break;
			}
			ESP_LOGI(TAG, "GYRO_CONFIG=0x%02X -> FS_SEL=%d, %s (%.1f LSB/°/s)",
					 gyro_cfg, fs_sel, fs_txt, lsb_per_dps);

			// cache the scale so conversions are always correct
			m_gyro_lsb_per_dps = lsb_per_dps;

			// If you want a heads-up when FS isn’t the expected ±250:
			if (fs_sel != 0)
			{
				ESP_LOGW(TAG,
						 "FS_SEL=%d (not ±250 dps). Using %.1f LSB/°/s for "
						 "conversion.",
						 fs_sel, lsb_per_dps);
			}
		}
	}

	return true;
}

bool MPU6050Driver::readRegister(uint8_t reg, uint8_t &val)
{
	uint8_t v = 0;
	// Adjust this call to your SensorI2CBus API if needed:
	// expected signature: bus->read(device_addr, reg, buf, len)
	if (!m_bus.read(m_addr, reg, &v, 1))
	{
		ESP_LOGW(TAG, "read reg 0x%02X failed", reg);
		return false;
	}
	val = v;
	return true;
}

bool MPU6050Driver::modifyRegister(uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t cur;
	if (!readRegister(reg, cur))
		return false;
	uint8_t next = (cur & ~mask) | (value & mask);
	return writeRegister(reg, next);
}

bool MPU6050Driver::writeRegister(uint8_t reg, uint8_t value)
{
	return m_bus.write(m_addr, reg, &value, 1);
}

bool MPU6050Driver::readRawData(uint8_t start_reg, int16_t &x, int16_t &y,
								int16_t &z)
{
	uint8_t buf[6];
	if (!m_bus.read(m_addr, start_reg, buf, sizeof(buf)))
	{
		return false;
	}
	x = (int16_t)((buf[0] << 8) | buf[1]);
	y = (int16_t)((buf[2] << 8) | buf[3]);
	z = (int16_t)((buf[4] << 8) | buf[5]);
	return true;
}

bool MPU6050Driver::readAcceleration(float &ax, float &ay, float &az)
{
	int16_t x, y, z;
	if (!readRawData(REG_ACCEL_XOUT_H, x, y, z))
		return false;
	ax = static_cast<float>(x) / ACCEL_LSB_2G * 9.80665f;
	ay = static_cast<float>(y) / ACCEL_LSB_2G * 9.80665f;
	az = static_cast<float>(z) / ACCEL_LSB_2G * 9.80665f;
	return true;
}

bool MPU6050Driver::readGyroscope(float &gx, float &gy, float &gz)
{
	int16_t x, y, z;
	if (!readRawData(REG_GYRO_XOUT_H, x, y, z))
		return false;
	gx = static_cast<float>(x) / m_gyro_lsb_per_dps; // deg/s
	gy = static_cast<float>(y) / m_gyro_lsb_per_dps;
	gz = static_cast<float>(z) / m_gyro_lsb_per_dps;
	return true;
}

bool MPU6050Driver::readTemperature(float &temp)
{
	uint8_t buf[2];
	if (!m_bus.read(m_addr, REG_TEMP_OUT_H, buf, sizeof(buf)))
		return false;
	int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
	// From datasheet: Temp in °C = (raw / 340) + 36.53
	temp = (static_cast<float>(raw) / 340.0f) + 36.53f;
	return true;
}

// Minimal convenience wrapper that fills SI-ish units if your driver already
// does. If your driver exposes raw reads, convert using your configured ranges.
bool MPU6050Driver::sample(float &ax, float &ay, float &az, float &gx,
						   float &gy, float &gz)
{
	bool ok_acc = readAcceleration(ax, ay, az);
	bool ok_gyr = readGyroscope(gx, gy, gz);
	return ok_acc && ok_gyr;
}

void MPU6050Driver::addObserver(SensorObserver *observer)
{
	m_observers.push_back(observer);
}

void MPU6050Driver::notifyObservers()
{
	for (auto *o : m_observers)
	{
		o->onSensorUpdated();
	}
}

bool MPU6050Driver::setBypass(bool enable)
{
	// Make sure internal I2C master is OFF when enabling bypass
	if (enable && !writeRegister(REG_USER_CTRL, 0x00))
	{
		return false;
	}

	uint8_t cfg = 0;
	if (!m_bus.read(m_addr, REG_INT_PIN_CFG, &cfg, 1))
		return false;
	if (enable)
		cfg |= BIT_I2C_BYPASS_EN;
	else
		cfg &= ~BIT_I2C_BYPASS_EN;

	ESP_LOGI(TAG, "I2C bypass %s (INT_PIN_CFG=0x%02X)",
			 (enable ? "enabled" : "disabled"), cfg);

	return writeRegister(REG_INT_PIN_CFG, cfg);
}

bool MPU6050Driver::readRawAccel(int16_t &x, int16_t &y, int16_t &z)
{
	return readRawData(REG_ACCEL_XOUT_H, x, y, z);
}
