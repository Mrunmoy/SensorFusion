#pragma once

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include <cstdint>
#include <optional>

class SensorAdcBus
{
  public:
	struct Config
	{
		adc_unit_t unit = ADC_UNIT_1;		   // Use ADC1 on ESP32-S3
		adc_channel_t channel = ADC_CHANNEL_0; // GPIO1 == ADC1_CH0 on S3
		adc_atten_t atten = ADC_ATTEN_DB_12;   // ~0..3.3V on S3
		bool calibrate = true;
	};

	SensorAdcBus() = default;
	~SensorAdcBus();

	bool init(const Config &cfg);
	bool readRaw(int &raw) const;
	bool readMilliVolts(int &mv) const; // returns true; mv = -1 if uncalibrated

	bool reconfigure(adc_atten_t atten);

	// Introspection
	inline adc_channel_t channel() const
	{
		return m_cfg.channel;
	}
	inline adc_atten_t attenuation() const
	{
		return m_cfg.atten;
	}
	inline bool calibrated() const
	{
		return m_caliEnabled;
	}

  private:
	bool createCalibration();
	void destroyCalibration();

  private:
	Config m_cfg{};
	adc_oneshot_unit_handle_t m_unitHandle = nullptr;
	adc_cali_handle_t m_caliHandle = nullptr;
	bool m_caliEnabled = false;
};
