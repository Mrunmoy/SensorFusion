#include "Ad8232Driver.hpp"
#include "esp_log.h"

static const char *TAG = "Ad8232";

Ad8232Driver::Ad8232Driver(SensorAdcBus &bus)
	: m_bus(bus)
{
}

bool Ad8232Driver::init(const Config &cfg)
{
	m_cfg = cfg;
	SensorAdcBus::Config adcCfg = toAdcConfig(cfg);
	if (!m_bus.init(adcCfg))
	{
		ESP_LOGE(TAG, "ADC bus init failed");
		return false;
	}
	return true;
}

bool Ad8232Driver::configure(Attenuation atten)
{
	if (atten == m_cfg.atten)
		return true;
	m_cfg.atten = atten;
	return m_bus.reconfigure(toEspAtten(atten));
}

bool Ad8232Driver::readRaw(int &raw) const
{
	return m_bus.readRaw(raw);
}

bool Ad8232Driver::readMilliVolts(int &mv) const
{
	return m_bus.readMilliVolts(mv);
}

bool Ad8232Driver::sample(int &mv)
{
	if (!readMilliVolts(mv))
		return false;
	notifyObservers();
	return true;
}

void Ad8232Driver::addObserver(SensorObserver *obs)
{
	if (!obs)
		return;
	m_observers.push_back(obs);
}

void Ad8232Driver::notifyObservers()
{
	for (auto *obs : m_observers)
	{
		if (obs)
			obs->onSensorUpdated(); // assuming your SensorObserver has this
									// hook
	}
}

SensorAdcBus::Config Ad8232Driver::toAdcConfig(const Config &cfg) const
{
	SensorAdcBus::Config out;
	out.unit = cfg.unit;
	out.channel = cfg.channel;
	out.atten = toEspAtten(cfg.atten);
	out.calibrate = cfg.calibrate;
	return out;
}

adc_atten_t Ad8232Driver::toEspAtten(Attenuation a) const
{
	switch (a)
	{
	case Attenuation::DB_0:
		return ADC_ATTEN_DB_0;
	case Attenuation::DB_2_5:
		return ADC_ATTEN_DB_2_5;
	case Attenuation::DB_6:
		return ADC_ATTEN_DB_6;
	// IDF deprecates 11 dB in favor of 12 dB (same behavior)
	case Attenuation::DB_11:
		return ADC_ATTEN_DB_12;
	default:
		return ADC_ATTEN_DB_12;
	}
}
