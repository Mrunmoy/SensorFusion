#pragma once

#include "SensorAdcBus.hpp"
#include "SensorObserver.hpp"
#include <vector>
#include <cstdint>

class Ad8232Driver
{
public:
    enum class Attenuation : uint8_t
    {
        DB_0,   // not typical for 3.3V system
        DB_2_5,
        DB_6,
        DB_11   // good for 0..~3.3V
    };

    struct Config
    {
        // On ESP32-S3: GPIO1 = ADC1 channel 0 by default
        adc_unit_t     unit       = ADC_UNIT_1;
        adc_channel_t  channel    = ADC_CHANNEL_0;
        Attenuation    atten      = Attenuation::DB_11;
        bool           calibrate  = true;
    };

    explicit Ad8232Driver(SensorAdcBus& bus);

    bool init(const Config& cfg);             // sets up ADC channel + (optional) calibration
    bool configure(Attenuation atten);        // change attenuation at runtime

    // Read APIs (non-blocking, one-shot)
    bool readRaw(int& raw) const;
    bool readMilliVolts(int& mv) const;

    // Convenience: returns true and fills mv; if uncalibrated, mv == -1
    bool sample(int& mv);

    // Observer pattern (e.g., push to processing chain)
    void addObserver(SensorObserver* obs);

private:
    SensorAdcBus::Config toAdcConfig(const Config& cfg) const;
    adc_atten_t toEspAtten(Attenuation a) const;
    void notifyObservers();

private:
    SensorAdcBus& m_bus;
    Config m_cfg{};
    std::vector<SensorObserver*> m_observers;
};
