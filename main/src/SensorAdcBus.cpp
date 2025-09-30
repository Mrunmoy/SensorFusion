#include "SensorAdcBus.hpp"
#include "esp_log.h"

static const char* TAG = "SensorAdcBus";

SensorAdcBus::~SensorAdcBus()
{
    destroyCalibration();
    if (m_unitHandle)
    {
        adc_oneshot_del_unit(m_unitHandle);
        m_unitHandle = nullptr;
    }
}

bool SensorAdcBus::init(const Config& cfg)
{
    m_cfg = cfg;

    adc_oneshot_unit_init_cfg_t unitCfg = {
        .unit_id = m_cfg.unit
    };
    if (adc_oneshot_new_unit(&unitCfg, &m_unitHandle) != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed");
        return false;
    }

    adc_oneshot_chan_cfg_t chanCfg = {
        .atten = m_cfg.atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(m_unitHandle, m_cfg.channel, &chanCfg) != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed");
        return false;
    }

    if (m_cfg.calibrate)
    {
        if (!createCalibration())
        {
            ESP_LOGW(TAG, "Calibration not available; proceeding without");
        }
    }

    return true;
}

bool SensorAdcBus::readRaw(int& raw) const
{
    if (!m_unitHandle) return false;
    return adc_oneshot_read(m_unitHandle, m_cfg.channel, &raw) == ESP_OK;
}

bool SensorAdcBus::readMilliVolts(int& mv) const
{
    int raw;
    if (!readRaw(raw)) return false;

    if (m_caliEnabled && m_caliHandle)
    {
        return adc_cali_raw_to_voltage(m_caliHandle, raw, &mv) == ESP_OK;
    }
    else
    {
        mv = -1; // indicate "uncalibrated"
        return true;
    }
}

bool SensorAdcBus::reconfigure(adc_atten_t atten)
{
    if (!m_unitHandle) return false;

    if (atten == m_cfg.atten) return true;

    m_cfg.atten = atten;
    adc_oneshot_chan_cfg_t chanCfg = {
        .atten = m_cfg.atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(m_unitHandle, m_cfg.channel, &chanCfg) != ESP_OK)
        return false;

    destroyCalibration();
    if (m_cfg.calibrate)
        createCalibration();

    return true;
}

bool SensorAdcBus::createCalibration()
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t c{
        .unit_id = m_cfg.unit,
        .atten   = m_cfg.atten,
        .bitwidth= ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&c, &m_caliHandle) == ESP_OK)
    {
        m_caliEnabled = true;
        ESP_LOGI(TAG, "ADC calibration: curve-fitting");
        return true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t c{
        .unit_id = m_cfg.unit,
        .atten   = m_cfg.atten,
        .bitwidth= ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&c, &m_caliHandle) == ESP_OK)
    {
        m_caliEnabled = true;
        ESP_LOGI(TAG, "ADC calibration: line-fitting");
        return true;
    }
#endif

    m_caliEnabled = false;
    m_caliHandle = nullptr;
    return false;
}

void SensorAdcBus::destroyCalibration()
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (m_caliHandle) adc_cali_delete_scheme_curve_fitting(m_caliHandle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (m_caliHandle) adc_cali_delete_scheme_line_fitting(m_caliHandle);
#endif
    m_caliHandle = nullptr;
    m_caliEnabled = false;
}
