#include "EspAdcChannel.hpp"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

namespace sf {

EspAdcChannel::EspAdcChannel(adc_oneshot_unit_handle_t unit, adc_channel_t channel)
    : unit_(unit), channel_(channel)
{}

bool EspAdcChannel::readRaw(int32_t& out) {
    int raw = 0;
    if (adc_oneshot_read(unit_, channel_, &raw) != ESP_OK) return false;
    out = raw;
    return true;
}

bool EspAdcChannel::readMillivolts(int32_t& out) {
    int raw = 0;
    if (adc_oneshot_read(unit_, channel_, &raw) != ESP_OK) return false;

    adc_cali_handle_t cali = nullptr;
    esp_err_t createRc = ESP_ERR_NOT_SUPPORTED;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {};
    cfg.unit_id = ADC_UNIT_1;
    cfg.chan = channel_;
    cfg.atten = ADC_ATTEN_DB_12;
    cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    createRc = adc_cali_create_scheme_curve_fitting(&cfg, &cali);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {};
    cfg.unit_id = ADC_UNIT_1;
    cfg.atten = ADC_ATTEN_DB_12;
    cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
#if CONFIG_IDF_TARGET_ESP32
    cfg.default_vref = 1100;
#endif
    createRc = adc_cali_create_scheme_line_fitting(&cfg, &cali);
#endif

    if (createRc != ESP_OK || !cali) {
        out = raw;
        return true;
    }

    int mv = 0;
    bool ok = adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(cali);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(cali);
#endif
    if (!ok) return false;
    out = mv;
    return true;
}

} // namespace sf
