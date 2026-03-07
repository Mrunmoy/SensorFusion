#include "EspNvStore.hpp"

extern "C" {
#include "nvs_flash.h"
}

namespace sf {

EspNvStore::EspNvStore(const char* ns, const char* key, size_t capacityBytes)
    : ns_(ns ? ns : "sensorfusion"),
      key_(key ? key : "nv"),
      capacity_(capacityBytes),
      cache_(capacityBytes, 0xFF)
{
    nvs_flash_init();
    if (nvs_open(ns_.c_str(), NVS_READWRITE, &handle_) == ESP_OK) {
        loadCache();
    }
}

EspNvStore::~EspNvStore() {
    if (handle_ != 0) {
        nvs_close(handle_);
    }
}

bool EspNvStore::inRange(uint32_t address, size_t len) const {
    return len <= capacity_ && address <= capacity_ - len;
}

bool EspNvStore::loadCache() {
    if (handle_ == 0) return false;

    size_t len = capacity_;
    esp_err_t rc = nvs_get_blob(handle_, key_.c_str(), cache_.data(), &len);
    if (rc == ESP_ERR_NVS_NOT_FOUND) {
        return flushCache();
    }
    if (rc != ESP_OK) return false;

    if (len < capacity_) {
        for (size_t i = len; i < capacity_; ++i) cache_[i] = 0xFF;
    }
    return true;
}

bool EspNvStore::flushCache() {
    if (handle_ == 0) return false;
    if (nvs_set_blob(handle_, key_.c_str(), cache_.data(), cache_.size()) != ESP_OK) return false;
    return nvs_commit(handle_) == ESP_OK;
}

bool EspNvStore::read(uint32_t address, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    if (!inRange(address, len)) return false;
    for (size_t i = 0; i < len; ++i) {
        buf[i] = cache_[address + i];
    }
    return true;
}

bool EspNvStore::write(uint32_t address, const uint8_t* data, size_t len) {
    if (!data || len == 0) return false;
    if (!inRange(address, len)) return false;

    for (size_t i = 0; i < len; ++i) {
        cache_[address + i] = data[i];
    }
    return flushCache();
}

} // namespace sf
