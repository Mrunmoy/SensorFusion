#pragma once

#include "INvStore.hpp"
#include <string>
#include <vector>

extern "C" {
#include "nvs.h"
}

namespace sf {

class EspNvStore : public INvStore {
public:
    EspNvStore(const char* ns, const char* key, size_t capacityBytes);
    ~EspNvStore() override;

    bool read(uint32_t address, uint8_t* buf, size_t len) override;
    bool write(uint32_t address, const uint8_t* data, size_t len) override;
    size_t capacity() const override { return capacity_; }

private:
    bool loadCache();
    bool flushCache();
    bool inRange(uint32_t address, size_t len) const;

    std::string ns_;
    std::string key_;
    size_t capacity_;
    nvs_handle_t handle_ = 0;
    std::vector<uint8_t> cache_;
};

} // namespace sf
