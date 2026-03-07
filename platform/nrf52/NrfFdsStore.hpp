#pragma once

#include "INvStore.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace sf {

class NrfFdsStore : public INvStore {
public:
    NrfFdsStore(uint16_t fileId, uint16_t recordKey, size_t capacityBytes);

    bool read(uint32_t address, uint8_t* buf, size_t len) override;
    bool write(uint32_t address, const uint8_t* data, size_t len) override;
    size_t capacity() const override { return cache_.size(); }

private:
    bool loadFromFds();
    bool flushToFds();
    bool inRange(uint32_t address, size_t len) const;

    uint16_t fileId_;
    uint16_t key_;
    std::vector<uint8_t> cache_;
};

} // namespace sf
