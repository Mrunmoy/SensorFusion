#pragma once

#include "IDelayProvider.hpp"
#include "II2CBus.hpp"
#include "INvStore.hpp"
#include <cstddef>
#include <cstdint>

namespace sf {

struct AT24CxxConfig {
    uint8_t i2cAddress = 0x50;
    size_t totalBytes = 4096;
    size_t pageSize = 16;
    uint8_t addressWidth = 2;   // 1 for <=256B parts, else 2
    uint32_t writeCycleMs = 5;
};

class AT24CxxNvStore : public INvStore {
public:
    AT24CxxNvStore(II2CBus& bus, IDelayProvider& delay, const AT24CxxConfig& cfg = {});

    bool read(uint32_t address, uint8_t* buf, size_t len) override;
    bool write(uint32_t address, const uint8_t* data, size_t len) override;
    size_t capacity() const override { return cfg_.totalBytes; }

private:
    II2CBus& bus_;
    IDelayProvider& delay_;
    AT24CxxConfig cfg_;

    bool inRange(uint32_t address, size_t len) const;
    size_t fillAddressBytes(uint32_t address, uint8_t* out) const;
};

} // namespace sf
