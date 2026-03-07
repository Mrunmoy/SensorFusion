#pragma once

#include "INvStore.hpp"
#include <cstddef>
#include <cstdint>

namespace sf {

struct StmNvStoreConfig {
    uint32_t baseAddress = 0;
    size_t sizeBytes = 0;
};

class StmNvStore : public INvStore {
public:
    explicit StmNvStore(const StmNvStoreConfig& cfg) : cfg_(cfg) {}

    bool read(uint32_t address, uint8_t* buf, size_t len) override;
    bool write(uint32_t address, const uint8_t* data, size_t len) override;
    size_t capacity() const override { return cfg_.sizeBytes; }

private:
    bool inRange(uint32_t address, size_t len) const;

    StmNvStoreConfig cfg_;
};

} // namespace sf
