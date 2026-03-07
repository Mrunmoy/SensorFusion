#pragma once

#include "INvStore.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace sf {

struct WearLevelingNvStoreConfig {
    uint32_t baseAddress = 0;
    size_t logicalPageSize = 32;
    size_t logicalPageCount = 4;
    size_t rotationDepth = 2;
};

class WearLevelingNvStore : public INvStore {
public:
    static constexpr uint32_t MAGIC = 0x574C4E56; // "WLNV"
    static constexpr size_t HEADER_SIZE = 16;

    WearLevelingNvStore(INvStore& rawStore, const WearLevelingNvStoreConfig& cfg = {});

    bool read(uint32_t address, uint8_t* buf, size_t len) override;
    bool write(uint32_t address, const uint8_t* data, size_t len) override;
    size_t capacity() const override { return cfg_.logicalPageSize * cfg_.logicalPageCount; }

private:
    struct PageMeta {
        bool hasValid = false;
        uint32_t seq = 0;
        size_t slot = 0;
    };

    INvStore& raw_;
    WearLevelingNvStoreConfig cfg_;
    std::vector<PageMeta> meta_;
    bool scanned_ = false;

    size_t recordSize() const { return HEADER_SIZE + cfg_.logicalPageSize; }
    bool inRange(uint32_t address, size_t len) const;
    bool ensureScanned();
    bool scanAllPages();
    bool scanPage(size_t pageIdx);
    uint32_t slotAddress(size_t pageIdx, size_t slotIdx) const;

    bool readPage(size_t pageIdx, uint8_t* outPage);
    bool writePage(size_t pageIdx, const uint8_t* inPage);
    bool readSlot(size_t pageIdx, size_t slotIdx, uint32_t& seq, uint8_t* payload);

    static uint32_t crc32(const uint8_t* data, size_t len);
    static void writeU32Le(uint8_t* dst, uint32_t value);
    static uint32_t readU32Le(const uint8_t* src);
};

} // namespace sf
