#include "WearLevelingNvStore.hpp"
#include <algorithm>
#include <cstring>

namespace sf {

WearLevelingNvStore::WearLevelingNvStore(INvStore& rawStore, const WearLevelingNvStoreConfig& cfg)
    : raw_(rawStore), cfg_(cfg), meta_(cfg.logicalPageCount)
{}

bool WearLevelingNvStore::inRange(uint32_t address, size_t len) const {
    const size_t cap = capacity();
    return len <= cap && address <= cap - len;
}

bool WearLevelingNvStore::ensureScanned() {
    if (scanned_) return true;
    scanned_ = scanAllPages();
    return scanned_;
}

bool WearLevelingNvStore::scanAllPages() {
    if (cfg_.logicalPageSize == 0 || cfg_.logicalPageCount == 0 || cfg_.rotationDepth == 0) return false;

    const size_t req = cfg_.baseAddress + cfg_.logicalPageCount * cfg_.rotationDepth * recordSize();
    if (raw_.capacity() < req) return false;

    for (size_t i = 0; i < cfg_.logicalPageCount; ++i) {
        if (!scanPage(i)) return false;
    }
    return true;
}

bool WearLevelingNvStore::scanPage(size_t pageIdx) {
    PageMeta best{};
    for (size_t slot = 0; slot < cfg_.rotationDepth; ++slot) {
        uint32_t seq = 0;
        std::vector<uint8_t> payload(cfg_.logicalPageSize);
        if (!readSlot(pageIdx, slot, seq, payload.data())) continue;

        if (!best.hasValid || seq > best.seq) {
            best.hasValid = true;
            best.seq = seq;
            best.slot = slot;
        }
    }
    meta_[pageIdx] = best;
    return true;
}

uint32_t WearLevelingNvStore::slotAddress(size_t pageIdx, size_t slotIdx) const {
    const size_t offset = (pageIdx * cfg_.rotationDepth + slotIdx) * recordSize();
    return cfg_.baseAddress + static_cast<uint32_t>(offset);
}

bool WearLevelingNvStore::readSlot(size_t pageIdx, size_t slotIdx, uint32_t& seq, uint8_t* payload) {
    std::vector<uint8_t> rec(recordSize(), 0xFF);
    if (!raw_.read(slotAddress(pageIdx, slotIdx), rec.data(), rec.size())) return false;

    const uint32_t magic = readU32Le(rec.data());
    const uint32_t storedSeq = readU32Le(rec.data() + 4);
    const uint32_t storedPage = readU32Le(rec.data() + 8);
    const uint32_t storedCrc = readU32Le(rec.data() + 12);
    if (magic != MAGIC) return false;
    if (storedPage != static_cast<uint32_t>(pageIdx)) return false;

    const uint8_t* srcPayload = rec.data() + HEADER_SIZE;
    const uint32_t computed = crc32(srcPayload, cfg_.logicalPageSize);
    if (computed != storedCrc) return false;

    std::memcpy(payload, srcPayload, cfg_.logicalPageSize);
    seq = storedSeq;
    return true;
}

bool WearLevelingNvStore::readPage(size_t pageIdx, uint8_t* outPage) {
    auto& m = meta_[pageIdx];
    if (!m.hasValid) {
        std::memset(outPage, 0xFF, cfg_.logicalPageSize);
        return true;
    }

    uint32_t seq = 0;
    if (readSlot(pageIdx, m.slot, seq, outPage)) return true;

    if (!scanPage(pageIdx)) return false;
    if (!meta_[pageIdx].hasValid) {
        std::memset(outPage, 0xFF, cfg_.logicalPageSize);
        return true;
    }

    return readSlot(pageIdx, meta_[pageIdx].slot, seq, outPage);
}

bool WearLevelingNvStore::writePage(size_t pageIdx, const uint8_t* inPage) {
    auto& m = meta_[pageIdx];
    const size_t nextSlot = m.hasValid ? ((m.slot + 1) % cfg_.rotationDepth) : 0;
    const uint32_t nextSeq = m.hasValid ? (m.seq + 1u) : 1u;

    std::vector<uint8_t> rec(recordSize(), 0xFF);
    writeU32Le(rec.data(), MAGIC);
    writeU32Le(rec.data() + 4, nextSeq);
    writeU32Le(rec.data() + 8, static_cast<uint32_t>(pageIdx));
    writeU32Le(rec.data() + 12, crc32(inPage, cfg_.logicalPageSize));
    std::memcpy(rec.data() + HEADER_SIZE, inPage, cfg_.logicalPageSize);

    if (!raw_.write(slotAddress(pageIdx, nextSlot), rec.data(), rec.size())) return false;

    m.hasValid = true;
    m.seq = nextSeq;
    m.slot = nextSlot;
    return true;
}

bool WearLevelingNvStore::read(uint32_t address, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    if (!inRange(address, len)) return false;
    if (!ensureScanned()) return false;

    while (len > 0) {
        const size_t pageIdx = static_cast<size_t>(address) / cfg_.logicalPageSize;
        const size_t pageOffset = static_cast<size_t>(address) % cfg_.logicalPageSize;
        const size_t chunk = std::min(len, cfg_.logicalPageSize - pageOffset);

        std::vector<uint8_t> page(cfg_.logicalPageSize, 0xFF);
        if (!readPage(pageIdx, page.data())) return false;
        std::memcpy(buf, page.data() + pageOffset, chunk);

        address += static_cast<uint32_t>(chunk);
        buf += chunk;
        len -= chunk;
    }
    return true;
}

bool WearLevelingNvStore::write(uint32_t address, const uint8_t* data, size_t len) {
    if (!data || len == 0) return false;
    if (!inRange(address, len)) return false;
    if (!ensureScanned()) return false;

    while (len > 0) {
        const size_t pageIdx = static_cast<size_t>(address) / cfg_.logicalPageSize;
        const size_t pageOffset = static_cast<size_t>(address) % cfg_.logicalPageSize;
        const size_t chunk = std::min(len, cfg_.logicalPageSize - pageOffset);

        std::vector<uint8_t> page(cfg_.logicalPageSize, 0xFF);
        if (!readPage(pageIdx, page.data())) return false;
        std::memcpy(page.data() + pageOffset, data, chunk);
        if (!writePage(pageIdx, page.data())) return false;

        address += static_cast<uint32_t>(chunk);
        data += chunk;
        len -= chunk;
    }
    return true;
}

uint32_t WearLevelingNvStore::crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? ((crc >> 1u) ^ 0xEDB88320u) : (crc >> 1u);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

void WearLevelingNvStore::writeU32Le(uint8_t* dst, uint32_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFFu);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    dst[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    dst[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

uint32_t WearLevelingNvStore::readU32Le(const uint8_t* src) {
    return static_cast<uint32_t>(src[0]) |
           (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) |
           (static_cast<uint32_t>(src[3]) << 24);
}

} // namespace sf
