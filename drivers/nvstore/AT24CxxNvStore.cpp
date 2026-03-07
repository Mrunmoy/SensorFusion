#include "AT24CxxNvStore.hpp"
#include <algorithm>
#include <cstring>

namespace sf {

AT24CxxNvStore::AT24CxxNvStore(II2CBus& bus, IDelayProvider& delay, const AT24CxxConfig& cfg)
    : bus_(bus), delay_(delay), cfg_(cfg)
{}

bool AT24CxxNvStore::inRange(uint32_t address, size_t len) const {
    return len <= cfg_.totalBytes &&
           address <= cfg_.totalBytes - len;
}

size_t AT24CxxNvStore::fillAddressBytes(uint32_t address, uint8_t* out) const {
    if (cfg_.addressWidth == 1) {
        out[0] = static_cast<uint8_t>(address & 0xFF);
        return 1;
    }
    out[0] = static_cast<uint8_t>((address >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(address & 0xFF);
    return 2;
}

bool AT24CxxNvStore::read(uint32_t address, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    if (!inRange(address, len)) return false;

    while (len > 0) {
        const size_t chunk = std::min<size_t>(len, 32);

        uint8_t addrBuf[2] = {};
        const size_t addrLen = fillAddressBytes(address, addrBuf);
        if (!bus_.rawWrite(cfg_.i2cAddress, addrBuf, addrLen)) return false;
        if (!bus_.rawRead(cfg_.i2cAddress, buf, chunk)) return false;

        address += static_cast<uint32_t>(chunk);
        buf += chunk;
        len -= chunk;
    }
    return true;
}

bool AT24CxxNvStore::write(uint32_t address, const uint8_t* data, size_t len) {
    if (!data || len == 0) return false;
    if (!inRange(address, len)) return false;
    if (cfg_.pageSize == 0) return false;

    while (len > 0) {
        const size_t pageOffset = static_cast<size_t>(address) % cfg_.pageSize;
        const size_t chunk = std::min(len, cfg_.pageSize - pageOffset);

        uint8_t payload[2 + 256] = {};
        const size_t addrLen = fillAddressBytes(address, payload);
        std::memcpy(payload + addrLen, data, chunk);

        if (!bus_.rawWrite(cfg_.i2cAddress, payload, addrLen + chunk)) return false;
        delay_.delayMs(cfg_.writeCycleMs);

        address += static_cast<uint32_t>(chunk);
        data += chunk;
        len -= chunk;
    }
    return true;
}

} // namespace sf
