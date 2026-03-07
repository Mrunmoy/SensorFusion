#include "StmNvStore.hpp"
#include "StmHal.hpp"

namespace sf {

bool StmNvStore::inRange(uint32_t address, size_t len) const {
    return len <= cfg_.sizeBytes && address <= cfg_.sizeBytes - len;
}

bool StmNvStore::read(uint32_t address, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    if (!inRange(address, len)) return false;

    const uint8_t* src = reinterpret_cast<const uint8_t*>(cfg_.baseAddress + address);
    for (size_t i = 0; i < len; ++i) {
        buf[i] = src[i];
    }
    return true;
}

bool StmNvStore::write(uint32_t address, const uint8_t* data, size_t len) {
    if (!data || len == 0) return false;
    if (!inRange(address, len)) return false;

    if (HAL_FLASH_Unlock() != HAL_OK) return false;

    uint32_t dest = cfg_.baseAddress + address;
#ifdef FLASH_TYPEPROGRAM_BYTE
    for (size_t i = 0; i < len; ++i) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, dest + i, data[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
#else
    HAL_FLASH_Lock();
    (void)dest;
    (void)data;
    (void)len;
    return false;
#endif

    HAL_FLASH_Lock();
    return true;
}

} // namespace sf
