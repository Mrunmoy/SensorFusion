#include "NrfFdsStore.hpp"
#include "NrfSdk.hpp"
#include <cstring>

namespace sf {

NrfFdsStore::NrfFdsStore(uint16_t fileId, uint16_t recordKey, size_t capacityBytes)
    : fileId_(fileId), key_(recordKey), cache_(capacityBytes, 0xFF)
{
    fds_init();
    loadFromFds();
}

bool NrfFdsStore::inRange(uint32_t address, size_t len) const {
    return len <= cache_.size() && address <= cache_.size() - len;
}

bool NrfFdsStore::loadFromFds() {
    fds_record_desc_t desc = {};
    fds_find_token_t tok = {};
    if (fds_record_find(fileId_, key_, &desc, &tok) != FDS_SUCCESS) {
        return flushToFds();
    }

    fds_flash_record_t rec = {};
    if (fds_record_open(&desc, &rec) != FDS_SUCCESS) return false;

    const size_t bytes = rec.p_header->length_words * sizeof(uint32_t);
    const size_t copyBytes = bytes < cache_.size() ? bytes : cache_.size();
    std::memcpy(cache_.data(), rec.p_data, copyBytes);
    fds_record_close(&desc);
    return true;
}

bool NrfFdsStore::flushToFds() {
    fds_record_t rec = {};
    rec.file_id = fileId_;
    rec.key = key_;
    rec.data.p_data = cache_.data();
    rec.data.length_words = static_cast<uint16_t>((cache_.size() + 3u) / 4u);

    fds_record_desc_t desc = {};
    fds_find_token_t tok = {};
    if (fds_record_find(fileId_, key_, &desc, &tok) == FDS_SUCCESS) {
        return fds_record_update(&desc, &rec) == FDS_SUCCESS;
    }

    return fds_record_write(&desc, &rec) == FDS_SUCCESS;
}

bool NrfFdsStore::read(uint32_t address, uint8_t* buf, size_t len) {
    if (!buf || len == 0) return false;
    if (!inRange(address, len)) return false;
    std::memcpy(buf, cache_.data() + address, len);
    return true;
}

bool NrfFdsStore::write(uint32_t address, const uint8_t* data, size_t len) {
    if (!data || len == 0) return false;
    if (!inRange(address, len)) return false;
    std::memcpy(cache_.data() + address, data, len);
    return flushToFds();
}

} // namespace sf
