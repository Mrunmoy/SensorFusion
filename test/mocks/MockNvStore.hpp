#pragma once

#include "INvStore.hpp"
#include <gmock/gmock.h>
#include <cstring>
#include <vector>

namespace sf { namespace test {

class MockNvStore : public INvStore {
public:
    MOCK_METHOD(bool, read, (uint32_t address, uint8_t* buf, size_t len), (override));
    MOCK_METHOD(bool, write, (uint32_t address, const uint8_t* data, size_t len), (override));
    MOCK_METHOD(size_t, capacity, (), (const, override));

    // Helper: simulate a backing store for round-trip tests
    void useBackingStore(size_t size) {
        store_.resize(size, 0xFF);
        using ::testing::Invoke;
        ON_CALL(*this, read).WillByDefault(
            [this](uint32_t addr, uint8_t* buf, size_t len) -> bool {
                if (addr + len > store_.size()) return false;
                std::memcpy(buf, store_.data() + addr, len);
                return true;
            });
        ON_CALL(*this, write).WillByDefault(
            [this](uint32_t addr, const uint8_t* data, size_t len) -> bool {
                if (addr + len > store_.size()) return false;
                std::memcpy(store_.data() + addr, data, len);
                return true;
            });
        ON_CALL(*this, capacity).WillByDefault(
            [this]() -> size_t { return store_.size(); });
    }

private:
    std::vector<uint8_t> store_;
};

}} // namespace sf::test
