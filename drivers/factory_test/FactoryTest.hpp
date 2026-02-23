#pragma once

#include <cstdint>
#include <cstddef>

namespace sf {

enum class TestStatus : uint8_t {
    PASS    = 0,
    FAIL    = 1,
    SKIPPED = 2,
};

struct TestResult {
    const char* name;
    TestStatus  status;
    const char* detail;  // short failure reason or nullptr
};

class IFactoryTest {
public:
    virtual ~IFactoryTest() = default;
    virtual const char* name() const = 0;
    virtual TestResult run() = 0;
};

// Callback for reporting results one-by-one (serial output)
using TestReportCallback = void(*)(const TestResult& result, void* ctx);

class FactoryTestRunner {
public:
    static constexpr size_t MAX_TESTS = 16;

    bool addTest(IFactoryTest* test);
    size_t runAll(TestResult* results, size_t maxResults);
    size_t runAll(TestReportCallback cb, void* ctx);

    size_t testCount() const { return count_; }
    size_t passCount() const { return passCount_; }
    size_t failCount() const { return failCount_; }
    size_t skippedCount() const { return skippedCount_; }

private:
    IFactoryTest* tests_[MAX_TESTS]{};
    size_t count_ = 0;
    size_t passCount_ = 0;
    size_t failCount_ = 0;
    size_t skippedCount_ = 0;
};

} // namespace sf
