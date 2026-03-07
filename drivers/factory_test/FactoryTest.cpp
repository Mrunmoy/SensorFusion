#include "FactoryTest.hpp"
#include <cstdio>
#include <cstring>

namespace sf {

namespace {
const char* statusToStr(TestStatus status) {
    switch (status) {
        case TestStatus::PASS: return "PASS";
        case TestStatus::FAIL: return "FAIL";
        case TestStatus::SKIPPED: return "SKIPPED";
    }
    return "UNKNOWN";
}
} // namespace

bool FactoryTestRunner::addTest(IFactoryTest* test) {
    if (!test || count_ >= MAX_TESTS) return false;
    tests_[count_++] = test;
    return true;
}

size_t FactoryTestRunner::runAll(TestResult* results, size_t maxResults) {
    if (!results || maxResults == 0) return 0;
    passCount_ = 0;
    failCount_ = 0;
    skippedCount_ = 0;
    size_t ran = 0;

    for (size_t i = 0; i < count_ && ran < maxResults; ++i) {
        results[ran] = tests_[i]->run();
        if (results[ran].status == TestStatus::PASS)
            ++passCount_;
        else if (results[ran].status == TestStatus::FAIL)
            ++failCount_;
        else if (results[ran].status == TestStatus::SKIPPED)
            ++skippedCount_;
        ++ran;
    }
    return ran;
}

size_t FactoryTestRunner::runAll(TestReportCallback cb, void* ctx) {
    passCount_ = 0;
    failCount_ = 0;
    skippedCount_ = 0;

    for (size_t i = 0; i < count_; ++i) {
        TestResult r = tests_[i]->run();
        if (r.status == TestStatus::PASS)
            ++passCount_;
        else if (r.status == TestStatus::FAIL)
            ++failCount_;
        else if (r.status == TestStatus::SKIPPED)
            ++skippedCount_;
        if (cb) cb(r, ctx);
    }
    return count_;
}

size_t formatFactoryReportCsv(const TestResult* results, size_t count, char* out, size_t outSize) {
    if (!results || !out || outSize == 0) return 0;

    size_t written = 0;
    int n = std::snprintf(out, outSize, "name,status,detail\n");
    if (n <= 0) return 0;
    if (static_cast<size_t>(n) >= outSize) return 0;
    written = static_cast<size_t>(n);

    for (size_t i = 0; i < count; ++i) {
        const char* detail = results[i].detail ? results[i].detail : "";
        n = std::snprintf(out + written, outSize - written, "%s,%s,%s\n",
                          results[i].name ? results[i].name : "",
                          statusToStr(results[i].status),
                          detail);
        if (n <= 0) return 0;
        if (static_cast<size_t>(n) >= outSize - written) return 0;
        written += static_cast<size_t>(n);
    }
    return written;
}

} // namespace sf
