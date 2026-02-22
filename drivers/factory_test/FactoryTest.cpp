#include "FactoryTest.hpp"

namespace sf {

bool FactoryTestRunner::addTest(IFactoryTest* test) {
    if (!test || count_ >= MAX_TESTS) return false;
    tests_[count_++] = test;
    return true;
}

size_t FactoryTestRunner::runAll(TestResult* results, size_t maxResults) {
    passCount_ = 0;
    failCount_ = 0;
    size_t ran = 0;

    for (size_t i = 0; i < count_ && ran < maxResults; ++i) {
        results[ran] = tests_[i]->run();
        if (results[ran].status == TestStatus::PASS)
            ++passCount_;
        else if (results[ran].status == TestStatus::FAIL)
            ++failCount_;
        ++ran;
    }
    return ran;
}

size_t FactoryTestRunner::runAll(TestReportCallback cb, void* ctx) {
    passCount_ = 0;
    failCount_ = 0;

    for (size_t i = 0; i < count_; ++i) {
        TestResult r = tests_[i]->run();
        if (r.status == TestStatus::PASS)
            ++passCount_;
        else if (r.status == TestStatus::FAIL)
            ++failCount_;
        if (cb) cb(r, ctx);
    }
    return count_;
}

} // namespace sf
