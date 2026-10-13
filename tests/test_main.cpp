// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_main.cpp
//  Runs every registered case and reports.  Exit code 0 only when all pass.
// =============================================================================
#include "tests/test_harness.h"

int main(int argc, char** argv) {
    const std::string filter = argc > 1 ? argv[1] : "";
    int ran = 0, failedCases = 0;
    std::printf("=======================================================================\n");
    std::printf(" MUMBAI 1:1 EXACT — test suite (%zu cases)\n", mumbai::test::cases().size());
    std::printf("=======================================================================\n");
    for (const auto& c : mumbai::test::cases()) {
        if (!filter.empty() && c.name.find(filter) == std::string::npos) continue;
        const int before = mumbai::test::failureCount();
        std::printf("  %-46s", c.name.c_str());
        std::fflush(stdout);
        c.fn();
        const bool ok = mumbai::test::failureCount() == before;
        std::printf("%s\n", ok ? "ok" : "FAILED");
        ran++;
        if (!ok) failedCases++;
    }
    std::printf("-----------------------------------------------------------------------\n");
    std::printf(" %d cases, %d checks, %d failing cases — %s\n", ran, mumbai::test::checkCount(),
                failedCases, failedCases == 0 ? "PASS" : "FAIL");
    std::printf("=======================================================================\n");
    return failedCases == 0 ? 0 : 1;
}
