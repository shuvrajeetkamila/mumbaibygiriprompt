// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_harness.h
//
//  A dependency-free test harness.  The project has no third-party C++
//  dependencies by policy (decision D-014), so the suite brings its own
//  registrar, checks and reporting rather than pulling in a framework.
// =============================================================================
#pragma once
#include <cmath>
#include <type_traits>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace mumbai::test {

struct Case {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<Case>& cases() {
    static std::vector<Case> c;
    return c;
}
inline int& failureCount() {
    static int f = 0;
    return f;
}
inline int& checkCount() {
    static int c = 0;
    return c;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { cases().push_back({name, std::move(fn)}); }
};

inline void reportFailure(const char* file, int line, const std::string& what) {
    ++failureCount();
    std::printf("    FAIL %s:%d  %s\n", file, line, what.c_str());
}

// Renders a value for diagnostics without requiring std::to_string support:
// strings pass through, arithmetic types print numerically.
template <typename T>
std::string describe(const T& v) {
    if constexpr (std::is_same_v<T, std::string> || std::is_same_v<T, const char*>) return std::string(v);
    else if constexpr (std::is_floating_point_v<T>) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.10g", static_cast<double>(v));
        return buf;
    } else if constexpr (std::is_integral_v<T>) {
        return std::to_string(static_cast<long long>(v));
    } else return "<?>";
}

template <typename A, typename B>
void checkEqual(const char* file, int line, const char* expr, const A& a, const B& b) {
    ++checkCount();
    if (!(a == b)) reportFailure(file, line, std::string(expr) + "  (" + describe(a) + " != " + describe(b) + ")");
}

inline void checkNear(const char* file, int line, const char* expr, double a, double b, double eps) {
    ++checkCount();
    if (!(std::fabs(a - b) <= eps)) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s  (%.12g vs %.12g, tol %.3g, err %.3g)", expr, a, b, eps,
                      std::fabs(a - b));
        reportFailure(file, line, buf);
    }
}

inline void checkTrue(const char* file, int line, const char* expr, bool cond) {
    ++checkCount();
    if (!cond) reportFailure(file, line, std::string(expr) + "  (false)");
}

}  // namespace mumbai::test

#define TEST(name)                                                             \
    static void name();                                                        \
    static ::mumbai::test::Registrar reg_##name(#name, name);                   \
    static void name()

#define CHECK(cond) ::mumbai::test::checkTrue(__FILE__, __LINE__, #cond, static_cast<bool>(cond))
#define CHECK_EQ(a, b) ::mumbai::test::checkEqual(__FILE__, __LINE__, #a " == " #b, (a), (b))
#define CHECK_NEAR(a, b, eps) ::mumbai::test::checkNear(__FILE__, __LINE__, #a " ~= " #b, (a), (b), (eps))
#define CHECK_FALSE(cond) CHECK(!(cond))
