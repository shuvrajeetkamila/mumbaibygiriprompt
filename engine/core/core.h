// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/core.h
//  Common types, diagnostics and small utilities shared by every subsystem.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>
#include <array>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <limits>
#include <chrono>
#include <filesystem>

namespace mumbai {

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using i8  = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;
using f32 = float;
using f64 = double;

template <typename T> inline constexpr T clamp(T v, T lo, T hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
template <typename T> inline constexpr T lerp(T a, T b, T t) { return a + (b - a) * t; }
template <typename T> inline constexpr T smoothstep(T t) { return t * t * (3 - 2 * t); }
template <typename T> inline constexpr T sign(T v) { return v < T(0) ? T(-1) : T(1); }

inline constexpr f64 PI   = 3.14159265358979323846;
inline constexpr f64 TAU  = 6.28318530717958647692;
inline constexpr f64 DEG2RAD = PI / 180.0;
inline constexpr f64 RAD2DEG = 180.0 / PI;

inline bool isFinite(f32 v) { return std::isfinite(v); }
inline bool isFinite(f64 v) { return std::isfinite(v); }

// -----------------------------------------------------------------------------
//  Logging. Every subsystem reports through this so that pipeline runs and
//  engine validation runs leave an auditable trail (spec: "document every
//  decision", Section 7 completion criteria).
// -----------------------------------------------------------------------------
enum class LogLevel { Trace = 0, Debug, Info, Warn, Error };

void logSetLevel(LogLevel lvl);
LogLevel logGetLevel();
void logMessage(LogLevel lvl, const char* fmt, ...);

#define MLOG_TRACE(...) ::mumbai::logMessage(::mumbai::LogLevel::Trace, __VA_ARGS__)
#define MLOG_DEBUG(...) ::mumbai::logMessage(::mumbai::LogLevel::Debug, __VA_ARGS__)
#define MLOG_INFO(...)  ::mumbai::logMessage(::mumbai::LogLevel::Info,  __VA_ARGS__)
#define MLOG_WARN(...)  ::mumbai::logMessage(::mumbai::LogLevel::Warn,  __VA_ARGS__)
#define MLOG_ERR(...)   ::mumbai::logMessage(::mumbai::LogLevel::Error, __VA_ARGS__)

// -----------------------------------------------------------------------------
//  Hard assertions. In the pipeline a violated invariant is a data bug and must
//  fail loudly rather than propagate silently into the shipped world.
// -----------------------------------------------------------------------------
#define MCHECK(cond, ...)                                                      \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ::mumbai::logMessage(::mumbai::LogLevel::Error,                    \
                "MCHECK failed: %s  (%s:%d)", #cond, __FILE__, __LINE__);      \
            ::mumbai::logMessage(::mumbai::LogLevel::Error, __VA_ARGS__);      \
            std::abort();                                                      \
        }                                                                      \
    } while (0)

// -----------------------------------------------------------------------------
//  FNV-1a 64 hashing — used for asset ids, manifest integrity and deterministic
//  procedural variation seeds (rule-based generation must be reproducible).
// -----------------------------------------------------------------------------
inline u64 fnv1a64(std::string_view s, u64 seed = 1469598103934665603ull) {
    u64 h = seed;
    for (char c : s) { h ^= static_cast<u8>(c); h *= 1099511628211ull; }
    return h;
}
inline u64 fnv1a64(u64 v, u64 seed = 1469598103934665603ull) {
    u64 h = seed;
    for (int i = 0; i < 8; ++i) { h ^= (v >> (i * 8)) & 0xFFu; h *= 1099511628211ull; }
    return h;
}

// Deterministic PRNG (splitmix64). Procedural content must be identical across
// pipeline runs and across machines, so nothing here may use std::rand or time.
struct Rng {
    u64 s;
    explicit Rng(u64 seed = 0x9E3779B97F4A7C15ull) : s(seed ? seed : 1) {}
    u64 next() {
        s += 0x9E3779B97F4A7C15ull;
        u64 z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    u32 nextU32() { return static_cast<u32>(next() >> 32); }
    f32 nextF32() { return static_cast<f32>(next() >> 40) * (1.0f / 16777216.0f); }
    f32 range(f32 lo, f32 hi) { return lo + (hi - lo) * nextF32(); }
    i32 rangeI(i32 lo, i32 hi) { return lo + static_cast<i32>(next() % static_cast<u64>(hi - lo + 1)); }
    bool chance(f32 p) { return nextF32() < p; }
};

// -----------------------------------------------------------------------------
//  Timing helpers for the performance budget reports (Section 6B).
// -----------------------------------------------------------------------------
class ScopedTimer {
public:
    explicit ScopedTimer(const char* label) : m_label(label), m_start(now()) {}
    ~ScopedTimer();
    f64 seconds() const { return now() - m_start; }
    static f64 now();
private:
    const char* m_label;
    f64 m_start;
};

// -----------------------------------------------------------------------------
//  File / path helpers (thin wrappers so the pipeline and runtime share policy).
// -----------------------------------------------------------------------------
std::string readFileToString(const std::string& path);
bool writeFile(const std::string& path, const void* data, size_t bytes);
inline bool writeFile(const std::string& path, const std::string& s) {
    return writeFile(path, s.data(), s.size());
}
std::string pathJoin(const std::string& a, const std::string& b);
void ensureDir(const std::string& path);
u64 fileSize(const std::string& path);

// -----------------------------------------------------------------------------
//  Simple CLI parsing shared by pipeline and engine binaries.
// -----------------------------------------------------------------------------
struct CliArgs {
    std::unordered_map<std::string, std::string> values;
    std::vector<std::string> flags;
    bool has(const std::string& key) const {
        return values.count(key) || std::find(flags.begin(), flags.end(), key) != flags.end();
    }
    std::string get(const std::string& key, const std::string& def = "") const {
        auto it = values.find(key);
        return it == values.end() ? def : it->second;
    }
    f64 getNum(const std::string& key, f64 def) const {
        auto it = values.find(key);
        return it == values.end() ? def : std::atof(it->second.c_str());
    }
};
CliArgs parseCli(int argc, char** argv);

} // namespace mumbai
