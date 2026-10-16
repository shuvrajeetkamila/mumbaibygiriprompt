// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/core.cpp
// =============================================================================
#include "engine/core/core.h"

#include <cstdarg>
#include <cstdio>
#include <ctime>

#if defined(__ANDROID__)
// An Android app with no Java has no console: stdout and stderr go to /dev/null
// and the only reader is logcat.  Routing the project's logger there is what makes
// a device log useful at all — without it the first APK reports nothing and every
// failure looks the same.  (This include was missing on the first CI run, which is
// the only place the #if branch is compiled: 5 errors, all "undeclared identifier"
// for macros that are declared in exactly this header.)
#include <android/log.h>
#endif

namespace mumbai {

static LogLevel g_level = LogLevel::Info;
static bool g_logTimestamps = false;

void logSetLevel(LogLevel lvl) { g_level = lvl; }
LogLevel logGetLevel() { return g_level; }

void logMessage(LogLevel lvl, const char* fmt, ...) {
    if (static_cast<int>(lvl) < static_cast<int>(g_level)) return;
#if defined(__ANDROID__)
    // One call, tag "mumbai", priority from the level: `adb logcat -s mumbai`.
    {
        va_list args;
        va_start(args, fmt);
        const int prio = (lvl == LogLevel::Error)   ? ANDROID_LOG_ERROR
                         : (lvl == LogLevel::Warn)  ? ANDROID_LOG_WARN
                         : (lvl == LogLevel::Info)  ? ANDROID_LOG_INFO
                                                    : ANDROID_LOG_DEBUG;
        __android_log_vprint(prio, "mumbai", fmt, args);
        va_end(args);
    }
    return;
#else
    const char* tag = "INFO ";
    switch (lvl) {
        case LogLevel::Trace: tag = "TRACE"; break;
        case LogLevel::Debug: tag = "DEBUG"; break;
        case LogLevel::Info:  tag = "INFO "; break;
        case LogLevel::Warn:  tag = "WARN "; break;
        case LogLevel::Error: tag = "ERROR"; break;
    }
    std::FILE* out = (lvl == LogLevel::Error || lvl == LogLevel::Warn) ? stderr : stdout;
    if (g_logTimestamps) {
        using clock = std::chrono::system_clock;
        const auto now = clock::now();
        const std::time_t t = clock::to_time_t(now);
        std::tm tm{};
        gmtime_r(&t, &tm);
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch()).count() % 1000;
        std::fprintf(out, "[%02d:%02d:%02d.%03d] ", tm.tm_hour, tm.tm_min, tm.tm_sec,
                     static_cast<int>(ms));
    }
    std::fprintf(out, "%s | ", tag);
    va_list args;
    va_start(args, fmt);
    std::vfprintf(out, fmt, args);
    va_end(args);
    std::fputc('\n', out);
    std::fflush(out);
#endif
}

// -----------------------------------------------------------------------------
f64 ScopedTimer::now() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<f64>(clock::now().time_since_epoch()).count();
}

ScopedTimer::~ScopedTimer() {
    MLOG_DEBUG("%s took %.3f s", m_label, seconds());
}

// -----------------------------------------------------------------------------
std::string readFileToString(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) { MLOG_ERR("cannot open %s", path.c_str()); return {}; }
    std::fseek(f, 0, SEEK_END);
    const long sz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::string out;
    if (sz > 0) {
        out.resize(static_cast<size_t>(sz));
        const size_t got = std::fread(out.data(), 1, static_cast<size_t>(sz), f);
        out.resize(got);
    }
    std::fclose(f);
    return out;
}

bool writeFile(const std::string& path, const void* data, size_t bytes) {
    ensureDir(std::filesystem::path(path).parent_path().string());
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) { MLOG_ERR("cannot write %s", path.c_str()); return false; }
    const size_t wrote = std::fwrite(data, 1, bytes, f);
    const bool ok = (wrote == bytes);
    if (std::fclose(f) != 0) return false;
    return ok;
}

std::string pathJoin(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (a.back() == '/') return a + b;
    return a + "/" + b;
}

void ensureDir(const std::string& path) {
    if (path.empty()) return;
    std::error_code ec;
    std::filesystem::create_directories(path, ec);
}

u64 fileSize(const std::string& path) {
    std::error_code ec;
    const auto sz = std::filesystem::file_size(path, ec);
    return ec ? 0 : static_cast<u64>(sz);
}

CliArgs parseCli(int argc, char** argv) {
    CliArgs a;
    for (int i = 1; i < argc; ++i) {
        std::string tok = argv[i];
        if (tok.rfind("--", 0) == 0) {
            const std::string key = tok;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                a.values[key] = argv[++i];
            } else {
                a.flags.push_back(key);
            }
        } else {
            a.flags.push_back(tok);
        }
    }
    return a;
}

} // namespace mumbai
