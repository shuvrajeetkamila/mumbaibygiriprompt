// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/world_download.cpp
//
//  The state machine behind "get the rest of the city": checksum list, then the
//  archive, streamed and verified, pausable at file boundaries.  See the header
//  for why the transport is an interface and why resume is at boundaries.
// =============================================================================
#include "engine/world/world_download.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>

namespace mumbai::world {
namespace {

// The checksum list for the whole city is about 100 KiB; the cap is here so that a
// server answering with a stream of nonsense cannot exhaust a phone's memory.
constexpr u64 kMaxChecksumBytes = 8u << 20;

constexpr const char* kResumeFile = ".download";

}  // namespace

WorldDownload::WorldDownload(Fetcher* fetcher, DownloadConfig config)
    : m_fetcher(fetcher), m_config(std::move(config)) {
    // Read before the first step, so the panel can say "resuming at 120 MiB" while
    // the checksum list is still in flight.
    m_boundary = loadBoundary();
    m_nextOffset = m_boundary;
}

WorldDownload::~WorldDownload() = default;

std::string worldReleaseUrl(const std::string& repository, const std::string& tag,
                            const std::string& asset) {
    return "https://github.com/" + repository + "/releases/download/" + tag + "/" + asset;
}

DownloadConfig configForRelease(const WorldRelease& release, const std::string& worldDir) {
    DownloadConfig config;
    config.worldDir = worldDir;
    config.manifestSha16 = release.manifestSha16;
    config.baseUrl = worldReleaseUrl(release.repository, release.tag, "");
    if (config.baseUrl.size() > 1 && config.baseUrl.back() == '/') config.baseUrl.pop_back();
    config.checksumsUrl = worldReleaseUrl(release.repository, release.tag, release.checksums);
    config.archiveUrl = worldReleaseUrl(release.repository, release.tag, release.asset);
    return config;
}

void WorldDownload::restart() {
    m_reader.reset();
    if (m_sink) m_sink->abortCurrent();
    m_sink.reset();
    saveBoundary(0);
    m_nextOffset = 0;
    m_attemptStart = 0;
    m_attemptBytes = 0;
    // A `.part` from before this restart is a prefix of a stream nobody kept, and
    // `ExtractingSink` starts every file afresh.
    world::removePartialFiles(m_config.worldDir);
}

void WorldDownload::step(u64 budgetBytes) {
    if (m_state == State::Done || m_state == State::Failed) return;
    if (!m_fetcher) {
        m_state = State::Failed;
        m_error = "no transport";
        return;
    }
    if (m_state == State::Idle) m_state = State::Checksums;

    if (m_state == State::Checksums) {
        if (!fetchChecksums()) return;
        m_state = State::Archive;
    }
    if (m_state == State::Archive) fetchArchive(budgetBytes);
}

bool WorldDownload::fetchChecksums() {
    m_checksums.clear();
    u64 total = 0;
    std::string error;
    const std::function<bool(const u8*, size_t)> collect = [&](const u8* bytes, size_t count) {
        if (m_checksums.size() + count > kMaxChecksumBytes) {
            error = "the checksum list is implausibly large";
            return false;
        }
        m_checksums.append(reinterpret_cast<const char*>(bytes), count);
        return true;
    };
    if (!m_fetcher->get(m_config.checksumsUrl, 0, collect, &total, &error)) {
        m_state = State::Failed;
        m_error = error.empty() ? std::string("could not fetch the checksum list") : error;
        return false;
    }
    if (!world::parseChecksumList(m_checksums, &m_expected)) {
        m_state = State::Failed;
        m_error = "the checksum list could not be read";
        return false;
    }
    // PRE-FLIGHT PROVENANCE, BEFORE ANY BYTE OF THE ARCHIVE IS ASKED FOR.  Every
    // build knows the sha256 of the world manifest it was made for, and the
    // published list names every file in the archive including that manifest.  If
    // the release has moved on to a different world, this notices now instead of
    // after 1.3 GB has been written into the install directory.
    if (m_config.manifestSha16.size() == 16) {
        const auto manifest = m_expected.find(world::ExtractingSink::kManifestName);
        if (manifest == m_expected.end()) {
            m_state = State::Failed;
            m_error = "the published world has no manifest";
            return false;
        }
        if (manifest->second.substr(0, 16) != m_config.manifestSha16) {
            m_state = State::Failed;
            m_error = "the published world is not the one this build expects";
            return false;
        }
    }
    return true;
}

bool WorldDownload::fetchArchive(u64 budgetBytes) {
    // `warm` means this process is already mid-stream: the parser holds the header of
    // the entry being written and the sink holds its open part file and its hash, so
    // the next bytes are simply the continuation.
    const bool warm = m_reader != nullptr && m_sink != nullptr;
    if (!m_sink) {
        // Cold: whatever a dead process left behind is not a prefix of this stream.
        world::removePartialFiles(m_config.worldDir);
        m_sink = std::make_unique<world::ExtractingSink>(m_config.worldDir, m_expected);
        m_sink->setError("");
    }
    if (!m_reader) {
        m_reader = std::make_unique<world::TarReader>(*m_sink);
        m_reader->onEntryComplete = [this](u64 offset) { saveBoundary(offset); };
    }

    m_attemptStart = m_nextOffset;
    m_attemptBytes = 0;
    m_paused = false;
    u64 fetched = 0;
    std::string error;
    u64 total = 0;
    const std::function<bool(const u8*, size_t)> sink = [&](const u8* bytes, size_t count) {
        // The budget is checked *between* chunks and never before the first one: a
        // step that accepted nothing would leave the download exactly where it was
        // and every later step would repeat it.  A chunk is the transport's unit, so
        // a step overshoots the budget by at most one of them.
        if (fetched > 0 && fetched >= budgetBytes) {
            m_paused = true;
            return false;   // "enough for this step"; not an error, see below
        }
        fetched += count;
        m_received += count;
        m_attemptBytes += count;
        if (!m_reader->feed(bytes, count)) {
            error = m_reader->error();
            return false;
        }
        return true;
    };

    const bool transportOk =
        m_fetcher->get(m_config.archiveUrl, m_attemptStart, sink, &total, &error);
    if (total > 0) m_total = total;
    m_filesWritten = m_sink->filesWritten();
    m_sawManifest = m_sink->sawManifest();

    if (m_paused) {
        // The budget ran out mid-entry and both the parser and the sink are holding
        // exactly where the stream stopped, so the next step continues *into the
        // file*.  This is the difference between a 57 MiB chunk taking 57 MiB of
        // transfer and it taking 57 MiB per step, for ever.  `m_boundary` stays
        // where it is: it is what a cold start would use, and it is only ever
        // advanced by a file that completed and was verified.
        (void)warm;
        m_nextOffset = m_attemptStart + m_attemptBytes;
        return true;
    }
    if (!transportOk) {
        if (m_fetcher->restartRequired()) {
            // A server that ignores Range hands back the whole file from zero, so
            // everything this attempt parsed came from a stream that has nothing to
            // do with the offset it asked for.  Start over: `restart()` forgets the
            // boundary and the part files, and the next step asks for offset 0, which
            // no server can refuse.
            MLOG_WARN("world download: the server ignored the byte range; restarting from zero");
            restart();
            return true;
        }
        m_state = State::Failed;
        if (error.empty()) error = "the download stopped before the archive ended";
        m_error = error;
        return false;
    }
    if (!m_reader->finish()) {
        m_state = State::Failed;
        m_error = m_reader->error().empty() ? "the archive ended early" : m_reader->error();
        return false;
    }
    if (!m_sink->sawManifest()) {
        m_state = State::Failed;
        m_error = "the archive carried no world manifest";
        return false;
    }
    if (m_config.manifestSha16.size() == 16 &&
        m_sink->manifestHex().substr(0, 16) != m_config.manifestSha16) {
        m_state = State::Failed;
        m_error = "the manifest is not the one this build expects";
        return false;
    }

    m_state = State::Done;
    m_nextOffset = 0;
    saveBoundary(0);   // a finished download has nothing to resume
    return true;
}

u64 WorldDownload::positionBytes() const {
    const u64 position = m_attemptStart + m_attemptBytes;
    if (m_total > 0 && position > m_total) return m_total;
    return position;
}

f64 WorldDownload::progress() const {
    if (m_state == State::Done) return 1.0;
    if (m_total == 0) return 0.0;
    // What the *archive position* is, not what has been pulled off the network: a
    // re-fetched entry would otherwise be counted twice and the bar would sit at
    // 100% while the job was still running (see `installer.bytesTotal`, D-047).
    const f64 done = static_cast<f64>(positionBytes());
    return std::clamp(done / static_cast<f64>(m_total), 0.0, 0.999);
}

std::string WorldDownload::status() const {
    switch (m_state) {
        case State::Idle:
            if (m_boundary > 0) {
                char resumed[128];
                std::snprintf(resumed, sizeof(resumed), "city: not started — resuming at %.0f MiB",
                              static_cast<f64>(m_boundary) / (1024.0 * 1024.0));
                return resumed;
            }
            return "city: not started";
        case State::Checksums:
            return "city: asking for the checksum list";
        case State::Done:
            return "city: complete — " + std::to_string(m_filesWritten) + " files verified";
        case State::Failed:
            return "city: failed — " + (m_error.empty() ? std::string("no reason given") : m_error);
        case State::Archive:
            break;
    }
    char buffer[192];
    const f64 mib = 1024.0 * 1024.0;
    if (m_total > 0) {
        std::snprintf(buffer, sizeof(buffer), "city: %.0f of %.0f MiB (%.0f%%)  %llu files",
                      static_cast<f64>(positionBytes()) / mib, static_cast<f64>(m_total) / mib,
                      progress() * 100.0, static_cast<unsigned long long>(m_filesWritten));
    } else {
        std::snprintf(buffer, sizeof(buffer), "city: %.0f MiB received  %llu files",
                      static_cast<f64>(positionBytes()) / mib,
                      static_cast<unsigned long long>(m_filesWritten));
    }
    return buffer;
}

void WorldDownload::saveBoundary(u64 offset) {
    m_boundary = offset;
    const std::string path = m_config.worldDir + "/" + kResumeFile;
    std::error_code ec;
    if (offset == 0) {
        std::filesystem::remove(path, ec);
        return;
    }
    std::filesystem::create_directories(m_config.worldDir, ec);
    if (std::FILE* file = std::fopen(path.c_str(), "wb")) {
        std::fprintf(file, "%llu\n", static_cast<unsigned long long>(offset));
        std::fclose(file);
    }
}

u64 WorldDownload::loadBoundary() const {
    const std::string path = m_config.worldDir + "/" + kResumeFile;
    std::FILE* file = std::fopen(path.c_str(), "rb");
    if (!file) return 0;
    unsigned long long offset = 0;
    const int read = std::fscanf(file, "%llu", &offset);
    std::fclose(file);
    if (read != 1) return 0;
    return static_cast<u64>(offset);
}

}  // namespace mumbai::world
