// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_world_download.cpp
//
//  The download state machine, driven by a transport that serves an archive out of
//  memory: a whole download, a paced one, one that is interrupted and resumed by a
//  *new* downloader (which is what an app restart is), a transport that dies, and
//  a world whose bytes are not the ones this build expects.
//
//  No phone, no network, no TLS: which is the point of the transport being an
//  interface.  What is tested here is everything except the sockets.
// =============================================================================
#include "engine/world/world_download.h"
#include "engine/core/sha256.h"
#include "tests/test_harness.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace mumbai;
using namespace mumbai::world;
using mumbai::world::DownloadConfig;
using mumbai::world::WorldRelease;

namespace {

constexpr size_t kBlock = 512;

void putOctal(u8* field, size_t length, u64 value) {
    std::string text;
    do {
        text.insert(text.begin(), static_cast<char>('0' + (value & 7u)));
        value >>= 3;
    } while (value != 0);
    while (text.size() + 1 < length) text.insert(text.begin(), '0');
    for (size_t i = 0; i < text.size() && i < length; ++i) field[i] = static_cast<u8>(text[i]);
    if (text.size() < length) field[text.size()] = 0;
}

void appendFile(std::vector<u8>& out, const std::string& name, const std::string& bytes) {
    const size_t base = out.size();
    out.resize(base + kBlock, 0);
    u8* block = out.data() + base;
    std::snprintf(reinterpret_cast<char*>(block), 100, "%s", name.c_str());
    putOctal(block + 100, 8, 0644);
    putOctal(block + 124, 12, bytes.size());
    for (size_t i = 148; i < 156; ++i) block[i] = ' ';
    block[156] = '0';
    std::snprintf(reinterpret_cast<char*>(block + 257), 6, "ustar");
    block[263] = '0';
    block[264] = '0';
    u64 sum = 0;
    for (size_t i = 0; i < kBlock; ++i) sum += block[i];
    std::string text;
    do {
        text.insert(text.begin(), static_cast<char>('0' + (sum & 7u)));
        sum >>= 3;
    } while (sum != 0);
    while (text.size() < 6) text.insert(text.begin(), '0');
    for (size_t i = 0; i < 6; ++i) block[148 + i] = static_cast<u8>(text[i]);
    block[154] = 0;
    block[155] = ' ';

    const size_t data = out.size();
    out.resize(data + bytes.size());
    std::memcpy(out.data() + data, bytes.data(), bytes.size());
    while (out.size() % kBlock != 0) out.push_back(0);
}

// A world small enough to hold in memory and large enough that a step's budget has
// to be spent: three cells and a manifest.
struct Fixture {
    std::string manifest;
    std::vector<std::pair<std::string, std::string>> files;
    std::string checksums;
    std::vector<u8> archive;

    Fixture() {
        manifest = "{\"cells\":3}";
        files.push_back({"cells/cell_0001_0001.mumbai_chunk", std::string(9000, 'A')});
        files.push_back({"cells/cell_0001_0002.mumbai_chunk", std::string(7000, 'B')});
        files.push_back({"world_manifest.json", manifest});
        for (const auto& file : files) {
            checksums += sha256Hex(file.second.data(), file.second.size()) + "  " + file.first + "\n";
        }
        for (const auto& file : files) appendFile(archive, file.first, file.second);
        archive.resize(archive.size() + 2 * kBlock, 0);
    }

    std::string manifestSha16() const { return sha256Hex(manifest.data(), manifest.size()).substr(0, 16); }
};

// A transport that serves bytes from memory, one chunk per callback, with knobs
// for the ways a real one misbehaves.
class MemoryFetcher : public Fetcher {
  public:
    const Fixture* fixture = nullptr;
    std::string checksumsUrl = "https://example.invalid/world/checksums.sha256";
    std::string archiveUrl = "https://example.invalid/world/world.tar";
    u64 chunk = 4096;
    u64 dieAfterBytes = 0;      // 0 = never
    bool refuseRanges = false;  // answer 200 to a Range request, as a bad proxy does
    int calls = 0;
    u64 firstOffset = 0;        // the offset the first archive byte-range asked for
    std::vector<u64> archiveOffsets;   // every offset the archive was asked for

    bool sawArchiveCall = false;

    bool restartRequired() const override { return m_restartRequired; }

    // Public because this is a test fixture, not an implementation: the tests set it
    // and read it directly, and a getter for a getter is noise.
    bool m_restartRequired = false;

    bool get(const std::string& url, u64 offset,
             const std::function<bool(const u8*, size_t)>& onData, u64* totalBytes,
             std::string* error) override {
        ++calls;
        const std::string* text = nullptr;
        const std::vector<u8>* blob = nullptr;
        if (url == checksumsUrl) {
            text = &fixture->checksums;
        } else if (url == archiveUrl) {
            blob = &fixture->archive;
            if (!sawArchiveCall) {
                firstOffset = offset;
                sawArchiveCall = true;
            }
            archiveOffsets.push_back(offset);
            // A server (or proxy) that ignores the Range header and starts from zero:
            // it answers 200 with the whole file, which the transport notices before
            // reading a byte of the body, so nothing is handed to the parser.
            if (refuseRanges && offset > 0) {
                m_restartRequired = true;
                if (error) *error = "the server ignored the byte range";
                return false;
            }
        } else {
            if (error) *error = "no such resource: " + url;
            return false;
        }
        const size_t size = text ? text->size() : blob->size();
        if (totalBytes) *totalBytes = size;
        if (offset >= size) return true;
        size_t position = static_cast<size_t>(offset);
        u64 delivered = 0;
        while (position < size) {
            const size_t count = static_cast<size_t>(std::min<u64>(chunk, size - position));
            const u8* bytes = text ? reinterpret_cast<const u8*>(text->data()) + position
                                   : blob->data() + position;
            if (!onData(bytes, count)) return false;   // the caller paused
            position += count;
            delivered += count;
            if (dieAfterBytes && delivered >= dieAfterBytes) {
                if (error) *error = "the connection dropped";
                return false;
            }
        }
        return true;
    }
};

std::string freshWorldDir(const char* leaf) {
    const std::string dir = std::string("/tmp/mumbai_download_test_") + leaf;
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir, ec);
    return dir;
}

std::string readWhole(const std::string& path) {
    std::FILE* file = std::fopen(path.c_str(), "rb");
    if (!file) return "<missing>";
    std::string out;
    char buffer[8192];
    size_t got = 0;
    while ((got = std::fread(buffer, 1, sizeof(buffer), file)) > 0) out.append(buffer, got);
    std::fclose(file);
    return out;
}

u64 partCount(const std::string& dir) {
    u64 parts = 0;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::recursive_directory_iterator(dir)) {
        const std::string name = entry.path().filename().string();
        if (name.size() > 5 && name.compare(name.size() - 5, 5, ".part") == 0) ++parts;
    }
    return parts;
}

void plantBytes(const std::string& path, const std::string& text) {
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::FILE* file = std::fopen(path.c_str(), "wb");
    CHECK(file != nullptr);
    if (!file) return;
    std::fwrite(text.data(), 1, text.size(), file);
    std::fclose(file);
}

DownloadConfig configFor(const Fixture& fixture, const std::string& dir) {
    DownloadConfig config;
    config.checksumsUrl = "https://example.invalid/world/checksums.sha256";
    config.archiveUrl = "https://example.invalid/world/world.tar";
    config.manifestSha16 = fixture.manifestSha16();
    config.worldDir = dir;
    config.baseUrl = "https://example.invalid/world";
    return config;
}

}  // namespace

TEST(worldDownloadFetchesVerifiesAndCompletes) {
    const Fixture fixture;
    const std::string dir = freshWorldDir("complete");
    MemoryFetcher fetcher;
    fetcher.fixture = &fixture;

    WorldDownload download(&fetcher, configFor(fixture, dir));
    CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Idle));
    download.step(64u << 20);   // one generous step: the whole archive
    CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Done));
    CHECK(download.error().empty());
    CHECK_EQ(download.filesWritten(), 3u);
    CHECK(download.sawManifest());
    CHECK_EQ(readWhole(dir + "/cells/cell_0001_0001.mumbai_chunk"), fixture.files[0].second);
    CHECK_EQ(readWhole(dir + "/cells/cell_0001_0002.mumbai_chunk"), fixture.files[1].second);
    CHECK_EQ(readWhole(dir + "/world_manifest.json"), fixture.manifest);
    CHECK_EQ(download.progress(), 1.0);
    CHECK(download.status().find("complete") != std::string::npos);
    // A finished download leaves nothing to resume from.
    CHECK(!std::filesystem::exists(dir + "/.download"));
    CHECK_EQ(fetcher.calls, 2);   // the checksum list, then the archive

    // Asking again is a no-op: the state machine does not re-fetch a finished world.
    download.step(1u << 20);
    CHECK_EQ(fetcher.calls, 2);
}

TEST(worldDownloadIsPacedWithoutRefetchingAndResumesAfterARestart) {
    const Fixture fixture;
    const u64 archiveSize = fixture.archive.size();

    // 1. Paced, in-process: two small steps finish a download whose archive is
    //    bigger than either step's budget — and pull every byte exactly once.  A
    //    step that ends inside a 9 KiB entry must continue *into* that entry, not
    //    go back to the boundary it last passed, or the download would re-fetch the
    //    same prefix for ever (which is the shape of the real world: 57 MiB chunks
    //    against a per-frame budget).
    {
        const std::string dir = freshWorldDir("paced");
        MemoryFetcher fetcher;
        fetcher.fixture = &fixture;
        fetcher.chunk = 1024;
        WorldDownload download(&fetcher, configFor(fixture, dir));
        download.step(11000);
        CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Archive));
        CHECK(download.progress() > 0.0 && download.progress() < 1.0);
        CHECK(download.positionBytes() == 11264u);   // 11 whole chunks, then the budget
        CHECK_EQ(download.resumeBytes(), 9728u);     // the one boundary it has passed
        CHECK(std::filesystem::exists(dir + "/.download"));
        CHECK(!std::filesystem::exists(dir + "/cells/cell_0001_0002.mumbai_chunk"));
        download.step(11000);
        CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Done));
        CHECK_EQ(download.receivedBytes(), archiveSize);   // not one byte twice
        CHECK_EQ(readWhole(dir + "/cells/cell_0001_0001.mumbai_chunk"), fixture.files[0].second);
        CHECK_EQ(readWhole(dir + "/cells/cell_0001_0002.mumbai_chunk"), fixture.files[1].second);
        CHECK_EQ(readWhole(dir + "/world_manifest.json"), fixture.manifest);
        CHECK(download.status().find("complete") != std::string::npos);
        CHECK(!std::filesystem::exists(dir + "/.download"));
    }

    // 2. Cold start: a process that stopped one step in, and a fresh downloader on
    //    the same directory.  It must resume from the boundary the first one
    //    remembered, keep the file that was already verified, and not re-read the
    //    prefix before that boundary.
    {
        const std::string dir = freshWorldDir("cold");
        MemoryFetcher first;
        first.fixture = &fixture;
        first.chunk = 1024;
        {
            WorldDownload download(&first, configFor(fixture, dir));
            download.step(11000);
            CHECK_EQ(static_cast<int>(download.state()),
                     static_cast<int>(WorldDownload::State::Archive));
            CHECK_EQ(download.resumeBytes(), 9728u);
        }   // the process dies here, mid-entry, with a `.part` on disk

        MemoryFetcher second;
        second.fixture = &fixture;
        second.chunk = 1024;
        WorldDownload resumed(&second, configFor(fixture, dir));
        CHECK_EQ(resumed.resumeBytes(), 9728u);   // known before the first step
        CHECK(resumed.status().find("resuming at") != std::string::npos);
        u64 steps = 0;
        for (int i = 0; i < 64; ++i) {
            resumed.step(2048);
            ++steps;
            if (resumed.state() == WorldDownload::State::Done) break;
            CHECK_EQ(static_cast<int>(resumed.state()),
                     static_cast<int>(WorldDownload::State::Archive));
        }
        CHECK_EQ(static_cast<int>(resumed.state()), static_cast<int>(WorldDownload::State::Done));
        CHECK(resumed.error().empty());
        CHECK(steps > 1);
        // Exactly the bytes after the boundary were fetched again, and the whole
        // archive is on disk.
        CHECK_EQ(second.firstOffset, 9728u);
        CHECK_EQ(resumed.receivedBytes(), archiveSize - 9728u);
        CHECK_EQ(readWhole(dir + "/cells/cell_0001_0001.mumbai_chunk"), fixture.files[0].second);
        CHECK_EQ(readWhole(dir + "/cells/cell_0001_0002.mumbai_chunk"), fixture.files[1].second);
        CHECK_EQ(readWhole(dir + "/world_manifest.json"), fixture.manifest);
    }

    // 3. A transport that drops inside an entry, and what it leaves behind.  While
    //    the sink is alive its half-written entry is on disk as `<name>.part`; a
    //    clean shutdown takes it away; a process the OS killed does not, and the next
    //    run has to throw the leftovers out rather than carry them into the next
    //    attempt (they are not part of this download, and they are megabytes of a
    //    world nobody asked for).
    {
        const std::string dir = freshWorldDir("recover");
        MemoryFetcher broken;
        broken.fixture = &fixture;
        broken.chunk = 1024;
        broken.dieAfterBytes = 3000;
        {
            const std::unique_ptr<WorldDownload> download(
                new WorldDownload(&broken, configFor(fixture, dir)));
            download->step(1u << 20);
            CHECK_EQ(static_cast<int>(download->state()),
                     static_cast<int>(WorldDownload::State::Failed));
            CHECK(download->error().find("dropped") != std::string::npos);
            CHECK_EQ(partCount(dir), 1u);   // file 1, half written
        }
        CHECK_EQ(partCount(dir), 0u);   // ... and gone once the sink is destroyed

        // Now the dirty case: a part file and a stale resume point, as a killed
        // process leaves them.
        plantBytes(dir + "/cells/cell_0001_0001.mumbai_chunk.part", "junk from a dead process");
        CHECK_EQ(partCount(dir), 1u);

        MemoryFetcher good;
        good.fixture = &fixture;
        good.chunk = 1024;
        WorldDownload recovered(&good, configFor(fixture, dir));
        for (int i = 0; i < 64 && recovered.state() != WorldDownload::State::Done; ++i) {
            recovered.step(4096);
        }
        CHECK_EQ(static_cast<int>(recovered.state()), static_cast<int>(WorldDownload::State::Done));
        CHECK_EQ(partCount(dir), 0u);
        CHECK_EQ(readWhole(dir + "/cells/cell_0001_0001.mumbai_chunk"), fixture.files[0].second);
        CHECK_EQ(readWhole(dir + "/cells/cell_0001_0002.mumbai_chunk"), fixture.files[1].second);
        CHECK_EQ(readWhole(dir + "/world_manifest.json"), fixture.manifest);
    }
}

TEST(worldDownloadFailsLoudlyOnABrokenTransportOrAMismatchedWorld) {
    // 1. The transport dies in the middle of the archive.
    {
        const Fixture fixture;
        const std::string dir = freshWorldDir("dropped");
        MemoryFetcher fetcher;
        fetcher.fixture = &fixture;
        fetcher.chunk = 1024;
        fetcher.dieAfterBytes = 3000;
        WorldDownload download(&fetcher, configFor(fixture, dir));
        download.step(64u << 20);
        CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Failed));
        CHECK(!download.error().empty());
        CHECK(download.error().find("dropped") != std::string::npos);
        CHECK(download.status().find("failed") != std::string::npos);
        // A failed download does not pretend to be finished, and a later step does
        // not silently continue with a half-parsed stream.
        const u64 filesAtFailure = download.filesWritten();
        download.step(64u << 20);
        CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Failed));
        CHECK_EQ(download.filesWritten(), filesAtFailure);
    }

    // 2. The published list names a manifest this build was not made for: caught
    //    before a single byte of the archive is asked for, so a release that has
    //    moved on cannot leave a foreign world in the install directory.
    {
        const Fixture fixture;
        const std::string dir = freshWorldDir("wrongworld");
        MemoryFetcher fetcher;
        fetcher.fixture = &fixture;
        DownloadConfig config = configFor(fixture, dir);
        config.manifestSha16 = "0000000000000000";
        WorldDownload download(&fetcher, config);
        download.step(64u << 20);
        CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Failed));
        CHECK(download.error().find("not the one this build expects") != std::string::npos);
        CHECK_EQ(fetcher.calls, 1);   // the list only; the archive was never opened
        CHECK_EQ(download.positionBytes(), 0u);
    }

    // 2b. A list that names no manifest at all is refused the same way.
    {
        Fixture fixture;
        fixture.checksums.clear();
        for (const auto& file : fixture.files) {
            if (file.first == "world_manifest.json") continue;
            fixture.checksums += sha256Hex(file.second.data(), file.second.size()) + "  " +
                                 file.first + "\n";
        }
        const std::string dir = freshWorldDir("nomanifest");
        MemoryFetcher fetcher;
        fetcher.fixture = &fixture;
        WorldDownload download(&fetcher, configFor(fixture, dir));
        download.step(64u << 20);
        CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Failed));
        CHECK(download.error().find("no manifest") != std::string::npos);
    }

    // 3. A checksum list that cannot be parsed stops the download before a single
    //    byte of the archive is asked for.
    {
        Fixture fixture;
        fixture.checksums = "this is not a checksum list\n";
        const std::string dir = freshWorldDir("badlist");
        MemoryFetcher fetcher;
        fetcher.fixture = &fixture;
        WorldDownload download(&fetcher, configFor(fixture, dir));
        download.step(64u << 20);
        CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Failed));
        CHECK(download.error().find("checksum list") != std::string::npos);
        CHECK_EQ(fetcher.calls, 1);   // the list, and nothing else
    }

    // 4. Parsing the list accepts the three spellings `sha256sum` writes, including
    //    the binary-mode marker and a comment line.
    {
        std::map<std::string, std::string> parsed;
        const std::string hex(64, 'a');
        CHECK(world::parseChecksumList("# generated\n" + hex + "  cells/a.mumbai_chunk\n" + hex +
                                          " *cells/b.mumbai_chunk\n" + hex +
                                          " cells/c.mumbai_chunk\n",
                                      &parsed));
        CHECK_EQ(parsed.size(), 3u);
        CHECK(parsed.count("cells/a.mumbai_chunk") == 1);
        CHECK(parsed.count("cells/b.mumbai_chunk") == 1);
        CHECK(parsed.count("cells/c.mumbai_chunk") == 1);
    }
}

TEST(worldDownloadRestartsWhenTheTransportRefusesAByteRange) {
    // A proxy that ignores Range answers 200 with the whole file.  Appending those
    // bytes to a stream that was parsed from an offset is how a world gets built out
    // of the tail of one download and the head of another, so the downloader throws
    // its stream away and asks for offset zero again: one wasted round trip, and
    // every file on disk afterwards belongs to one download.
    const Fixture fixture;
    const std::string dir = freshWorldDir("norange");
    MemoryFetcher fetcher;
    fetcher.fixture = &fixture;
    fetcher.chunk = 1024;
    WorldDownload download(&fetcher, configFor(fixture, dir));

    download.step(11000);   // pauses mid-entry, past the first boundary
    CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Archive));
    CHECK(download.resumeBytes() > 0);

    fetcher.refuseRanges = true;   // from here the server will not honour a range
    download.step(11000);
    CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Archive));
    CHECK(download.error().empty());          // a restart is not a failure
    CHECK_EQ(download.resumeBytes(), 0u);     // the boundary is forgotten on purpose

    fetcher.refuseRanges = false;             // offset zero is servable by any server
    for (int i = 0; i < 64 && download.state() != world::WorldDownload::State::Done; ++i) {
        download.step(4096);
    }
    CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(world::WorldDownload::State::Done));
    CHECK_EQ(readWhole(dir + "/cells/cell_0001_0001.mumbai_chunk"), fixture.files[0].second);
    CHECK_EQ(readWhole(dir + "/cells/cell_0001_0002.mumbai_chunk"), fixture.files[1].second);
    // And the story the offsets tell: from zero, continued mid-entry, then from zero
    // again because the range was refused.
    CHECK(fetcher.archiveOffsets.size() >= 4u);
    CHECK_EQ(fetcher.archiveOffsets.front(), 0u);
    CHECK(fetcher.archiveOffsets[1] > 0u);            // the warm continuation
    CHECK_EQ(fetcher.archiveOffsets[2], 0u);          // the restart, and then
    CHECK(fetcher.archiveOffsets[3] > 0u);            // continue mid-entry again
    u64 zeros = 0;
    for (u64 offset : fetcher.archiveOffsets) {
        if (offset == 0) ++zeros;
    }
    CHECK_EQ(zeros, 2u);
}

TEST(worldReleaseDescribesAWholeDownload) {
    // This build's own release: a bare checkout knows of none, and CI's build (the
    // generated header) knows of one.  Both are checked for self-consistency, because
    // the strings a build compiles in have to name one world or no world.
    const world::WorldRelease release = world::publishedWorldRelease();
    if (release.known()) {
        CHECK_EQ(std::string(release.asset).rfind("mumbai-world-", 0), 0u);
        CHECK_EQ(std::string(release.checksums).rfind("sha256sums-", 0), 0u);
        CHECK(std::string(release.asset).find(release.manifestSha16) != std::string::npos);
        CHECK(std::string(release.checksums).find(release.manifestSha16) != std::string::npos);
        CHECK_EQ(std::string(release.manifestSha16).size(), 16u);
    } else {
        CHECK_EQ(std::string(release.tag), std::string("world-data"));
    }

    // A release composes the URLs the workflow publishes.
    const world::WorldRelease fabricated{"owner/repo", "world-data",
                                         "mumbai-world-0123456789abcdef.tar",
                                         "sha256sums-0123456789abcdef.txt", "0123456789abcdef"};
    CHECK(fabricated.known());
    const world::DownloadConfig config = world::configForRelease(fabricated, "/tmp/world");
    CHECK_EQ(config.archiveUrl,
             std::string("https://github.com/owner/repo/releases/download/world-data/"
                         "mumbai-world-0123456789abcdef.tar"));
    CHECK_EQ(config.checksumsUrl,
             std::string("https://github.com/owner/repo/releases/download/world-data/"
                         "sha256sums-0123456789abcdef.txt"));
    CHECK_EQ(config.baseUrl,
             std::string("https://github.com/owner/repo/releases/download/world-data"));
    CHECK_EQ(config.manifestSha16, std::string("0123456789abcdef"));
    CHECK_EQ(config.worldDir, std::string("/tmp/world"));

    // An empty release is not known, and that is the whole of the "no published
    // world" case: the client shows the sentence instead of trying a URL.
    const world::WorldRelease nothing{"", "world-data", "", "", ""};
    CHECK(!nothing.known());
}

TEST(worldReleaseUrlPointsAtThePublishedAsset) {
    const std::string url = worldReleaseUrl("owner/repo", "world-data",
                                           "mumbai-world-0123456789abcdef.tar");
    CHECK_EQ(url, std::string("https://github.com/owner/repo/releases/download/world-data/"
                              "mumbai-world-0123456789abcdef.tar"));
}

// =============================================================================
//  THE ARCHIVE THE WORKFLOW ACTUALLY PUBLISHES.
//
//  Everything above writes tar blocks by hand, which proves the parser against the
//  specification.  This case proves it against the *program*: it packs three of the
//  pipeline's own chunk files and the world manifest with GNU tar, exactly as
//  `.github/workflows/android.yml` does when it publishes the city, and streams the
//  result through the same state machine — in bounded steps, so it pauses and
//  continues the way a frame loop makes it.
//
//  It runs only where the pipeline's output exists: a bare checkout has no
//  data/processed, and a missing world is a state, not a failure.
// =============================================================================
namespace {

// A transport that serves files from disk, honouring offsets: the shape a real HTTP
// range request has, without the network.
class FileFetcher : public Fetcher {
  public:
    std::string checksumsUrl = "file:///checksums.sha256";
    std::string archiveUrl = "file:///world.tar";
    std::string checksumsPath;
    std::string archivePath;
    u64 chunk = 64 * 1024;
    u64 bytesServed = 0;

    bool get(const std::string& url, u64 offset,
             const std::function<bool(const u8*, size_t)>& onData, u64* totalBytes,
             std::string* error) override {
        const std::string path = url == checksumsUrl ? checksumsPath : archivePath;
        std::FILE* file = std::fopen(path.c_str(), "rb");
        if (!file) {
            if (error) *error = "cannot open " + path;
            return false;
        }
        std::fseek(file, 0, SEEK_END);
        const long long size = std::ftell(file);
        if (totalBytes) *totalBytes = static_cast<u64>(size);
        std::fseek(file, static_cast<long>(offset), SEEK_SET);
        long long left = size - static_cast<long long>(offset);
        std::vector<u8> buffer(static_cast<size_t>(chunk));
        while (left > 0) {
            const size_t want =
                static_cast<size_t>(std::min<long long>(static_cast<long long>(chunk), left));
            const size_t got = std::fread(buffer.data(), 1, want, file);
            if (got == 0) break;
            bytesServed += got;
            if (!onData(buffer.data(), got)) {
                std::fclose(file);
                return false;   // the caller paused
            }
            left -= static_cast<long long>(got);
        }
        std::fclose(file);
        return true;
    }
};

void writeWholeFile(const std::string& path, const std::string& text) {
    std::FILE* file = std::fopen(path.c_str(), "wb");
    CHECK(file != nullptr);
    if (!file) return;
    const size_t written = std::fwrite(text.data(), 1, text.size(), file);
    std::fclose(file);
    CHECK_EQ(written, text.size());
}

}  // namespace

TEST(worldDownloadStreamsAnArchiveMadeByGnuTar) {
    const std::string processed = "data/processed";
    if (!std::filesystem::exists(processed + "/world_manifest.json")) {
        std::printf("  (skipped: this checkout has no data/processed)\n");
        return;
    }

    // The three smallest cells: real chunk files, real headers, and a test that
    // still runs in a couple of seconds.
    std::vector<std::pair<u64, std::string>> cells;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(processed + "/cells")) {
        if (entry.path().extension() != ".mumbai_chunk") continue;
        cells.push_back({static_cast<u64>(std::filesystem::file_size(entry.path())),
                         entry.path().filename().string()});
    }
    std::sort(cells.begin(), cells.end());
    CHECK(cells.size() >= 3u);
    if (cells.size() > 3) cells.resize(3);

    const std::string tarPath = "/tmp/mumbai_gnu_tar_test.tar";
    std::string command = "tar -cf " + tarPath + " -C " + processed;
    for (const auto& cell : cells) command += " cells/" + cell.second;
    command += " world_manifest.json";
    CHECK_EQ(std::system(command.c_str()), 0);   // the workflow's own command
    CHECK(std::filesystem::exists(tarPath));

    // The published list, in sha256sum's own format.
    const std::vector<std::string> names = {"cells/" + cells[0].second, "cells/" + cells[1].second,
                                           "cells/" + cells[2].second, "world_manifest.json"};
    std::vector<std::pair<std::string, std::string>> expected;   // path, hex
    std::string list;
    for (const std::string& name : names) {
        std::string hex;
        CHECK(sha256HexFile(processed + "/" + name, &hex));
        expected.push_back({name, hex});
        list += hex + "  " + name + "\n";
    }
    const std::string sumsPath = "/tmp/mumbai_gnu_tar_test.sha256";
    writeWholeFile(sumsPath, list);

    const std::string dir = freshWorldDir("gnutar");
    FileFetcher fetcher;
    fetcher.checksumsPath = sumsPath;
    fetcher.archivePath = tarPath;
    DownloadConfig config;
    config.checksumsUrl = fetcher.checksumsUrl;
    config.archiveUrl = fetcher.archiveUrl;
    config.manifestSha16 = expected.back().second.substr(0, 16);
    config.worldDir = dir;
    config.baseUrl = "file:///";

    WorldDownload download(&fetcher, config);
    u64 steps = 0;
    for (int i = 0; i < 4000 && download.state() != WorldDownload::State::Done &&
                    download.state() != WorldDownload::State::Failed;
         ++i) {
        // 128 KiB a step, well under the archive: the real download pauses several
        // times, in the middle of entries, and has to continue where it stopped.
        download.step(128u << 10);
        ++steps;
    }
    CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Done));
    CHECK(download.error().empty());
    CHECK(steps >= 3u);
    CHECK_EQ(download.filesWritten(), 4u);
    CHECK(download.receivedBytes() >= std::filesystem::file_size(tarPath));
    CHECK(download.sawManifest());

    // Every extracted byte is the pipeline's own byte.
    for (const auto& item : expected) {
        std::string hex;
        CHECK(sha256HexFile(dir + "/" + item.first, &hex));
        CHECK_EQ(hex, item.second);
    }
    CHECK(!std::filesystem::exists(dir + "/.download"));   // finished: nothing to resume
}

TEST(worldDownloadInstallsTheArchiveTheReleaseStepPacks) {
    // The state machine has been tested against tars built by hand and against a
    // tar built by `tar`.  Neither is what the release publishes: the published
    // archive is packed by tools/pack_world.py, gzipped members inside a plain
    // tar (D-050).  This case runs that tool over a world it writes itself and
    // installs the result through the same steps the client uses, so a change in
    // the packer's member names or the sink's `.gz` rule fails here rather than on
    // somebody's phone.
    if (!std::filesystem::exists("tools/pack_world.py")) {
        std::printf("  (skipped: no tools/pack_world.py in this directory)\n");
        return;
    }
    if (std::system("python3 -c 'pass' 2>/dev/null") != 0) {
        std::printf("  (skipped: no python3 on this machine)\n");
        return;
    }

    const std::string processed = "/tmp/mumbai_packed_world";
    const std::string outDir = "/tmp/mumbai_packed_release";
    std::error_code ec;
    std::filesystem::remove_all(processed, ec);
    std::filesystem::remove_all(outDir, ec);
    std::filesystem::create_directories(processed + "/cells", ec);
    std::filesystem::create_directories(processed + "/textures", ec);

    // Two chunks whose bytes are worth compressing, so the case would notice if the
    // tool quietly stored them raw.
    std::string chunkA;
    std::string chunkB;
    for (int i = 0; i < 4000; ++i) {
        chunkA += "A1: a terrain vertex, and the triangles that use it\n";
        chunkB += "B2: another cell, another four thousand lines of it\n";
    }
    const std::string texture(9000, 'T');
    const std::string manifest =
        "{\"cells\":[{\"file\":\"cells/cell_0_0.mumbai_chunk\",\"bytes\":" +
        std::to_string(chunkA.size()) + "}],\"totals\":{}}";
    writeWholeFile(processed + "/cells/cell_0_0.mumbai_chunk", chunkA);
    writeWholeFile(processed + "/cells/cell_1_0.mumbai_chunk", chunkB);
    writeWholeFile(processed + "/textures/albedo.mumbaitex", texture);
    writeWholeFile(processed + "/world_manifest.json", manifest);

    const std::string command = "python3 tools/pack_world.py --processed " + processed +
                                " --out " + outDir + " --verify > /tmp/mumbai_pack.log 2>&1";
    CHECK_EQ(std::system(command.c_str()), 0);   // the packer verifies its own archive

    std::string sumsPath;
    std::string tarPath;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(outDir)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("sha256sums-", 0) == 0) sumsPath = entry.path().string();
        if (name.rfind("mumbai-world-", 0) == 0) tarPath = entry.path().string();
    }
    CHECK(!sumsPath.empty());
    CHECK(!tarPath.empty());
    if (sumsPath.empty() || tarPath.empty()) return;

    // The list must name the members the archive carries, and the manifest's entry
    // in it must be the manifest this build would accept.
    const std::string list = readWhole(sumsPath);
    CHECK(list.find("cells/cell_0_0.mumbai_chunk.gz") != std::string::npos);
    CHECK(list.find("textures/albedo.mumbaitex.gz") != std::string::npos);
    CHECK(list.find("world_manifest.json") != std::string::npos);
    std::string manifestHex;
    CHECK(sha256HexFile(processed + "/world_manifest.json", &manifestHex));

    const std::string dir = freshWorldDir("packed");
    FileFetcher fetcher;
    fetcher.checksumsPath = sumsPath;
    fetcher.archivePath = tarPath;
    DownloadConfig config;
    config.checksumsUrl = fetcher.checksumsUrl;
    config.archiveUrl = fetcher.archiveUrl;
    config.manifestSha16 = manifestHex.substr(0, 16);
    config.worldDir = dir;
    config.baseUrl = "file:///";

    WorldDownload download(&fetcher, config);
    for (int i = 0; i < 2000 && download.state() != WorldDownload::State::Done &&
                    download.state() != WorldDownload::State::Failed;
         ++i) {
        download.step(96u << 10);
    }
    CHECK_EQ(static_cast<int>(download.state()), static_cast<int>(WorldDownload::State::Done));
    CHECK(download.error().empty());
    CHECK_EQ(download.filesWritten(), 4u);   // two chunks, one texture array, the manifest

    // The phone ends up with the pipeline's tree, not the archive's: no `.gz`, no
    // part file, and every byte the packer read back out of the archive.
    CHECK_EQ(readWhole(dir + "/cells/cell_0_0.mumbai_chunk"), chunkA);
    CHECK_EQ(readWhole(dir + "/cells/cell_1_0.mumbai_chunk"), chunkB);
    CHECK_EQ(readWhole(dir + "/textures/albedo.mumbaitex"), texture);
    CHECK(!std::filesystem::exists(dir + "/cells/cell_0_0.mumbai_chunk.gz"));
    CHECK(!std::filesystem::exists(dir + "/cells/cell_0_0.mumbai_chunk.part"));
    CHECK(download.sawManifest());

    // A world packed for a different build is refused before a byte of the archive
    // is fetched: the checksum list names the manifest, and the manifest is the
    // identity of the world (D-048).
    DownloadConfig elsewhere = config;
    elsewhere.manifestSha16 = std::string(16, 'f');
    elsewhere.worldDir = freshWorldDir("packed_wrong");
    WorldDownload wrongFetcher(&fetcher, elsewhere);
    wrongFetcher.step(64u << 10);
    CHECK_EQ(static_cast<int>(wrongFetcher.state()),
             static_cast<int>(WorldDownload::State::Failed));
    CHECK(!wrongFetcher.error().empty());
    CHECK_EQ(wrongFetcher.receivedBytes(), 0u);   // nothing of the archive was pulled
}

TEST(diskSpaceIsCheckedBeforeTheCityIsPulledAndNeverOnAGuess) {
    // The failure this prevents: a phone with too little room downloads 370 MiB,
    // then fails with "short write into cells/....part" and has nothing to show
    // for the transfer.  The numbers are measured (world_download.h) and the
    // decision is pure, so it can be driven here.
    DiskSpace plenty{4000ull * 1024 * 1024, true};
    CHECK(cityFitsIn(plenty));
    CHECK(cityHasRunningRoom(plenty));
    CHECK(startSpaceSentence(plenty).empty());     // nothing to say, either way
    CHECK(runningSpaceSentence(plenty).empty());

    // Room for the world but not for the world plus a margin: refused, and the
    // sentence says both numbers so a person knows how much to free.
    DiskSpace tight{kCityInstalledBytes + 1024, true};
    CHECK(!cityFitsIn(tight));
    const std::string sentence = startSpaceSentence(tight);
    CHECK(sentence.find("not enough storage") != std::string::npos);
    CHECK(sentence.find("GB") != std::string::npos);

    // Exactly the requirement: allowed.  A guard that refuses the boundary is a
    // guard that refuses machines which would have worked.
    DiskSpace exact{kCityInstalledBytes + kCityInstallMargin, true};
    CHECK(cityFitsIn(exact));

    // Mid-download, the reserve is what matters: the world has already been
    // written, so what is left to protect is the next chunk's part file and the
    // app's own working room.
    DiskSpace running{kCityRunningReserve + 1024, true};
    CHECK(cityHasRunningRoom(running));
    CHECK(cityFitsIn(running) == false);        // and starting here would be refused
    // A phone with nothing left gets the *start* advice when the button is pressed,
    // and the *paused* advice once a download is already running: the two are not
    // the same sentence, and neither invents the other's number.
    DiskSpace nearlyFull{1024, true};
    CHECK(!cityHasRunningRoom(nearlyFull));
    CHECK(startSpaceSentence(nearlyFull).find("not enough storage") != std::string::npos);
    CHECK(runningSpaceSentence(nearlyFull).find("nearly full") != std::string::npos);
    CHECK(runningSpaceSentence(nearlyFull).find("paused") != std::string::npos);

    // AN UNKNOWN ANSWER NEVER REFUSES.  A platform that cannot report free space
    // must not stop a download that would have worked, and the panel must not
    // invent a number: the sentence is empty and the checks pass.
    DiskSpace unknown{0, false};
    CHECK(cityFitsIn(unknown));
    CHECK(cityHasRunningRoom(unknown));
    CHECK(startSpaceSentence(unknown).empty());
    CHECK(runningSpaceSentence(unknown).empty());

    // Zero free with a known answer is a full disk, and that is not unknown.
    DiskSpace full{0, true};
    CHECK(!cityFitsIn(full));
    CHECK(!cityHasRunningRoom(full));
}
