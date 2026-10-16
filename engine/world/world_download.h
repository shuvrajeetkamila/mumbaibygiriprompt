// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/world_download.h
//
//  THE REST OF THE CITY, ON REQUEST.
//
//  A fresh install carries South Bombay inside the APK (D-047) and nothing else:
//  1.3 GB of chunks is not something to put in a package, and it is not something
//  to start pulling over someone's mobile data without being asked.  This is the
//  state machine behind the client's one download: fetch the published checksum
//  list, then stream the archive, verifying and extracting as it arrives, with the
//  ability to stop and resume from the last complete file.
//
//  WHY THE TRANSPORT IS AN INTERFACE.  The engine has no TLS stack and should not
//  grow one; Android already has one behind java.net.  So the state machine here
//  knows *what* to fetch and *what to do with the bytes*, and knows nothing about
//  sockets, threads or the platform (engine/app/android_http.h is the Android
//  side).  That is also what makes this file testable: the tests drive it with a
//  fetcher that serves a tar out of memory, including a pause in the middle.
//
//  RESUME HAS TWO LEVELS, AND IT NEEDS BOTH.  A step is bounded by a budget so that
//  the frame loop stays a frame loop, but a `.mumbai_chunk` is tens of megabytes,
//  so a step almost always ends *inside* a file:
//
//    * in-process (warm): the parser and the sink keep the state of the stream, and
//      the next step asks the transport for the very byte after the last one and
//      carries on in the middle of the entry.  Nothing is re-fetched, and a 57 MiB
//      chunk is 57 MiB of transfer no matter how small the per-step budget is.
//      Pausing back to the last boundary here — the obvious design, and the first
//      one written — re-fetches the same bytes on every step and never finishes.
//
//    * cold start (a process that died): the archive is a stream and the parser
//      cannot restart inside a file, so the resume point is the last *entry
//      boundary* — the byte after a file's padding, remembered in
//      `<worldDir>/.download` and written only when a file completes.  The cost is
//      at most one entry re-fetched; the benefit is that what is on disk after a
//      cold start is finished work, never a half-written cell that the world
//      reader would happily mmap.
//
//  See D-048.
// =============================================================================
#pragma once
#include "engine/core/core.h"
#include "engine/world/world_archive.h"
#include "engine/world/world_release.h"

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace mumbai::world {

// One byte-range GET.  The transport streams the body to `onData` in whatever
// chunks it receives; returning false from `onData` means "stop, I have enough for
// now", which is how a download is paced by the frame loop.  On success the
// transport reports the resource's full size when the server sends one.
class Fetcher {
  public:
    virtual ~Fetcher() = default;
    virtual bool get(const std::string& url, u64 offset,
                     const std::function<bool(const u8*, size_t)>& onData, u64* totalBytes,
                     std::string* error) = 0;

    // True when the last `get` learned that this resource cannot be fetched as a
    // byte range — a server or a proxy that answers 200 with the whole file to a
    // Range request.  The downloader's answer is to throw its stream away and start
    // the archive again from zero: one wasted round trip, and never a world built
    // out of the tail of one download appended to the head of another.
    virtual bool restartRequired() const { return false; }
};

struct DownloadConfig {
    std::string checksumsUrl;    // where the published `sha256  name` list lives
    std::string archiveUrl;      // where the tar lives
    std::string manifestSha16;   // the manifest's sha256, first 16 hex digits
    std::string worldDir;        // the directory the world is installed into
    std::string baseUrl;         // for the panel's sentence, and for the log
};

class WorldDownload {
  public:
    enum class State {
        Idle,        // never started
        Checksums,   // asking for the checksum list
        Archive,     // streaming, verifying and extracting the archive
        Done,        // every file verified, the manifest matches its name
        Failed       // `error()` says why
    };

    WorldDownload(Fetcher* fetcher, DownloadConfig config);
    ~WorldDownload();

    // One slice of work, bounded by `budgetBytes` of payload.  Calling it again
    // continues where this call stopped; a paused download is not an error.
    void step(u64 budgetBytes);

    // Throws away every trace of a partial download and starts the archive from the
    // beginning (used when the server ignores Range and answers 200 with the whole
    // file, which must not be appended to a prefix we already have).
    void restart();

    State state() const { return m_state; }
    const std::string& error() const { return m_error; }
    f64 progress() const;
    // Bytes of the archive accounted for: where this attempt started plus what it has
    // delivered.  A cold start can make this number go backwards, and then it is
    // telling the truth: those bytes were thrown away with the dead process and are
    // about to be fetched again.
    u64 positionBytes() const;
    u64 receivedBytes() const { return m_received; }   // everything ever pulled
    u64 totalBytes() const { return m_total; }
    u64 resumeBytes() const { return m_boundary; }     // where a cold start would resume
    u64 filesWritten() const { return m_filesWritten; }
    bool sawManifest() const { return m_sawManifest; }
    std::string status() const;    // one sentence for the panel

  private:
    bool fetchChecksums();
    bool fetchArchive(u64 budgetBytes);
    void saveBoundary(u64 offset);
    u64 loadBoundary() const;

    Fetcher* m_fetcher = nullptr;
    DownloadConfig m_config;
    State m_state = State::Idle;
    std::string m_error;
    std::map<std::string, std::string> m_expected;
    std::unique_ptr<world::TarReader> m_reader;
    std::unique_ptr<world::ExtractingSink> m_sink;
    std::string m_checksums;
    u64 m_received = 0;      // every byte this object has ever pulled, re-fetches included
    u64 m_total = 0;         // the archive's size, as the server reports it
    u64 m_boundary = 0;      // last entry boundary: the cold-start resume point
    u64 m_nextOffset = 0;    // where the transport will be asked to continue from
    u64 m_attemptStart = 0;  // the offset the current attempt began at
    u64 m_attemptBytes = 0;  // what the current attempt has delivered
    u64 m_filesWritten = 0;
    bool m_sawManifest = false;
    bool m_paused = false;
};

// -----------------------------------------------------------------------------
//  ROOM FOR THE CITY
// -----------------------------------------------------------------------------
// A 370 MiB download that unpacks to 1.35 GiB must not be discovered at ninety per
// cent.  Without a check, a phone with too little space downloads for twenty
// minutes and then fails with "short write into cells/....part" — the person has
// paid for the transfer and has nothing to show for it.  So the arithmetic lives
// here, as pure functions over a number the platform supplies (android_main.cpp
// asks the filesystem; everything else can pass a simulated one), and the
// sentences are written once, beside the numbers they explain.
//
// MEASURED, NOT GUESSED: this checkout's city is 1,328,994,192 bytes of cells plus
// 107,921,238 bytes of texture arrays plus the manifest — 1.35 GiB on disk — while
// the archive that carries it is 370 MiB.  The starter slice is already installed
// by the time this is asked, and the 25 cells it contains are overwritten by the
// city rather than duplicated.
struct DiskSpace {
    u64 freeBytes = 0;
    bool known = false;   // false: the platform could not tell, so nothing is refused
};

inline constexpr u64 kCityInstalledBytes = 1449ull * 1024 * 1024;  // 1.35 GiB, rounded up
inline constexpr u64 kCityInstallMargin = 128ull * 1024 * 1024;    // room to keep running
inline constexpr u64 kCityRunningReserve = 96ull * 1024 * 1024;    // pause before the disk fills

// Enough room to start?  An unknown answer never refuses: a guard that blocks a
// download because it could not read a filesystem is worse than the failure it
// exists to prevent, and the failure it prevents is only ever a delay.
bool cityFitsIn(DiskSpace space);
// Enough room to keep going, asked again while the download runs (a phone's free
// space also shrinks for reasons that have nothing to do with this app).
bool cityHasRunningRoom(DiskSpace space);
// One sentence for the panel, or an empty string when there is nothing to say.
// Two functions rather than one, because the two situations want different advice
// and only one of them is true at a time: someone *pressing* GET CITY on a phone
// with no room needs to be told how much to free, and someone whose download is
// already running needs to be told it has paused and why.  One function that
// guessed which of those it was looking at would say "nearly full, paused" to a
// person who has not started, or invent a figure to free mid-download.
std::string startSpaceSentence(DiskSpace space);
std::string runningSpaceSentence(DiskSpace space);

// One URL in the release's shape: <repo>/releases/download/<tag>/<asset>.
std::string worldReleaseUrl(const std::string& repository, const std::string& tag,
                            const std::string& asset);

// The configuration a published release describes, which is the only way a client
// should ever build one: the two URLs, the manifest hash it will accept, and the
// directory to install into.  `world_release.h` carries the four strings for the
// build in hand; `publishedWorldRelease()` is empty for a build that ships no world
// download, and then this is a configuration with no URLs (see D-048).
DownloadConfig configForRelease(const WorldRelease& release, const std::string& worldDir);

}  // namespace mumbai::world
