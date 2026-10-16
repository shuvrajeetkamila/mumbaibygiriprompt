// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/world_archive.h
//
//  THE WORLD ARRIVES AS A TAR, AND IT ARRIVES ONCE.
//
//  The published city is a tar archive of `cells/`, `textures/` and
//  `world_manifest.json` — the same tree `make world` writes.  A tar is the right
//  container here for an ungrateful reason: it can be read *while it streams*,
//  with no index and no random access, so a 1.3 GB download can be unpacked as it
//  arrives and hash-checked entry by entry.
//
//  Two pieces, both deliberately free of Android and of sockets so they can be
//  tested on a machine with neither:
//
//    * TarReader      — the ustar parser.  Feed it bytes; it drives a TarSink.
//    * ExtractingSink — a sink that verifies and writes: every file must match a
//                       hash from the published checksum list before it is allowed
//                       to exist under its final name, and no path may escape the
//                       world directory.
//
//  WHAT THIS IS NOT.  A tar is not compression, and this is not a security
//  sandbox: the hashes come from the same place as the bytes, so they catch
//  corruption and truncation, not someone who can rewrite both.  See D-048.
// =============================================================================
#pragma once
#include "engine/core/core.h"
#include "engine/core/sha256.h"

#include <cstdio>
#include <functional>
#include <map>
#include <string>

namespace mumbai::world {

// One file's worth of archive metadata: what it is called and how big it claims
// to be.  The size is the tar header's, not a promise — a truncated stream is
// detected by the reader, never trusted from the header.
struct TarEntry {
    std::string path;
    u64 size = 0;
};

// What the reader drives.  `begin` is given the entry, then `data` is called with
// the file's bytes in order, then `end`.  Any of the three returning false stops
// the reader and becomes its error: a sink that cannot write must not be fed more.
class TarSink {
  public:
    virtual ~TarSink() = default;
    virtual bool begin(const TarEntry& entry) = 0;
    virtual bool data(const u8* bytes, size_t count) = 0;
    virtual bool end() = 0;

    // Why the sink refused, for the reader to report instead of a generic message.
    // A sink that knows says so: "checksum mismatch for cells/x.mumbai_chunk" is a
    // diagnosis, "the sink rejected a file" is a shrug.
    virtual std::string error() const { return std::string(); }
};

class TarReader {
  public:
    explicit TarReader(TarSink& sink) : m_sink(sink) {}

    bool feed(const u8* bytes, size_t count);
    bool finish();

    // Called with the number of bytes consumed once a *file* entry is complete —
    // the only kind of offset a resumed download may start from, because it is the
    // only one at which the parser's position is also a file boundary.
    std::function<void(u64)> onEntryComplete;

    u64 entries() const { return m_entries; }
    u64 files() const { return m_files; }
    u64 skipped() const { return m_skipped; }
    u64 bytesIn() const { return m_bytesIn; }
    const std::string& error() const { return m_error; }

  private:
    bool consume(const u8* bytes, size_t count);
    bool beginEntry(const u8* header);

    TarSink& m_sink;
    std::string m_error;
    std::string m_longName;      // GNU tar's 'L' entry, if one arrives
    u64 m_remaining = 0;         // bytes still to come of what is being read
    u64 m_padding = 0;           // block padding after it
    u64 m_entries = 0;
    u64 m_files = 0;
    u64 m_skipped = 0;
    u64 m_bytesIn = 0;
    size_t m_nameUsed = 0;       // how much of the long name has arrived
    bool m_inFile = false;
    bool m_collectingName = false;
    bool m_fileJustEnded = false;
    u8 m_header[512]{};
    size_t m_headerUsed = 0;
};

// Parses the `sha256  name` lines `sha256sum` writes (and that the workflow
// publishes beside the archive).  A `*` binary marker before the name is accepted.
bool parseChecksumList(const std::string& text, std::map<std::string, std::string>* out);

// A sink that writes verified files under a root directory.
//
//   * every file must appear in `expected`, or it is refused (a world is exactly
//     the set of files the build published, not whatever a stream contains);
//   * its sha256 is computed while it is written, into `<path>.part`;
//   * the part file is renamed into place only after the hash matches, so an
//     interrupted or corrupted download never leaves a readable bad chunk;
//   * no path may be absolute, contain a `..` component, or use a backslash.
class ExtractingSink : public TarSink {
  public:
    ExtractingSink(std::string root, std::map<std::string, std::string> expected);
    ~ExtractingSink() override;

    bool begin(const TarEntry& entry) override;
    bool data(const u8* bytes, size_t count) override;
    bool end() override;

    u64 filesWritten() const { return m_filesWritten; }
    u64 bytesWritten() const { return m_bytesWritten; }
    u64 rejected() const { return m_rejected; }
    bool sawManifest() const { return m_sawManifest; }
    const std::string& manifestHex() const { return m_manifestHex; }
    const std::string& lastPath() const { return m_lastPath; }
    std::string error() const override { return m_error; }
    void setError(const std::string& text) { m_error = text; }

    // The paths a caller needs without knowing the archive's layout.
    static constexpr const char* kManifestName = "world_manifest.json";

  private:
    void closePart(bool keep);

    std::string m_root;
    std::map<std::string, std::string> m_expected;
    std::string m_error;
    std::string m_lastPath;
    std::string m_manifestHex;
    std::string m_partPath;
    std::string m_finalPath;
    std::FILE* m_file = nullptr;
    Sha256 m_hash;
    u64 m_expectedBytes = 0;
    u64 m_seenBytes = 0;
    bool m_sawManifest = false;
    u64 m_filesWritten = 0;
    u64 m_bytesWritten = 0;
    u64 m_rejected = 0;
};

// Directory helper the sink needs and the client reuses: creates every missing
// component of `path`.  Returns false only when a component exists and is not a
// directory.  (The client's own installer writes `cells/` and `textures/` one
// level down; this is the general form.)
bool makeDirectories(const std::string& path);

}  // namespace mumbai::world
