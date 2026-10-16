// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/gzip.h
//
//  WHY THE ENGINE INFLATES AT ALL.
//
//  The city is a download.  A download of 1.4 GB over a phone is not a download
//  anyone finishes, and a *compressed* 1.4 GB is worse than useless: a paused
//  download has to come back to a byte offset, and a deflate stream has no usable
//  offsets — resuming would mean inflating from zero every time, which is the one
//  thing the resume design exists to avoid (D-049).
//
//  So the world travels as an uncompressed tar whose *members* are gzipped.  The
//  container stays a stream a client can walk entry by entry — every file boundary
//  is a resume point — and the payload inside each member is compressed, which is
//  what turns 1.4 GB of transfer into about 180 MB.  This is the inflater for the
//  members; `world::ExtractingSink` is what feeds it.
//
//  It is a streaming inflater and nothing else: no file API, no zlib types in this
//  header, and no compression.  Bytes in, bytes out, one block at a time, so a
//  57 MiB chunk costs a 64 KiB buffer rather than 57 MiB of RSS on a phone that is
//  already streaming a city.
//
//  Integrity, in the order it is checked: this reports a broken stream (a
//  truncated member, a wrong length, a wrong window), zlib itself refuses a member
//  whose trailer CRC-32 or length does not match its payload, and the download
//  checks the *compressed* bytes against the published sha256 before the file is
//  renamed into place.  Three independent refusals, so a corrupt member cannot
//  become a corrupted city (which would read as terrain ending in a cliff or a
//  building standing in the sea — artifacts by the specification's own definition).
//
//  See `third_party/zlib/README.md` for the vendored inflate code and why it is
//  compiled with Z_PREFIX.
// =============================================================================
#pragma once
#include "engine/core/core.h"

#include <functional>
#include <memory>
#include <string>

namespace mumbai {

class GzipInflater {
  public:
    // The sink receives decompressed bytes.  Returning false means "stop"; the
    // inflater then refuses to go on, because a sink that stopped mid-member has
    // nowhere to put the rest of it.
    using Sink = std::function<bool(const u8*, size_t)>;

    GzipInflater();
    ~GzipInflater();
    GzipInflater(const GzipInflater&) = delete;
    GzipInflater& operator=(const GzipInflater&) = delete;

    void setSink(Sink sink);

    // Feeds the next slice of a gzip member.  False means the stream is broken and
    // nothing after it should be trusted; `error()` says what happened.
    bool update(const u8* bytes, size_t count);

    // Call once the member's last compressed byte has been fed.  True only when the
    // member actually ended: a truncated member reports false here, and this is the
    // only place the end of a member can be noticed, because the inflater cannot
    // know how many bytes were supposed to arrive.
    bool finish();

    bool ended() const;      // the member's own end marker was reached
    u64 outBytes() const;    // decompressed bytes handed to the sink
    const std::string& error() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

}  // namespace mumbai
