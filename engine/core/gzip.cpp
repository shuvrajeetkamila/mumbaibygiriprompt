// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/gzip.cpp
//
//  A streaming inflater over the vendored zlib.  See the header for why the world
//  arrives compressed inside an uncompressed container.
//
//  THE OUTPUT BLOCK IS THE UNIT OF MEMORY.  Every call hands zlib a 64 KiB window
//  and passes whatever it filled straight to the sink; nothing accumulates.  That
//  is what lets a phone inflate a 57 MiB terrain chunk while it is also holding the
//  chunks it already has.
//
//  ERRORS ARE STICKY AND SPECIFIC.  Once a stream has been refused it stays
//  refused: half a member is not a file, and an installer that shrugged and carried
//  on would rename a truncated chunk into place.  zlib's own message is kept, since
//  "invalid distance too far back" is more useful in a log than "decompression
//  failed".
// =============================================================================
#include "engine/core/gzip.h"

// The vendored copy, seen through the same Z_PREFIX the build compiles it with: an
// Android process already has a zlib, and this library must not export symbols that
// could be bound to it (third_party/zlib/README.md).
#include <zlib.h>

namespace mumbai {
namespace {

// The decompressed block handed to the sink.  zlib's own advice is to size this to
// the working set; 64 KiB is the size the engine streams with elsewhere.
constexpr size_t kOutBlock = 64u << 10;

// 15 window bits is the maximum deflate allows, and +16 asks for the gzip wrapper
// (header, CRC-32, length) rather than the raw or zlib-wrapped stream.  A member
// that is not a gzip member at all is refused by inflateInit2's first byte check.
constexpr int kGzipWindowBits = 15 + 16;

std::string zlibMessage(int code, const z_stream& stream) {
    if (stream.msg && stream.msg[0]) return stream.msg;
    const char* text = zError(code);
    return text ? std::string(text) : std::string("deflate error");
}

}  // namespace

struct GzipInflater::Impl {
    z_stream stream{};
    std::vector<u8> out = std::vector<u8>(kOutBlock);
    GzipInflater::Sink sink;
    std::string error;
    bool initialised = false;
    bool sawEnd = false;
    bool finished = false;
    u64 produced = 0;
};

GzipInflater::GzipInflater() : m_impl(std::make_unique<Impl>()) {}

GzipInflater::~GzipInflater() {
    if (m_impl->initialised) inflateEnd(&m_impl->stream);
}

void GzipInflater::setSink(Sink sink) { m_impl->sink = std::move(sink); }

bool GzipInflater::update(const u8* bytes, size_t count) {
    Impl& self = *m_impl;
    if (self.error.empty() == false) return false;
    // A member's bytes are exactly the tar entry's bytes, so anything that arrives
    // after the member has ended is a header disagreeing with its payload.
    if (self.sawEnd) {
        if (count > 0) self.error = "bytes after the end of a compressed member";
        return count == 0;
    }
    if (!self.initialised) {
        self.stream.zalloc = nullptr;   // zlib's default allocator
        self.stream.zfree = nullptr;
        self.stream.opaque = nullptr;
        const int rc = inflateInit2(&self.stream, kGzipWindowBits);
        if (rc != Z_OK) {
            self.error = "could not start inflating: " + zlibMessage(rc, self.stream);
            return false;
        }
        self.initialised = true;
    }
    // zlib counts input in 32-bit units.  A caller feeding more than that at once
    // (nothing in this engine does) would silently lose the tail, so the loop below
    // works in pieces and `avail_in` is always what the stream was given.
    size_t offset = 0;
    while (offset < count || self.stream.avail_in > 0) {
        if (self.stream.avail_in == 0 && offset < count) {
            const size_t piece = std::min<size_t>(count - offset, 1u << 30);
            self.stream.next_in = const_cast<Bytef*>(bytes + offset);
            self.stream.avail_in = static_cast<uInt>(piece);
            offset += piece;
        }
        self.stream.next_out = reinterpret_cast<Bytef*>(self.out.data());
        self.stream.avail_out = static_cast<uInt>(self.out.size());
        const int rc = inflate(&self.stream, Z_NO_FLUSH);
        const size_t produced = self.out.size() - self.stream.avail_out;
        if (produced > 0) {
            self.produced += produced;
            if (self.sink && !self.sink(self.out.data(), produced)) {
                self.error = "the caller stopped reading the compressed member";
                return false;
            }
        }
        if (rc == Z_STREAM_END) {
            self.sawEnd = true;
            // The tar member's length is the member's length: leftover input here
            // means the container and the payload disagree about where the file
            // ended, which is exactly the corruption the checksum could not see.
            if (self.stream.avail_in > 0) {
                self.error = "the compressed member ended before its file did";
                return false;
            }
            return true;
        }
        if (rc == Z_OK || rc == Z_BUF_ERROR) {
            // Z_BUF_ERROR with input left and an empty 64 KiB window means the
            // stream cannot make progress at all: it is malformed, not merely
            // unfinished.  Reported rather than looped on.
            if (rc == Z_BUF_ERROR && produced == 0 && self.stream.avail_in > 0) {
                self.error = "the compressed member stalled";
                return false;
            }
            if (self.stream.avail_in == 0 && produced == 0) return true;  // need more input
            continue;
        }
        self.error = zlibMessage(rc, self.stream);
        return false;
    }
    return true;
}

bool GzipInflater::finish() {
    Impl& self = *m_impl;
    if (!self.error.empty()) return false;
    if (!self.sawEnd) {
        self.error = "the compressed member ended early";
        return false;
    }
    self.finished = true;
    return true;
}

bool GzipInflater::ended() const { return m_impl->sawEnd; }
u64 GzipInflater::outBytes() const { return m_impl->produced; }
const std::string& GzipInflater::error() const { return m_impl->error; }

}  // namespace mumbai
