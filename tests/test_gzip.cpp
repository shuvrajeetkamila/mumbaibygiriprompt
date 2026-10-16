// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_gzip.cpp
//
//  The member inflater, tested against streams `gzip` itself produced.  Nothing
//  here writes a gzip stream by hand: the point of the test is that the code reads
//  what the release step will actually make, and the release step is `gzip`.
//
//  The failures are the interesting half.  A truncated member, a member with a
//  flipped bit in the middle, a member whose trailer lies about its length, and a
//  member followed by bytes that belong to something else: each one has to be
//  refused, because each one, written out, is a terrain chunk that ends in nothing
//  (an artifact by the specification's own definition).
// =============================================================================
#include "engine/core/gzip.h"
#include "tests/test_harness.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace mumbai;

namespace {

std::string readWhole(const std::string& path) {
    std::FILE* file = std::fopen(path.c_str(), "rb");
    if (!file) return std::string();
    std::string out;
    char buffer[4096];
    size_t got = 0;
    while ((got = std::fread(buffer, 1, sizeof(buffer), file)) > 0) out.append(buffer, got);
    std::fclose(file);
    return out;
}

bool writeWhole(const std::string& path, const std::string& text) {
    std::FILE* file = std::fopen(path.c_str(), "wb");
    if (!file) return false;
    const size_t written = std::fwrite(text.data(), 1, text.size(), file);
    std::fclose(file);
    return written == text.size();
}

// Compresses with the real `gzip`, exactly as the release step does.  Returns an
// empty string when the tool is missing, and every case below then skips itself:
// a machine without gzip is a machine this test cannot speak to.
std::string gzipWithCli(const std::string& payload) {
    const std::string in = "/tmp/mumbai_gzip_test_in.bin";
    const std::string out = "/tmp/mumbai_gzip_test_out.gz";
    std::remove(out.c_str());
    if (!writeWhole(in, payload)) return std::string();
    const std::string command = "gzip -6 -n -c '" + in + "' > '" + out + "' 2>/dev/null";
    if (std::system(command.c_str()) != 0) return std::string();
    return readWhole(out);
}

// A payload that is not one repeated byte: deflate of a constant is a few bytes,
// and a test that never exercises the window proves nothing about the window.
std::string payloadOf(size_t bytes, u32 seed) {
    std::string text;
    text.reserve(bytes);
    u32 state = seed;
    for (size_t i = 0; i < bytes; ++i) {
        state = state * 1664525u + 1013904223u;
        text.push_back(static_cast<char>('a' + ((state >> 16) % 26)));
    }
    return text;
}

std::string inflate(const std::string& compressed, bool* ok, std::string* error,
                    size_t feedBytes = 0) {
    GzipInflater inflater;
    std::string out;
    inflater.setSink([&](const u8* bytes, size_t count) {
        out.append(reinterpret_cast<const char*>(bytes), count);
        return true;
    });
    const size_t step = feedBytes == 0 ? compressed.size() : feedBytes;
    for (size_t offset = 0; offset < compressed.size() && inflater.error().empty(); offset += step) {
        const size_t take = std::min(step, compressed.size() - offset);
        inflater.update(reinterpret_cast<const u8*>(compressed.data()) + offset, take);
    }
    const bool finished = inflater.finish();
    *ok = finished;
    *error = inflater.error();
    return out;
}

}  // namespace

TEST(gzipInflaterRoundTripsWhatTheReleaseStepMakes) {
    const std::string payload = payloadOf(300'000, 7);
    const std::string compressed = gzipWithCli(payload);
    if (compressed.empty()) return;   // no `gzip` on this machine

    bool ok = false;
    std::string error;
    const std::string out = inflate(compressed, &ok, &error);
    CHECK(ok);
    CHECK_EQ(out.size(), payload.size());
    CHECK(out == payload);
    // A real compressor must actually compress: a "gzip" that stored the payload
    // uncompressed would pass everything above and save nobody a megabyte.
    CHECK(compressed.size() < payload.size());
}

TEST(gzipInflaterSurvivesBeingFedOneByteAtATime) {
    // The transport's chunks are whatever the socket hands over, and the state
    // machine may hand the inflater a slice that ends in the middle of a deflate
    // block.  One byte at a time is the worst case of that (D-048's budget rule).
    const std::string payload = payloadOf(70'000, 11);
    const std::string compressed = gzipWithCli(payload);
    if (compressed.empty()) return;

    bool ok = false;
    std::string error;
    const std::string out = inflate(compressed, &ok, &error, 1);
    CHECK(ok);
    CHECK(out == payload);
}

TEST(gzipInflaterRefusesTruncationCorruptionAndTrailingBytes) {
    const std::string payload = payloadOf(200'000, 23);
    const std::string compressed = gzipWithCli(payload);
    if (compressed.empty()) return;
    bool ok = false;
    std::string error;

    // Truncated: the trailer never arrives, so `finish` must refuse.
    const std::string cut = compressed.substr(0, compressed.size() - 8);
    inflate(cut, &ok, &error);
    CHECK(!ok);
    CHECK(!error.empty());

    // Corrupted in the middle: either the stream or the trailer catches it.
    std::string broken = compressed;
    broken[broken.size() / 2] = static_cast<char>(broken[broken.size() / 2] ^ 0x5a);
    inflate(broken, &ok, &error);
    CHECK(!ok);

    // Trailing bytes after the member: the tar header said the member ended here,
    // so anything after the stream's own end is a container disagreeing with its
    // payload.  This is the check that keeps a wrong length from being written out.
    GzipInflater inflater;
    size_t produced = 0;
    inflater.setSink([&](const u8*, size_t count) {
        produced += count;
        return true;
    });
    CHECK(inflater.update(reinterpret_cast<const u8*>(compressed.data()), compressed.size()));
    CHECK(inflater.ended());
    CHECK_EQ(produced, payload.size());
    CHECK(!inflater.update(reinterpret_cast<const u8*>("x"), 1));
    CHECK(!inflater.error().empty());
}

TEST(gzipInflaterStopsWhenItsSinkStops) {
    // The sink is a file that ran out of space, or a caller with nowhere to put the
    // rest: the inflater must refuse to carry on rather than dropping those bytes.
    const std::string payload = payloadOf(400'000, 31);
    const std::string compressed = gzipWithCli(payload);
    if (compressed.empty()) return;

    GzipInflater inflater;
    size_t produced = 0;
    inflater.setSink([&](const u8*, size_t count) {
        produced += count;
        return produced < 10'000;   // "that is enough"
    });
    bool ok = true;
    for (size_t offset = 0; offset < compressed.size() && ok; offset += 4096) {
        const size_t take = std::min<size_t>(4096, compressed.size() - offset);
        ok = inflater.update(reinterpret_cast<const u8*>(compressed.data()) + offset, take);
    }
    CHECK(!ok);
    CHECK(!inflater.error().empty());
    CHECK(inflater.outBytes() < payload.size());
}
