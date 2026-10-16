// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_world_archive.cpp
//
//  The client's download path, tested where no phone and no network are needed:
//  SHA-256 against the standard's own vectors, the tar reader against archives
//  written here in memory, and the verifier against the three ways a download goes
//  wrong — a corrupted file, an archive that stops early, and a path that tries to
//  leave the world directory.
//
//  A tar writer is deliberately NOT part of the engine: the published archive is
//  built by `tar` in the workflow, and a writer in the engine would be a second
//  implementation to keep in step with it.  The few lines needed to build one for
//  a test live here, where they cannot be mistaken for the real thing.
// =============================================================================
#include "engine/core/sha256.h"
#include "engine/world/world_archive.h"
#include "tests/test_harness.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

using namespace mumbai;
using namespace mumbai::world;

namespace {

constexpr size_t kBlock = 512;

void putOctal(u8* field, size_t length, u64 value) {
    // Twelve octal digits, NUL terminated — the form `tar` writes for sizes.
    std::string text;
    do {
        text.insert(text.begin(), static_cast<char>('0' + (value & 7u)));
        value >>= 3;
    } while (value != 0);
    while (text.size() + 1 < length) text.insert(text.begin(), '0');
    for (size_t i = 0; i < text.size() && i < length; ++i) field[i] = static_cast<u8>(text[i]);
    if (text.size() < length) field[text.size()] = 0;
}

void appendHeader(std::vector<u8>& out, const std::string& name, u64 size, char type) {
    const size_t base = out.size();
    out.resize(base + kBlock, 0);
    u8* block = out.data() + base;
    std::snprintf(reinterpret_cast<char*>(block), 100, "%s", name.c_str());
    putOctal(block + 100, 8, 0644);      // mode
    putOctal(block + 108, 8, 0);         // uid
    putOctal(block + 116, 8, 0);         // gid
    putOctal(block + 124, 12, size);     // size
    putOctal(block + 136, 12, 0);        // mtime
    for (size_t i = 148; i < 156; ++i) block[i] = ' ';   // checksum field as spaces
    block[156] = static_cast<u8>(type);
    std::snprintf(reinterpret_cast<char*>(block + 257), 6, "ustar");
    block[263] = '0';
    block[264] = '0';
    u64 sum = 0;
    for (size_t i = 0; i < kBlock; ++i) sum += block[i];
    // The checksum is written as six octal digits, NUL, space.
    std::string text;
    do {
        text.insert(text.begin(), static_cast<char>('0' + (sum & 7u)));
        sum >>= 3;
    } while (sum != 0);
    while (text.size() < 6) text.insert(text.begin(), '0');
    for (size_t i = 0; i < 6; ++i) block[148 + i] = static_cast<u8>(text[i]);
    block[154] = 0;
    block[155] = ' ';
}

void appendFile(std::vector<u8>& out, const std::string& name, const std::string& bytes) {
    appendHeader(out, name, bytes.size(), '0');
    const size_t base = out.size();
    out.resize(base + bytes.size());
    std::memcpy(out.data() + base, bytes.data(), bytes.size());
    while (out.size() % kBlock != 0) out.push_back(0);
}

// GNU tar's long-name entry: a pseudo-file whose payload is the path of the next
// entry.  The client must read the real path, not a truncated one.
void appendLongName(std::vector<u8>& out, const std::string& name) {
    appendHeader(out, "././@LongLink", name.size() + 1, 'L');
    const size_t base = out.size();
    out.resize(base + name.size() + 1, 0);
    std::memcpy(out.data() + base, name.data(), name.size());
    while (out.size() % kBlock != 0) out.push_back(0);
}

void finishArchive(std::vector<u8>& out) {
    out.resize(out.size() + 2 * kBlock, 0);   // the two end blocks
}

std::string checksumLine(const std::string& name, const std::string& bytes) {
    return sha256Hex(bytes.data(), bytes.size()) + "  " + name + "\n";
}

std::string tempRoot(const char* leaf) {
    const std::string root = std::string("/tmp/mumbai_archive_test_") + leaf;
    std::error_code ec;
    std::filesystem::remove_all(root, ec);
    std::filesystem::create_directories(root, ec);
    return root;
}

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

// Compresses with the real `gzip`, as the release step does.  An empty string means
// the tool is missing, and the cases that use it then skip themselves.
std::string gzipBytes(const std::string& payload) {
    const std::string in = "/tmp/mumbai_archive_test_in.bin";
    const std::string out = "/tmp/mumbai_archive_test_out.gz";
    std::remove(out.c_str());
    std::FILE* file = std::fopen(in.c_str(), "wb");
    if (!file) return std::string();
    std::fwrite(payload.data(), 1, payload.size(), file);
    std::fclose(file);
    const std::string command = "gzip -6 -n -c '" + in + "' > '" + out + "' 2>/dev/null";
    if (std::system(command.c_str()) != 0) return std::string();
    return readWhole(out);
}

}  // namespace

TEST(sha256MatchesTheStandardsOwnVectors) {
    CHECK_EQ(sha256Hex("", 0),
             std::string("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"));
    CHECK_EQ(sha256Hex("abc", 3),
             std::string("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
    const std::string two =
        "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";   // 56 bytes: the padding edge
    CHECK_EQ(sha256Hex(two.data(), two.size()),
             std::string("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"));

    // 55 and 56 bytes are the two sides of the length-padding boundary: a hash that
    // forgets to reserve eight bytes for the length gets one of them wrong.
    std::string edge(55, 'a');
    CHECK_EQ(sha256Hex(edge.data(), edge.size()),
             std::string("9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318"));
    edge.push_back('a');
    CHECK_EQ(sha256Hex(edge.data(), edge.size()),
             std::string("b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a"));

    // Streaming in awkward chunk sizes must give the same digest as one shot.
    std::string payload(100000, 'x');
    for (int i = 0; i < 100000; ++i) payload[static_cast<size_t>(i)] = static_cast<char>(i % 251);
    Sha256 streamed;
    for (size_t offset = 0; offset < payload.size();) {
        const size_t chunk = std::min<size_t>(7, payload.size() - offset);
        streamed.update(payload.data() + offset, chunk);
        offset += chunk;
    }
    CHECK_EQ(streamed.digestHex(), sha256Hex(payload.data(), payload.size()));

    // A million 'a's: the standard's long-message vector.
    std::string million(1000000, 'a');
    CHECK_EQ(sha256Hex(million.data(), million.size()),
             std::string("cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"));

    // And the real thing, when this machine has the source data: the OSM extract's
    // first bytes are a known value from DOCS/01_DATA_SOURCES.md.
    std::string hex;
    if (sha256HexFile("data/raw/mumbai_region.osm", &hex)) {
        CHECK_EQ(hex.substr(0, 8), std::string("54f4f6bc"));
    }
    CHECK(!sha256HexFile("/tmp/mumbai_there_is_no_such_file.osm", &hex));
}

TEST(tarReaderExtractsAVerifiedArchiveAndResumesAtABoundary) {
    const std::string root = tempRoot("roundtrip");
    std::vector<u8> tar;
    const std::string a = "MUMCHNK1 the first chunk";
    const std::string b = std::string(1500, 'B');           // crosses a block boundary
    const std::string c = "{}";                              // the manifest, small
    appendFile(tar, "cells/cell_0006_0003.mumbai_chunk", a);
    appendFile(tar, "cells/cell_0006_0004.mumbai_chunk", b);
    appendFile(tar, "world_manifest.json", c);
    finishArchive(tar);

    std::map<std::string, std::string> expected;
    CHECK(parseChecksumList(checksumLine("cells/cell_0006_0003.mumbai_chunk", a) +
                                checksumLine("cells/cell_0006_0004.mumbai_chunk", b) +
                                checksumLine("world_manifest.json", c),
                            &expected));
    CHECK_EQ(expected.size(), 3u);

    // Feeds of seven bytes at a time: the reader must never assume it is handed a
    // block, a header, or even a file.
    ExtractingSink sink(root, expected);
    TarReader reader(sink);
    std::vector<u64> boundaries;
    reader.onEntryComplete = [&](u64 offset) { boundaries.push_back(offset); };
    for (size_t offset = 0; offset < tar.size(); offset += 7) {
        const size_t chunk = std::min<size_t>(7, tar.size() - offset);
        CHECK(reader.feed(tar.data() + offset, chunk));
    }
    CHECK(reader.finish());
    CHECK_EQ(reader.files(), 3u);
    CHECK_EQ(sink.filesWritten(), 3u);
    CHECK_EQ(sink.bytesWritten(), a.size() + b.size() + c.size());
    CHECK(sink.sawManifest());
    CHECK_EQ(sink.manifestHex(), sha256Hex(c.data(), c.size()));
    CHECK_EQ(readWhole(root + "/cells/cell_0006_0003.mumbai_chunk"), a);
    CHECK_EQ(readWhole(root + "/cells/cell_0006_0004.mumbai_chunk"), b);
    CHECK_EQ(readWhole(root + "/world_manifest.json"), c);
    CHECK(std::filesystem::exists(root + "/cells") && std::filesystem::is_directory(root + "/cells"));

    // The reported offsets are block boundaries, in order, and each one is a place
    // a resumed download can start from.
    CHECK_EQ(boundaries.size(), 3u);
    for (size_t i = 0; i < boundaries.size(); ++i) {
        CHECK_EQ(boundaries[i] % kBlock, 0u);
        if (i > 0) CHECK(boundaries[i] > boundaries[i - 1]);
    }

    // Resume: a second reader, given the archive from a boundary, finishes the job
    // without repeating the files the first one wrote.  `boundaries[i]` is the
    // offset just after file i, so starting at boundaries[1] delivers exactly the
    // third file — which is what a download resuming after two files does.
    const std::string root2 = tempRoot("resume");
    ExtractingSink sink2(root2, expected);
    TarReader reader2(sink2);
    CHECK(reader2.feed(tar.data() + boundaries[1], tar.size() - boundaries[1]));
    CHECK(reader2.finish());
    CHECK_EQ(reader2.files(), 1u);
    CHECK_EQ(sink2.filesWritten(), 1u);
    CHECK_EQ(readWhole(root2 + "/world_manifest.json"), c);
    CHECK(!std::filesystem::exists(root2 + "/cells/cell_0006_0003.mumbai_chunk"));

    // Nothing is left behind under a part name when everything verified.
    CHECK(!std::filesystem::exists(root + "/cells/cell_0006_0003.mumbai_chunk.part"));
}

TEST(tarReaderRefusesCorruptionTruncationAndEscape) {
    // 1. A file whose bytes do not match the published hash is refused, and the
    //    half-written part file is removed rather than left to be read later.
    {
        const std::string root = tempRoot("corrupt");
        std::vector<u8> tar;
        appendFile(tar, "cells/a.mumbai_chunk", "good bytes");
        finishArchive(tar);
        std::map<std::string, std::string> expected{
            {"cells/a.mumbai_chunk", sha256Hex("different bytes", 15)}};
        ExtractingSink sink(root, expected);
        TarReader reader(sink);
        CHECK(!reader.feed(tar.data(), tar.size()));
        CHECK(reader.error().find("checksum") != std::string::npos);
        CHECK(!std::filesystem::exists(root + "/cells/a.mumbai_chunk"));
        CHECK(!std::filesystem::exists(root + "/cells/a.mumbai_chunk.part"));
    }

    // 2. An archive that stops in the middle of a file is not a world.
    {
        const std::string root = tempRoot("truncated");
        std::vector<u8> tar;
        appendFile(tar, "cells/b.mumbai_chunk", std::string(2000, 'z'));
        finishArchive(tar);
        std::map<std::string, std::string> expected{
            {"cells/b.mumbai_chunk", sha256Hex(std::string(2000, 'z').data(), 2000)}};
        ExtractingSink sink(root, expected);
        TarReader reader(sink);
        const size_t cut = tar.size() - 300;   // inside the file's own bytes
        // The bytes that did arrive are accepted — the sink cannot know they were
        // meant to be the whole file — and the *end of the stream* is what refuses.
        CHECK(reader.feed(tar.data(), cut));
        CHECK(!reader.finish());
        CHECK(reader.error().find("middle") != std::string::npos);
        // A stream that ends after the header alone is refused the same way.
        const std::string root2 = tempRoot("truncated2");
        ExtractingSink sink2(root2, expected);
        TarReader reader2(sink2);
        CHECK(reader2.feed(tar.data(), 512));
        CHECK(!reader2.finish());
        CHECK(reader2.error().find("middle") != std::string::npos);
    }

    // 3. A path that tries to leave the world directory is refused before any
    //    byte is written — and what a hostile archive asks for does not exist.
    {
        const std::string root = tempRoot("escape");
        std::vector<u8> tar;
        appendFile(tar, "../escaped.mumbai_chunk", "nope");
        finishArchive(tar);
        std::map<std::string, std::string> expected{
            {"../escaped.mumbai_chunk", sha256Hex("nope", 4)}};
        ExtractingSink sink(root, expected);
        TarReader reader(sink);
        CHECK(!reader.feed(tar.data(), tar.size()));
        CHECK(reader.error().find("unsafe path") != std::string::npos);
        CHECK(!std::filesystem::exists("/tmp/mumbai_archive_test_escape/../escaped.mumbai_chunk"));
        std::error_code ec;
        CHECK(!std::filesystem::exists("/tmp/escaped.mumbai_chunk", ec));
    }

    // 4. A file the build never published is refused: a world is exactly the set of
    //    files the checksum list names.
    {
        const std::string root = tempRoot("extra");
        std::vector<u8> tar;
        appendFile(tar, "cells/extra.mumbai_chunk", "unpublished");
        finishArchive(tar);
        std::map<std::string, std::string> expected{
            {"cells/a.mumbai_chunk", sha256Hex("x", 1)}};
        ExtractingSink sink(root, expected);
        TarReader reader(sink);
        CHECK(!reader.feed(tar.data(), tar.size()));
        CHECK(sink.error().find("not published") != std::string::npos);
        CHECK(!std::filesystem::exists(root + "/cells/extra.mumbai_chunk"));
    }

    // 5. A symlink is refused, not followed: it is a way out of the directory.
    {
        const std::string root = tempRoot("symlink");
        std::vector<u8> tar;
        appendHeader(tar, "cells/link", 0, '2');   // type '2' = symlink
        while (tar.size() % kBlock != 0) tar.push_back(0);
        finishArchive(tar);
        std::map<std::string, std::string> expected{{"cells/link", sha256Hex("", 0)}};
        ExtractingSink sink(root, expected);
        TarReader reader(sink);
        CHECK(!reader.feed(tar.data(), tar.size()));
        CHECK(reader.error().find("type") != std::string::npos);
    }

    // 6. A header whose checksum does not match is not a header.
    {
        const std::string root = tempRoot("badheader");
        std::vector<u8> tar;
        appendFile(tar, "cells/c.mumbai_chunk", "bytes");
        tar[10] ^= 0x5A;   // a stray bit in the name field
        finishArchive(tar);
        std::map<std::string, std::string> expected{
            {"cells/c.mumbai_chunk", sha256Hex("bytes", 5)}};
        ExtractingSink sink(root, expected);
        TarReader reader(sink);
        CHECK(!reader.feed(tar.data(), tar.size()));
        CHECK(reader.error().find("checksum mismatch") != std::string::npos);
    }
}

TEST(tarReaderFollowsGnuLongNamesAndSkipsDirectories) {
    const std::string root = tempRoot("longname");
    const std::string longName = "cells/" + std::string(120, 'l') + ".mumbai_chunk";
    std::vector<u8> tar;
    appendHeader(tar, "cells/", 0, '5');       // a directory entry: skipped, not read
    while (tar.size() % kBlock != 0) tar.push_back(0);
    appendLongName(tar, longName);
    appendHeader(tar, "truncated-name", 5, '0');   // the long name overrides this
    const char payload[5] = {'h', 'e', 'l', 'l', 'o'};
    tar.insert(tar.end(), payload, payload + 5);
    while (tar.size() % kBlock != 0) tar.push_back(0);
    finishArchive(tar);

    std::map<std::string, std::string> expected{{longName, sha256Hex("hello", 5)}};
    ExtractingSink sink(root, expected);
    TarReader reader(sink);
    CHECK(reader.feed(tar.data(), tar.size()));
    CHECK(reader.finish());
    CHECK_EQ(reader.files(), 1u);
    CHECK_EQ(sink.filesWritten(), 1u);
    CHECK_EQ(readWhole(root + "/" + longName), std::string("hello"));
    CHECK(!std::filesystem::exists(root + "/truncated-name"));
}

TEST(tarReaderInflatesGzipMembersIntoTheNamesTheManifestUses) {
    // THE SHAPE THE RELEASE ACTUALLY PUBLISHES (D-050).  `world-data` is an
    // uncompressed tar whose members are gzipped, so that a paused download keeps
    // its resume points (every file boundary is one) and the transfer is about an
    // eighth of the size.  What has to hold: the *archive's* name is what the
    // checksum list covers, and the *stored* name is what the manifest describes —
    // `cells/a.mumbai_chunk.gz` in, `cells/a.mumbai_chunk` out, byte for byte.
    std::string chunk;
    for (int line = 0; line < 40; ++line) {
        chunk += "a terrain chunk that compresses, so the test is really reading gzip\n";
    }
    const std::string gz = gzipBytes(chunk);
    if (gz.empty()) return;   // no `gzip` on this machine
    const std::string manifest = "{\"cells\":[{\"file\":\"cells/a.mumbai_chunk\"}]}";
    const std::string textureGz = gzipBytes(std::string(700, 'T'));

    std::vector<u8> archive;
    appendFile(archive, "cells/a.mumbai_chunk.gz", gz);
    appendFile(archive, "textures/albedo.mumbaitex.gz", textureGz);
    appendFile(archive, "world_manifest.json", manifest);
    finishArchive(archive);

    std::string list;
    list += checksumLine("cells/a.mumbai_chunk.gz", gz);
    list += checksumLine("textures/albedo.mumbaitex.gz", textureGz);
    list += checksumLine("world_manifest.json", manifest);
    std::map<std::string, std::string> expected;
    CHECK(parseChecksumList(list, &expected));

    const std::string root = tempRoot("gzip");
    ExtractingSink sink(root, expected);
    TarReader reader(sink);
    CHECK(reader.feed(archive.data(), archive.size()));
    CHECK(reader.finish());
    CHECK_EQ(sink.filesWritten(), 3u);
    CHECK(sink.sawManifest());
    // The compressed bytes are what arrived; what landed is the payload.
    CHECK_EQ(readWhole(root + "/cells/a.mumbai_chunk"), chunk);
    CHECK_EQ(std::filesystem::file_size(root + "/cells/a.mumbai_chunk"), chunk.size());
    CHECK_EQ(readWhole(root + "/textures/albedo.mumbaitex"), std::string(700, 'T'));
    // No `.gz` and no part file is left anywhere under the world directory: the
    // reader of the world never has to know it arrived compressed.
    CHECK(!std::filesystem::exists(root + "/cells/a.mumbai_chunk.gz"));
    CHECK(!std::filesystem::exists(root + "/cells/a.mumbai_chunk.part"));
    CHECK(!std::filesystem::exists(root + "/textures/albedo.mumbaitex.gz"));
    CHECK_EQ(sink.bytesWritten(), chunk.size() + 700 + manifest.size());

    // A member whose bytes are not a gzip stream at all is refused, and it leaves
    // nothing behind: `storedPath` may take the suffix off, but the inflater has to
    // agree that there was a stream under it.
    std::vector<u8> lying;
    appendFile(lying, "cells/a.mumbai_chunk.gz", "this is not deflate at all, not even close");
    appendFile(lying, "world_manifest.json", manifest);
    finishArchive(lying);
    std::string lyingList;
    lyingList += checksumLine("cells/a.mumbai_chunk.gz", "this is not deflate at all, not even close");
    lyingList += checksumLine("world_manifest.json", manifest);
    std::map<std::string, std::string> lyingExpected;
    CHECK(parseChecksumList(lyingList, &lyingExpected));
    const std::string lyingRoot = tempRoot("gzip_lying");
    ExtractingSink lyingSink(lyingRoot, lyingExpected);
    TarReader lyingReader(lyingSink);
    CHECK(!lyingReader.feed(lying.data(), lying.size()));
    CHECK(!lyingSink.error().empty());
    CHECK(!std::filesystem::exists(lyingRoot + "/cells/a.mumbai_chunk"));

    // A member cut short — a download that ended in the middle of a file — must be
    // refused too, even though every byte that did arrive was genuine.
    std::vector<u8> shortArchive;
    appendFile(shortArchive, "cells/a.mumbai_chunk.gz", gz.substr(0, gz.size() - 24));
    finishArchive(shortArchive);
    std::string shortList = checksumLine("cells/a.mumbai_chunk.gz", gz.substr(0, gz.size() - 24));
    std::map<std::string, std::string> shortExpected;
    CHECK(parseChecksumList(shortList, &shortExpected));
    ExtractingSink shortSink(tempRoot("gzip_short"), shortExpected);
    TarReader shortReader(shortSink);
    CHECK(!shortReader.feed(shortArchive.data(), shortArchive.size()));
    CHECK(!shortSink.error().empty());
}
