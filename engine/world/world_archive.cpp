// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/world_archive.cpp
//
//  The ustar reader and the verifying extractor.  See the header for the design
//  and D-048 for why the client reads its world out of a stream at all.
//
//  The format, in the only detail this needs: a file is a 512-byte header block,
//  then its contents padded up to a 512-byte boundary.  The header carries the path
//  in `name` (100 bytes) and, for longer paths, in `prefix` (155) with `name` being
//  the tail; the size is twelve octal digits; `typeflag` says whether the entry is
//  a file ('0' or NUL), a directory ('5'), a link, or one of GNU's extensions.  The
//  archive ends with zero blocks.  A reader that does not insist on the padding and
//  the block boundary is a reader that accepts a truncated download as a world.
// =============================================================================
#include "engine/world/world_archive.h"

#include <cstring>
#include <filesystem>
#include <vector>

namespace mumbai::world {
namespace {

constexpr size_t kBlock = 512;

bool octal(const u8* field, size_t length, u64* out) {
    u64 value = 0;
    size_t i = 0;
    while (i < length && (field[i] == ' ' || field[i] == '\0')) ++i;
    bool any = false;
    for (; i < length; ++i) {
        const u8 c = field[i];
        if (c == '\0' || c == ' ') break;
        if (c < '0' || c > '7') return false;
        value = (value << 3) | static_cast<u64>(c - '0');
        any = true;
    }
    if (!any) return false;
    *out = value;
    return true;
}

std::string field(const u8* base, size_t offset, size_t length) {
    size_t count = 0;
    while (count < length && base[offset + count] != '\0') ++count;
    return std::string(reinterpret_cast<const char*>(base + offset), count);
}

bool allZero(const u8* block, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (block[i] != 0) return false;
    }
    return true;
}

// The header's own checksum: every byte summed with the checksum field read as
// spaces.  A tar whose checksum does not match is not a tar, whatever it claims.
bool headerChecksumOk(const u8* block) {
    u64 stored = 0;
    if (!octal(block + 148, 8, &stored)) return false;
    u64 sum = 0;
    for (size_t i = 0; i < kBlock; ++i) sum += (i >= 148 && i < 156) ? 0x20u : block[i];
    return sum == stored;
}

bool pathIsSafe(const std::string& path) {
    if (path.empty() || path.size() > 512) return false;
    if (path[0] == '/' || path[0] == '\\') return false;
    if (path.find('\\') != std::string::npos) return false;
    size_t start = 0;
    while (true) {
        const size_t slash = path.find('/', start);
        const std::string part =
            path.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
        if (part.empty() || part == "." || part == "..") return false;
        if (slash == std::string::npos) break;
        start = slash + 1;
    }
    return true;
}

}  // namespace

u64 removePartialFiles(const std::string& root) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::is_directory(root, ec)) return 0;
    u64 removed = 0;
    for (fs::recursive_directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec)) {
        const fs::directory_entry& entry = *it;
        std::error_code statEc;
        if (!entry.is_regular_file(statEc)) continue;
        const std::string name = entry.path().filename().string();
        if (name.size() > 5 && name.compare(name.size() - 5, 5, ".part") == 0) {
            std::error_code removeEc;
            if (fs::remove(entry.path(), removeEc)) removed += 1;
        }
    }
    return removed;
}

bool makeDirectories(const std::string& path) {
    if (path.empty()) return false;
    std::error_code ec;
    std::filesystem::create_directories(path, ec);
    if (ec) return false;
    return std::filesystem::is_directory(path, ec);
}

bool TarReader::feed(const u8* bytes, size_t count) {
    if (!m_error.empty()) return false;
    if (!bytes && count > 0) {
        m_error = "null data fed to the tar reader";
        return false;
    }
    while (count > 0) {
        size_t take = count;

        if (m_collectingName) {
            take = static_cast<size_t>(std::min<u64>(count, m_remaining));
            if (take > 0) {
                std::memcpy(&m_longName[m_nameUsed], bytes, take);
                m_nameUsed += take;
                m_remaining -= take;
            }
            if (m_remaining == 0) m_collectingName = false;
        } else if (m_inFile) {
            take = static_cast<size_t>(std::min<u64>(count, m_remaining));
            if (take > 0 && !m_sink.data(bytes, take)) {
                m_error = m_sink.error().empty() ? "the sink rejected file data" : m_sink.error();
                return false;
            }
            m_remaining -= take;
            if (m_remaining == 0) {
                if (!m_sink.end()) {
                    m_error =
                        m_sink.error().empty() ? "the sink rejected a file" : m_sink.error();
                    return false;
                }
                m_inFile = false;
                m_files += 1;
                m_fileJustEnded = true;
            }
        } else if (m_padding > 0) {
            // THE PADDING IS DRAINED BEFORE THE NEXT HEADER IS READ, and the order
            // is not cosmetic.  A file's contents are padded up to a block, so the
            // next header starts after that padding; a reader that begins
            // collecting a header first eats the padding as header bytes and is
            // misaligned from there on.  It does not show up when a whole archive
            // arrives in one call, because the padding is zeroes and a header of
            // zeroes looks like an end block — which is exactly how this survived
            // until a test fed the archive seven bytes at a time.
            take = static_cast<size_t>(std::min<u64>(count, m_padding));
            m_padding -= take;
        } else if (m_fileJustEnded) {
            // THE BOUNDARY IS REPORTED HERE, BETWEEN THE PADDING AND THE NEXT
            // HEADER, and nowhere else: it is the only position that is both a
            // parser position and a file boundary, which is what makes it the only
            // offset a resumed download may start from.  (A callback placed after
            // the header branch is unreachable while bytes remain, because there is
            // always a header to collect — the first version of this reported no
            // boundaries at all.)
            m_fileJustEnded = false;
            if (onEntryComplete) onEntryComplete(m_bytesIn);
            take = 0;
        } else if (m_headerUsed < kBlock) {
            take = std::min(count, kBlock - m_headerUsed);
            std::memcpy(m_header + m_headerUsed, bytes, take);
            m_headerUsed += take;
            if (m_headerUsed == kBlock && !beginEntry(m_header)) return false;
        } else {
            // Past the end blocks: the rest of the record is padding.
            take = count;
        }

        bytes += take;
        count -= take;
        m_bytesIn += take;
    }
    return true;
}

bool TarReader::beginEntry(const u8* header) {
    m_headerUsed = 0;
    m_remaining = 0;
    m_padding = 0;

    if (allZero(header, kBlock)) return true;   // an end block; finish() decides

    if (!headerChecksumOk(header)) {
        m_error = "tar header checksum mismatch";
        return false;
    }

    const std::string name = field(header, 0, 100);
    const std::string prefix = field(header, 345, 155);
    const char type = static_cast<char>(header[156]);
    u64 size = 0;
    if (!octal(header + 124, 12, &size)) {
        m_error = "tar header size is not octal";
        return false;
    }

    m_entries += 1;

    if (type == 'L') {
        // GNU tar writes a long path as its own entry whose payload is the path of
        // the next entry.  Our own paths are short enough for the plain header, but
        // an archive rewritten by another tool must not silently truncate one
        // chunk's name into another's.
        m_longName.assign(static_cast<size_t>(size), '\0');
        m_nameUsed = 0;
        m_collectingName = true;
        m_remaining = size;
        m_padding = (kBlock - (size % kBlock)) % kBlock;
        m_skipped += 1;
        return true;
    }

    std::string path = name;
    if (!m_longName.empty()) {
        // The payload is NUL terminated, so the path ends at the first NUL — a name
        // that kept the terminator would never match the checksum list.
        path = m_longName.substr(0, m_longName.find('\0'));
        m_longName.clear();
    } else if (!prefix.empty()) {
        path = prefix + "/" + name;
    }

    switch (type) {
        case '0':
        case '\0':
            if (!pathIsSafe(path)) {
                m_error = "refused an unsafe path in the archive: " + path;
                return false;
            }
            if (!m_sink.begin(TarEntry{path, size})) {
                m_error = m_sink.error().empty() ? "the sink refused " + path : m_sink.error();
                return false;
            }
            m_inFile = true;
            m_remaining = size;
            m_padding = (kBlock - (size % kBlock)) % kBlock;
            return true;
        case '5':      // directory
        case 'x':      // pax extended header
        case 'g':      // pax global header
        case 'V':      // GNU volume header
            m_skipped += 1;
            m_remaining = size;
            m_padding = (kBlock - (size % kBlock)) % kBlock;
            return true;
        default:
            // Links, devices, and anything else a world archive has no business
            // carrying.  Refused rather than skipped: a symlink inside a download
            // is a way out of the world directory.
            m_error = std::string("refused an archive entry of type '") + type + "': " + path;
            return false;
    }
}

bool TarReader::finish() {
    if (!m_error.empty()) return false;
    if (m_inFile || m_collectingName || m_headerUsed != 0 || m_padding != 0) {
        m_error = "the archive ended in the middle of an entry";
        return false;
    }
    if (m_files == 0) {
        m_error = "the archive contains no files";
        return false;
    }
    return true;
}

bool parseChecksumList(const std::string& text, std::map<std::string, std::string>* out) {
    if (!out) return false;
    out->clear();
    size_t start = 0;
    while (start < text.size()) {
        const size_t newline = text.find('\n', start);
        std::string line =
            text.substr(start, newline == std::string::npos ? std::string::npos : newline - start);
        start = newline == std::string::npos ? text.size() : newline + 1;
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        const size_t space = line.find(' ');
        if (space != 64 || line.size() < 67) return false;   // 64 hex, a space, a name
        std::string hex = line.substr(0, 64);
        for (char& c : hex) {
            if (c >= 'A' && c <= 'F') c = static_cast<char>(c - 'A' + 'a');
            const bool isHex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
            if (!isHex) return false;
        }
        // `sha256sum` separates the hash from the name with two spaces in text
        // mode and one in binary mode, and writes a `*` marker after it; all three
        // spellings name the same file.
        size_t start2 = space;
        while (start2 < line.size() && line[start2] == ' ') ++start2;
        std::string name = line.substr(start2);
        if (!name.empty() && name[0] == '*') name.erase(0, 1);
        if (name.empty()) return false;
        (*out)[name] = hex;
    }
    return !out->empty();
}

ExtractingSink::ExtractingSink(std::string root, std::map<std::string, std::string> expected)
    : m_root(std::move(root)), m_expected(std::move(expected)) {}

ExtractingSink::~ExtractingSink() {
    if (m_file) {
        std::fclose(m_file);
        m_file = nullptr;
        std::error_code ec;
        std::filesystem::remove(m_partPath, ec);
    }
}

void ExtractingSink::abortCurrent() {
    if (m_file) {
        std::fclose(m_file);
        m_file = nullptr;
    }
    if (!m_partPath.empty()) {
        std::error_code ec;
        std::filesystem::remove(m_partPath, ec);
    }
    m_seenBytes = 0;
    m_hash.reset();
}

bool ExtractingSink::begin(const TarEntry& entry) {
    m_seenBytes = 0;
    m_expectedBytes = entry.size;
    m_lastPath = entry.path;
    m_partPath = m_root + "/" + entry.path + ".part";
    m_finalPath = m_root + "/" + entry.path;

    if (m_expected.find(entry.path) == m_expected.end()) {
        m_rejected += 1;
        m_error = "the archive contains a file that was not published: " + entry.path;
        return false;
    }
    m_hash.reset();

    const size_t slash = m_partPath.find_last_of('/');
    if (slash != std::string::npos && !makeDirectories(m_partPath.substr(0, slash))) {
        m_error = "cannot create the directory for " + entry.path;
        return false;
    }
    m_file = std::fopen(m_partPath.c_str(), "wb");
    if (!m_file) {
        m_error = "cannot open " + m_partPath + " for writing";
        return false;
    }
    return true;
}

bool ExtractingSink::data(const u8* bytes, size_t count) {
    if (!m_file) {
        m_error = "data arrived for a file that was never opened";
        return false;
    }
    if (std::fwrite(bytes, 1, count, m_file) != count) {
        m_error = "short write into " + m_partPath;
        return false;
    }
    m_hash.update(bytes, count);
    m_seenBytes += count;
    m_bytesWritten += count;
    return true;
}

bool ExtractingSink::end() {
    if (!m_file) {
        m_error = "a file ended that was never opened";
        return false;
    }
    std::fclose(m_file);
    m_file = nullptr;

    std::error_code ec;
    const std::string hex = m_hash.digestHex();
    const auto it = m_expected.find(m_lastPath);
    if (m_seenBytes != m_expectedBytes || it == m_expected.end() || it->second != hex) {
        m_rejected += 1;
        m_error = "checksum mismatch for " + m_lastPath;
        std::filesystem::remove(m_partPath, ec);
        return false;
    }

    // The rename is the moment the file becomes part of the world, and it is
    // atomic within the directory: a kill in the middle of a download leaves
    // either a whole chunk or no chunk, never half of one.
    std::filesystem::rename(m_partPath, m_finalPath, ec);
    if (ec) {
        m_error = "cannot move the verified copy of " + m_lastPath + " into place";
        std::filesystem::remove(m_partPath, ec);
        return false;
    }

    m_filesWritten += 1;
    if (m_lastPath == kManifestName) {
        m_sawManifest = true;
        m_manifestHex = hex;
    }
    return true;
}

}  // namespace mumbai::world
