// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/texture.cpp
//
//  mmap reader and validator for `.mumbaitex` texture arrays.  See
//  engine/world/texture.h for the container contract.
// =============================================================================
#include "engine/world/texture.h"

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mumbai::tex {

const char* mapTypeName(MapType t) {
    switch (t) {
        case MapType::BaseColor: return "basecolor";
        case MapType::Normal:    return "normal";
        case MapType::ORM:       return "orm";
        default:                 return "unknown";
    }
}

u32 mapTypeChannels(MapType t) {
    switch (t) {
        case MapType::BaseColor: return 4;
        case MapType::Normal:    return 3;
        case MapType::ORM:       return 3;
        default:                 return 0;
    }
}

void meanLinearRgb(const u8* pixels, u32 size, u32 channels, f32* rgb) {
    if (!pixels || !rgb || channels < 3) return;
    const u64 texels = static_cast<u64>(size) * size;
    auto toLinear = [](u8 c) {
        const f32 v = c / 255.0f;
        return v <= 0.04045f ? v / 12.92f
                             : static_cast<f32>(std::pow((v + 0.055) / 1.055, 2.4));
    };
    f64 r = 0, g = 0, b = 0;
    for (u64 i = 0; i < texels; ++i) {
        r += toLinear(pixels[i * channels + 0]);
        g += toLinear(pixels[i * channels + 1]);
        b += toLinear(pixels[i * channels + 2]);
    }
    rgb[0] = static_cast<f32>(r / static_cast<f64>(texels));
    rgb[1] = static_cast<f32>(g / static_cast<f64>(texels));
    rgb[2] = static_cast<f32>(b / static_cast<f64>(texels));
}

TexArrayReader::~TexArrayReader() { close(); }

void TexArrayReader::close() {
    if (m_data) {
        ::munmap(m_data, static_cast<size_t>(m_fileBytes));
        m_data = nullptr;
    }
    m_fileBytes = 0;
    m_path.clear();
    m_header = TexArrayHeader{};
    m_layers.clear();
}

bool TexArrayReader::fail(const std::string& msg, std::string* error) {
    const std::string keep = m_path;
    if (error) *error = msg + " [" + keep + "]";
    close();
    m_path = keep;
    return false;
}

bool TexArrayReader::open(const std::string& path, bool verifyPayload, std::string* error) {
    close();
    m_path = path;

    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return fail("cannot open texture array", error);

    struct stat st {};
    if (::fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(TEXARRAY_HEADER_SIZE)) {
        ::close(fd);
        return fail("texture array too small", error);
    }
    m_fileBytes = static_cast<u64>(st.st_size);

    void* p = ::mmap(nullptr, static_cast<size_t>(m_fileBytes), PROT_READ, MAP_PRIVATE, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) {
        m_fileBytes = 0;
        return fail("mmap failed", error);
    }
    // Read once per launch, front to back (the streaming system re-advises).
    ::madvise(p, static_cast<size_t>(m_fileBytes), MADV_WILLNEED);
    m_data = static_cast<u8*>(p);

    std::memcpy(&m_header, m_data, sizeof(TexArrayHeader));
    if (std::memcmp(m_header.magic, TEXARRAY_MAGIC, 8) != 0) return fail("bad magic", error);
    if (m_header.version != TEXARRAY_VERSION) return fail("version mismatch", error);
    if (m_header.mapType >= static_cast<u32>(MapType::Count)) return fail("unknown map type", error);
    if (m_header.layerCount == 0 || m_header.layerCount > TEXARRAY_MAX_LAYERS)
        return fail("layer count out of range", error);
    if (m_header.size == 0 || m_header.size > TEXARRAY_MAX_SIZE)
        return fail("layer size out of range", error);
    if (m_header.size & (m_header.size - 1)) return fail("layer size is not a power of two", error);
    if (m_header.channels != mapTypeChannels(static_cast<MapType>(m_header.mapType)))
        return fail("channel count does not match map type", error);
    if (m_header.mipCount != mipCountFor(m_header.size)) return fail("mip count mismatch", error);

    const u64 tableBytes = static_cast<u64>(m_header.layerCount) * TEXLAYER_SIZE;
    if (TEXARRAY_HEADER_SIZE + tableBytes > m_fileBytes)
        return fail("layer table overruns file", error);

    m_layers.resize(m_header.layerCount);
    std::memcpy(m_layers.data(), m_data + TEXARRAY_HEADER_SIZE, static_cast<size_t>(tableBytes));

    // Every layer must carry exactly one mip chain, lie inside the file, start on
    // an 8-byte boundary (so a runtime can upload straight out of the mapping)
    // and follow its predecessor.
    const u64 expected = mipChainBytes(m_header.size, m_header.channels);
    u64 span = 0, previousEnd = TEXARRAY_HEADER_SIZE + tableBytes;
    for (const TexLayer& l : m_layers) {
        if (l.size != expected) return fail("layer mip chain is the wrong size", error);
        if (l.offset % 8 != 0) return fail("layer misaligned", error);
        if (l.offset < previousEnd) return fail("layers overlap or are out of order", error);
        if (l.offset + l.size > m_fileBytes) return fail("layer out of bounds", error);
        if (!(l.tileMetres > 0.0f) || !(l.tileMetres < 4096.0f))
            return fail("layer tile size out of range", error);
        previousEnd = l.offset + l.size;
        span += l.size;
    }
    if (span != m_header.payloadBytes) return fail("payload byte count mismatch", error);
    // Between aligned layers there may be up to 7 bytes of pad, so the file is
    // "table + payload + alignment": measure the real end and allow only a final
    // alignment sliver beyond it.
    if (previousEnd > m_fileBytes) return fail("file is truncated", error);
    if (m_fileBytes - previousEnd >= 8) return fail("file has trailing bytes", error);

    if (verifyPayload) {
        // The array hash runs over the layer *payloads* in order — the alignment
        // padding between them is not content, and hashing the file's raw
        // interior would make the value depend on the padding rule.
        u64 contentHash = 1469598103934665603ull;
        for (const TexLayer& l : m_layers) {
            const u64 h = fnv1a64(m_data + l.offset, static_cast<size_t>(l.size));
            if (h != l.hash) return fail("layer content hash mismatch", error);
            contentHash = fnv1a64(m_data + l.offset, static_cast<size_t>(l.size), contentHash);
        }
        if (contentHash != m_header.contentHash)
            return fail("array content hash mismatch", error);
    }
    return true;
}

bool TexArrayReader::layerMean(u32 index, f32* rgb, bool linearSpace) const {
    const TexLayer* l = layer(index);
    if (!l || !rgb || m_header.channels < 3) return false;
    if (linearSpace) {
        meanLinearRgb(m_data + l->offset, m_header.size, m_header.channels, rgb);
        return true;
    }
    const u8* px = m_data + l->offset;
    const u64 texels = static_cast<u64>(m_header.size) * m_header.size;
    const u32 ch = m_header.channels;
    f64 r = 0, g = 0, b = 0;
    for (u64 i = 0; i < texels; ++i) {
        r += px[i * ch + 0];
        g += px[i * ch + 1];
        b += px[i * ch + 2];
    }
    rgb[0] = static_cast<f32>(r / texels / 255.0);
    rgb[1] = static_cast<f32>(g / texels / 255.0);
    rgb[2] = static_cast<f32>(b / texels / 255.0);
    return true;
}

}  // namespace mumbai::tex
