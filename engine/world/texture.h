// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/texture.h
//
//  The runtime side of the procedural material library (Section 1B, A8).  The
//  pipeline bakes every material's base colour, normal and ORM maps into three
//  `.mumbaitex` texture arrays; the runtime maps one file per map type and hands
//  the GPU a pointer per layer.  Nothing is decoded, resized or re-compressed at
//  load: the mip chains are built offline, so `open()` is a handful of bounds
//  checks over an mmap and a layer is resident the moment it is bound.
//
//  This header is the *contract* — the offline writer includes it too, so a
//  field cannot drift between the two sides.  Layout (little-endian, every
//  offset 8-byte aligned):
//
//      TexArrayHeader           64 bytes
//      TexLayer[layerCount]     40 bytes each
//      layer 0 mip chain        size^2*ch + (size/2)^2*ch + ... + 1*ch
//      layer 1 mip chain        ...
//
//  A layer record carries not just its bytes but what the renderer has to know
//  about them: the mean *linear* albedo (the divisor that makes a texture a
//  detail layer rather than a re-grade) and the physical size of the tile in
//  metres.  A texture array is therefore self-describing: an engine that binds
//  layer N of the base-colour array knows the material's mean brightness and how
//  many metres one UV unit covers without consulting any other file.
//
//  Validation is "trust nothing": magic, version, map type, layer count, size,
//  mip count, channel count, table bounds, per-layer mip arithmetic, alignment,
//  ordering and (on request) the content hash are all checked, and a texture
//  that fails any of them is refused rather than rendered.
// =============================================================================
#pragma once
#include "engine/core/core.h"

#include <string>
#include <vector>

namespace mumbai::tex {

// -----------------------------------------------------------------------------
//  Container constants
// -----------------------------------------------------------------------------
inline constexpr char TEXARRAY_MAGIC[8] = {'M','U','M','T','E','X','1','\0'};
inline constexpr u32 TEXARRAY_VERSION      = 1;
inline constexpr u32 TEXARRAY_HEADER_SIZE  = 64;
inline constexpr u32 TEXLAYER_SIZE         = 40;

// Bounds the runtime enforces before it trusts a header, so a corrupt count
// cannot walk the reader off the mapping.
inline constexpr u32 TEXARRAY_MAX_LAYERS = 4096;
inline constexpr u32 TEXARRAY_MAX_SIZE   = 4096;

enum class MapType : u32 {
    BaseColor = 0,     // sRGB albedo, 4 channels; mean == MaterialDef.baseColor
    Normal    = 1,     // tangent space (+Y up in a V-up UV frame), 3 channels
    ORM       = 2,     // R: AO, G: absolute roughness, B: absolute metallic
    Count
};
const char* mapTypeName(MapType t);
u32 mapTypeChannels(MapType t);

#pragma pack(push, 1)
struct TexArrayHeader {
    char magic[8];
    u32  version;
    u32  mapType;          // MapType
    u32  layerCount;
    u32  size;             // mip0 edge in pixels (layers are square and equal)
    u32  mipCount;
    u32  channels;         // 3 or 4 (normal/ORM are 3, base colour is 4)
    u64  payloadBytes;
    u64  contentHash;      // FNV-1a 64 over every layer payload byte
    u64  reserved0;
    u64  reserved1;
};  // 8 + 6*4 + 4*8 = 64 bytes

struct TexLayer {
    u64 offset;            // from the start of the file
    u64 size;              // this layer's mip chain, bytes
    u64 hash;              // FNV-1a 64 of the layer's mip chain
    f32 meanR, meanG, meanB;   // mean *linear* mip0 colour (the renderer's divisor)
    f32 tileMetres;        // metres covered by one tile of this layer
};  // 24 + 16 = 40 bytes
#pragma pack(pop)

static_assert(sizeof(TexArrayHeader) == TEXARRAY_HEADER_SIZE, "header must stay 64 bytes");
static_assert(sizeof(TexLayer) == TEXLAYER_SIZE, "layer record must stay 40 bytes");

// Edge of mip `level` for an array built at `size`.  Mirrors the writer's box
// filter exactly (halve, floor, clamp at 1) — a mismatched rule here would make
// the runtime read a layer at the wrong stride.
inline u32 nextMipEdge(u32 w) { return w > 1 ? (w / 2 > 0 ? w / 2 : 1) : 1; }
inline u32 mipEdge(u32 size, u32 level) {
    u32 w = size;
    for (u32 i = 0; i < level; ++i) w = nextMipEdge(w);
    return w;
}
inline u32 mipCountFor(u32 size) {
    u32 n = 1;
    for (u32 w = size; w > 1; w = nextMipEdge(w)) ++n;
    return n;
}
inline u64 mipChainBytes(u32 size, u32 channels) {
    u64 total = 0;
    for (u32 w = size;; w = nextMipEdge(w)) {
        total += static_cast<u64>(w) * w * channels;
        if (w == 1) break;
    }
    return total;
}

// -----------------------------------------------------------------------------
//  TexArrayReader
// -----------------------------------------------------------------------------
class TexArrayReader {
public:
    TexArrayReader() = default;
    ~TexArrayReader();
    TexArrayReader(const TexArrayReader&) = delete;
    TexArrayReader& operator=(const TexArrayReader&) = delete;

    // Maps and validates.  `verifyPayload` re-hashes every layer (~34 MB per
    // layer set — the streaming path leaves it off, the build harness on).
    bool open(const std::string& path, bool verifyPayload = false, std::string* error = nullptr);
    void close();

    bool valid() const { return m_data != nullptr; }
    const TexArrayHeader& header() const { return m_header; }
    u32 layerCount() const { return m_header.layerCount; }
    u32 size() const { return m_header.size; }
    u32 channels() const { return m_header.channels; }
    MapType type() const { return static_cast<MapType>(m_header.mapType); }

    const TexLayer* layers() const { return m_layers.data(); }
    const TexLayer* layer(u32 index) const {
        return index < m_layers.size() ? &m_layers[index] : nullptr;
    }

    // Mip 0 pixels of a layer — a view into the mapping, no copy.
    const u8* pixels(u32 index) const {
        const TexLayer* l = layer(index);
        return l ? m_data + l->offset : nullptr;
    }
    // Any mip level of a layer (level 0 == pixels()).
    const u8* mip(u32 index, u32 level) const {
        const TexLayer* l = layer(index);
        if (!l || level >= m_header.mipCount) return nullptr;
        u64 off = 0;
        for (u32 i = 0; i < level; ++i) {
            const u64 e = mipEdge(m_header.size, i);
            off += e * e * m_header.channels;
        }
        return (off + 1 <= l->size) ? m_data + l->offset + off : nullptr;
    }

    // Mean mip0 colour of a layer, normalised to 0..1.  With `linearSpace` the
    // sRGB transfer function is applied first — the space the shader works in and
    // the space the layer's stored `meanR/G/B` are in.
    bool layerMean(u32 index, f32* rgb, bool linearSpace = false) const;

    u64 mappedBytes() const { return m_fileBytes; }
    const std::string& path() const { return m_path; }

private:
    bool fail(const std::string& msg, std::string* error);

    u8* m_data = nullptr;
    u64 m_fileBytes = 0;
    std::string m_path;
    TexArrayHeader m_header{};
    std::vector<TexLayer> m_layers;
};

// Mean colour of a pixel block in linear space, normalised to 0..1 — the value
// the writer stores in a layer record and the export publishes as `meanLinear`.
void meanLinearRgb(const u8* pixels, u32 size, u32 channels, f32* rgb);

// FNV-1a 64, the hash both the writer and the reader use.
inline u64 fnv1a64(const void* data, size_t bytes, u64 seed = 1469598103934665603ull) {
    const u8* p = static_cast<const u8*>(data);
    u64 h = seed;
    for (size_t i = 0; i < bytes; ++i) h = (h ^ p[i]) * 1099511628211ull;
    return h;
}

}  // namespace mumbai::tex
