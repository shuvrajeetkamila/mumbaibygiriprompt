// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/texture.h
//
//  Procedural PBR material content (Section 1B / Section 2A).  The world has
//  carried material *parameters* since the format was defined — albedo,
//  roughness, metallic, the wetness response the monsoon drives — but no
//  textures, because the offline tier cannot ship scanned maps it has no right
//  to and cannot download them either.  This module synthesises them instead:
//  every material in the library gets a tileable base-colour map, a normal map
//  derived from a height field, and an ORM map, all deterministic from the
//  material's name and the build seed.
//
//  Two rules make the textures *content* rather than decoration:
//
//   * **The mean is preserved.**  The tile's average albedo equals the albedo the
//     MaterialDef already carried, and the ORM green channel is *absolute*
//     roughness (glTF semantics) whose mean is the authored roughness.  Adding
//     textures to the world therefore changes its detail, not its tone, and the
//     wetness response, the baked AO and the tonemap keep the calibration they
//     were verified with.
//   * **The tile size is physical.**  A material declares how many metres one
//     tile covers (`tileMetres`), and the mesh builder scales UVs by that, so
//     brick courses are brick-sized on every facade in the city and asphalt
//     grain is asphalt-sized on every road, whatever their extents.
//
//  Containers
//  ----------
//  * `.mumbaitex` — a flat, mmap-able texture array: one map type, N square
//    layers of equal size, each with a box-filtered mip chain written at build
//    time so the runtime never builds one at load.  Layout:
//        TexArrayHeader | TexLayer[layerCount] | layer payloads (mip chain)
//  * PNG — for the browser preview and for review.  Written with *stored*
//    deflate blocks: a valid PNG that needs no compressor, no dependency and no
//    licensing, and whose bytes are byte-identical across machines.  It is a
//    review format here, not the shipping format; the runtime reads
//    `.mumbaitex`.
// =============================================================================
#pragma once
#include "engine/core/core.h"
#include "engine/world/texture.h"   // the .mumbaitex container, shared with the runtime

#include <string>
#include <vector>

namespace mumbai::tex {

// The `.mumbaitex` container (TexArrayHeader / TexLayer / MapType / the mmap
// reader the runtime uses) is defined in engine/world/texture.h, included above.

// -----------------------------------------------------------------------------
//  A synthesised material appearance
// -----------------------------------------------------------------------------
struct TextureSet {
    std::string name;
    f32 tileMetres = 4.0f;          // metres covered by one tile
    std::vector<u8> baseColor;      // size*size*4, sRGB
    std::vector<u8> normal;         // size*size*3, tangent space
    std::vector<u8> orm;            // size*size*3
};

// -----------------------------------------------------------------------------
//  Generator
// -----------------------------------------------------------------------------
//  `size` is the mip0 edge in pixels.  Every map for every material is generated
//  from the material's name alone (FNV-1a of the name seeds the noise), so two
//  builds of the same world produce byte-identical textures and a texture can be
//  regenerated in isolation for review.
class Generator {
public:
    explicit Generator(u32 size = 256);

    TextureSet make(const std::string& name, f32 roughness, f32 metallic,
                    const f32 baseColor[4]);

private:
    u32 m_size;
};

// The physical tile size a material's UVs should use, in metres.  It is derived
// from the material's *name*, so it is available for every material in the
// library — including the facade variants that are minted per building long
// after the base library was built — and it is what the mesh layer divides
// world metres by to get UVs.
f32 tileMetresFor(const std::string& materialName);

// Family name used by the recipes ("asphalt", "brick", ...).  Exposed for the
// QA report: it is how the library proves every material was covered.
const char* familyFor(const std::string& materialName);

// -----------------------------------------------------------------------------
//  Writers
// -----------------------------------------------------------------------------
// Appends the box-filtered mip chain of `pixels` (size×size×channels) to `out`.
void appendMipChain(std::vector<u8>& out, const u8* pixels, u32 size, u32 channels);

// One layer of a texture array: mip0 pixels (size×size×channels) and the physical
// size of one tile.  The mip chain, the layer's content hash and the layer's mean
// linear albedo are all derived from the pixels by the writer — a caller cannot
// record metadata the content does not have.
struct LayerInput {
    const std::vector<u8>* pixels = nullptr;
    f32 tileMetres = 1.0f;
};

// Writes an mmap-able texture array (see engine/world/texture.h for the layout).
bool writeTexArray(const std::string& path, MapType type, u32 size,
                   const std::vector<LayerInput>& layers);

// Writes a PNG using stored deflate blocks (no compression, no dependency).
// `channels` may be 1 (grey), 2 (grey+alpha), 3 (RGB) or 4 (RGBA).
bool writePng(const std::string& path, const u8* pixels, u32 width, u32 height,
              u32 channels);

// Composes a labelled contact sheet: `tiles` laid out `columns` wide, each
// scaled to `tile` pixels, with a 5x7 caption under each.  This is the review
// artifact — the only way to look at a procedural material library without a
// renderer.
bool writeContactSheet(const std::string& path, const std::vector<std::string>& names,
                       const std::vector<const std::vector<u8>*>& tiles,
                       u32 tile, u32 columns, const char* title);

// How many shipped layers needed the flat-albedo repair (a layer whose albedo field
// was constant and got a fine grain at the authored mean).  Exposed so the pipeline
// can publish it: a repair nobody counts is a silent change to a material (D-041).
u32 flatAlbedoRepairs();

}  // namespace mumbai::tex
