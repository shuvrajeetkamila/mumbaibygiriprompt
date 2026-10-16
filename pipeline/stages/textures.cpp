// =============================================================================
//  pipeline/stages/textures.cpp — A8: procedural PBR material content
//
//  Generates the texture library *after* the last material has been created —
//  which is after buildBuildings, because facade variants are minted per
//  building — and writes:
//
//    data/processed/textures/materials_basecolor.mumbaitex   (mmap-able array)
//    data/processed/textures/materials_normal.mumbaitex
//    data/processed/textures/materials_orm.mumbaitex
//    data/processed/textures/atlas_basecolor.png             (browser preview)
//    data/processed/textures/materials.png                   (full contact sheet)
//    data/processed/textures/texture_manifest.json           (layer -> material)
//
//  and publishes the per-material UV scale table the mesh layer divides its UVs
//  by, so that every surface in the city samples its material at its physical
//  tile size.  The stage is deterministic: same inputs, same bytes.
// =============================================================================
#include "pipeline/pipeline.h"
#include "pipeline/mgeo/texture.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace mumbai::pipeline {

namespace {

struct FamilyTally { std::string family; u32 count = 0; };

// Builds the non-square atlas the browser preview (and any quick image viewer)
// can load: every material tile laid out on one page, `tile` pixels each.  The
// runtime never reads this; the .mumbaitex arrays are the shipping format.
std::vector<u8> buildAtlas(const std::vector<const std::vector<u8>*>& tiles, u32 srcSize,
                           u32 tile, u32 columns, u32* outW, u32* outH) {
    const u32 rows = static_cast<u32>((tiles.size() + columns - 1) / columns);
    const u32 W = columns * tile, H = rows * tile;
    std::vector<u8> img(static_cast<size_t>(W) * H * 4, 0);
    for (size_t i = 0; i < tiles.size(); ++i) {
        const u32 cx = static_cast<u32>(i) % columns;
        const u32 cy = static_cast<u32>(i) / columns;
        for (u32 y = 0; y < tile; ++y) {
            for (u32 x = 0; x < tile; ++x) {
                const u32 su = static_cast<u32>((static_cast<f64>(x) + 0.5) / tile * srcSize) % srcSize;
                const u32 sv = static_cast<u32>((static_cast<f64>(y) + 0.5) / tile * srcSize) % srcSize;
                const u8* s = &(*tiles[i])[(static_cast<size_t>(sv) * srcSize + su) * 4];
                u8* d = &img[(static_cast<size_t>(cy * tile + y) * W + cx * tile + x) * 4];
                d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = s[3];
            }
        }
    }
    *outW = W; *outH = H;
    return img;
}

}  // namespace

bool buildTextures(Context& ctx) {
    const BuildConfig& cfg = ctx.cfg;
    const u32 size = std::max<u32>(64, cfg.textureSize);
    const std::string dir = pathJoin(cfg.outDir, "textures");
    ensureDir(dir);

    const std::vector<MaterialDef>& defs = ctx.materials.all();
    tex::Generator gen(size);

    std::vector<tex::TextureSet> sets(defs.size());
    std::vector<std::vector<u8>> baseColorLayers, normalLayers, ormLayers;
    std::vector<std::string> names;
    std::vector<u32> layerMaterial;             // material id per array layer
    std::vector<std::string> contactNames;
    std::vector<const std::vector<u8>*> contactTiles;
    std::unordered_map<std::string, u32> familyCount;
    f64 meanError = 0.0;
    f64 tileMin = 1e9, tileMax = 0.0;
    u32 textured = 0, unmapped = 0;

    const f64 t0 = ScopedTimer::now();
    for (u32 i = 0; i < defs.size(); ++i) {
        if (i == 0) continue;                       // id 0 is "unknown"
        const std::string name = defs[i].name;
        if (name.empty() || name == "unknown") continue;
        tex::TextureSet set = gen.make(name, defs[i].roughness, defs[i].metallic,
                                       defs[i].baseColor);
        sets[i] = std::move(set);
        const tex::TextureSet& s = sets[i];
        baseColorLayers.push_back(s.baseColor);
        normalLayers.push_back(s.normal);
        ormLayers.push_back(s.orm);
        names.push_back(name);
        layerMaterial.push_back(i);
        ctx.materials.setTextureSet(i, static_cast<u32>(baseColorLayers.size() - 1), s.tileMetres);
        ++textured;

        const std::string fam = tex::familyFor(name);
        ++familyCount[fam];
        if (fam == "unknown") ++unmapped;
        tileMin = std::min(tileMin, static_cast<f64>(s.tileMetres));
        tileMax = std::max(tileMax, static_cast<f64>(s.tileMetres));

        // --- the invariant this stage exists to keep -------------------------
        // The tile's mean linear albedo must equal the MaterialDef's albedo: the
        // texture is a detail layer, not a change of material.  Measure it back
        // off the bytes actually written, not off the generator's intent.
        f64 mr = 0, mg = 0, mb = 0;
        const size_t n = static_cast<size_t>(size) * size;
        for (size_t p = 0; p < n; ++p) {
            auto toLin = [](u8 c) {
                const f32 v = c / 255.0f;
                return v <= 0.04045f ? v / 12.92f : static_cast<f32>(std::pow((v + 0.055f) / 1.055f, 2.4f));
            };
            mr += toLin(s.baseColor[p * 4 + 0]);
            mg += toLin(s.baseColor[p * 4 + 1]);
            mb += toLin(s.baseColor[p * 4 + 2]);
        }
        mr /= n; mg /= n; mb /= n;
        auto rel = [](f64 got, f32 want) {
            const f64 w = std::max<f64>(1e-4, want);
            return std::fabs(got - w) / w;
        };
        const f64 err = std::max({rel(mr, defs[i].baseColor[0]), rel(mg, defs[i].baseColor[1]),
                                  rel(mb, defs[i].baseColor[2])});
        meanError = std::max(meanError, err);
        // Publish the layer's mean so the export can hand the shader a divisor
        // that makes the tile a pure detail modulation (mean 1).
        ctx.materials.setTextureLayerMean(static_cast<u32>(baseColorLayers.size() - 1),
                                          static_cast<f32>(mr), static_cast<f32>(mg),
                                          static_cast<f32>(mb));
    }

    if (textured == 0 || baseColorLayers.empty()) {
        ctx.qa.fail("texture generation produced no layers");
        return false;
    }

    // --- arrays ------------------------------------------------------------
    std::vector<tex::LayerInput> bc, nm, orm;
    for (size_t i = 0; i < baseColorLayers.size(); ++i) {
        const f32 tm = ctx.materials.tileMetres(layerMaterial[i]);
        bc.push_back({&baseColorLayers[i], tm});
        nm.push_back({&normalLayers[i], tm});
        orm.push_back({&ormLayers[i], tm});
    }
    const std::string bcPath = pathJoin(dir, "materials_basecolor.mumbaitex");
    const std::string nmPath = pathJoin(dir, "materials_normal.mumbaitex");
    const std::string ormPath = pathJoin(dir, "materials_orm.mumbaitex");
    // Each layer records its own physical tile size and mean linear albedo, so
    // the array is self-describing: a runtime binding layer N knows how many
    // metres one UV unit covers and what to divide the sample by.  The writer
    // measures the mean from the pixels, so the record cannot claim something the
    // content does not do.
    if (!tex::writeTexArray(bcPath, tex::MapType::BaseColor, size, bc) ||
        !tex::writeTexArray(nmPath, tex::MapType::Normal, size, nm) ||
        !tex::writeTexArray(ormPath, tex::MapType::ORM, size, orm)) {
        ctx.qa.fail("texture array write failed");
        return false;
    }
    u64 bytes = 0;
    for (const std::string& p : {bcPath, nmPath, ormPath}) bytes += fileSize(p);

    // --- browser atlas + review contact sheet ------------------------------
    const u32 atlasTile = std::min<u32>(128, size);
    ctx.materials.setTextureLayerCount(textured);
    ctx.materials.setTextureAtlasTile(atlasTile);
    const u32 atlasCols = static_cast<u32>(std::ceil(std::sqrt(static_cast<f64>(textured))));
    u32 aw = 0, ah = 0;
    std::vector<const std::vector<u8>*> bcPtr, nmPtr, ormPtr;
    for (size_t i = 0; i < bc.size(); ++i) {
        bcPtr.push_back(bc[i].pixels);
        nmPtr.push_back(nm[i].pixels);
        ormPtr.push_back(orm[i].pixels);
    }
    const std::vector<u8> atlas = buildAtlas(bcPtr, size, atlasTile, atlasCols, &aw, &ah);
    tex::writePng(pathJoin(dir, "atlas_basecolor.png"), atlas.data(), aw, ah, 4);
    // Normal + ORM atlases too: they are what makes a material review possible
    // without the renderer (a flat normal map is instantly visible as a bug).
    std::vector<const std::vector<u8>*> nm4, orm4;
    std::vector<std::vector<u8>> nmP4, ormP4;
    nmP4.reserve(nm.size()); ormP4.reserve(orm.size());
    for (size_t i = 0; i < nmPtr.size(); ++i) {
        std::vector<u8> t(static_cast<size_t>(size) * size * 4);
        for (size_t p = 0; p < static_cast<size_t>(size) * size; ++p) {
            t[p * 4 + 0] = (*(nmPtr[i]))[p * 3 + 0];
            t[p * 4 + 1] = (*(nmPtr[i]))[p * 3 + 1];
            t[p * 4 + 2] = (*(nmPtr[i]))[p * 3 + 2];
            t[p * 4 + 3] = 255;
        }
        nmP4.push_back(std::move(t));
        std::vector<u8> o(static_cast<size_t>(size) * size * 4);
        for (size_t p = 0; p < static_cast<size_t>(size) * size; ++p) {
            o[p * 4 + 0] = (*(ormPtr[i]))[p * 3 + 0];
            o[p * 4 + 1] = (*(ormPtr[i]))[p * 3 + 1];
            o[p * 4 + 2] = (*(ormPtr[i]))[p * 3 + 2];
            o[p * 4 + 3] = 255;
        }
        ormP4.push_back(std::move(o));
        nm4.push_back(&nmP4.back());
        orm4.push_back(&ormP4.back());
    }
    const std::vector<u8> nmAtlas = buildAtlas(nm4, size, atlasTile, atlasCols, &aw, &ah);
    tex::writePng(pathJoin(dir, "atlas_normal.png"), nmAtlas.data(), aw, ah, 4);
    const std::vector<u8> ormAtlas = buildAtlas(orm4, size, atlasTile, atlasCols, &aw, &ah);
    tex::writePng(pathJoin(dir, "atlas_orm.png"), ormAtlas.data(), aw, ah, 4);

    for (size_t i = 0; i < bcPtr.size(); ++i) contactTiles.push_back(bcPtr[i]);
    contactNames = names;
    const std::string sheet = pathJoin(dir, "materials.png");
    tex::writeContactSheet(sheet, contactNames, contactTiles, std::min<u32>(128, size), 8,
                           "MUMBAI 1:1 - procedural material library (base colour)");

    // --- manifest ----------------------------------------------------------
    Json j = Json::object();
    j["format"] = Json("mumbai-textures");
    j["formatVersion"] = Json(1);
    j["tileSize"] = Json(static_cast<i64>(size));
    j["materialCount"] = Json(static_cast<i64>(textured));
    Json arr = Json::array();
    {
        u32 layer = 0;
        for (u32 i = 0; i < defs.size(); ++i) {
            if (sets[i].name.empty()) continue;
            ++layer;
            Json e = Json::object();
            // `layer` is the 0-based array/atlas index the runtime uses (`tex` in
            // the viewer manifest is the same number); `layerNumber` is the human
            // count.  They differ by one, deliberately and visibly.
            e["layer"] = Json(static_cast<i64>(layer - 1));
            e["layerNumber"] = Json(static_cast<i64>(layer));
            e["materialId"] = Json(static_cast<i64>(i));
            e["name"] = Json(sets[i].name);
            e["family"] = Json(tex::familyFor(sets[i].name));
            e["tileMetres"] = Json(static_cast<f64>(sets[i].tileMetres));
            // The authored appearance the tile must reproduce, published here so
            // the runtime (and the test suite) can verify mean preservation from
            // the shipped bytes without rebuilding the material library.
            Json alb = Json::array();
            for (u32 c = 0; c < 3; ++c) alb.push(Json(static_cast<f64>(defs[i].baseColor[c])));
            e["albedo"] = alb;
            e["roughness"] = Json(static_cast<f64>(defs[i].roughness));
            e["metallic"] = Json(static_cast<f64>(defs[i].metallic));
            arr.push(e);
        }
    }
    j["layers"] = arr;
    j["arrayBytes"] = Json(static_cast<f64>(bytes));
    writeFile(pathJoin(dir, "texture_manifest.json"), j.dump(1));

    ctx.qa.add("textures.materials", textured);
    ctx.qa.add("textures.layers", baseColorLayers.size());
    ctx.qa.add("textures.families", familyCount.size());
    ctx.qa.add("textures.unmappedFamilies", unmapped, unmapped ? 1u : 0u);
    ctx.qa.add("textures.tileMetresMin", tileMin);
    ctx.qa.add("textures.tileMetresMax", tileMax);
    ctx.qa.add("textures.meanAlbedoError", meanError * 100.0, meanError > 0.02 ? 2u : 0u);
    ctx.qa.add("textures.arrayBytes", static_cast<f64>(bytes));
    // A repair the shipper made, published rather than silent: how many layers were
    // shipped with a constant albedo and had to be given a fine grain (D-041).
    ctx.qa.add("textures.flatAlbedoRepairs", static_cast<f64>(tex::flatAlbedoRepairs()));
    MLOG_INFO("textures: %u of %zu layers needed the flat-albedo repair",
              tex::flatAlbedoRepairs(), baseColorLayers.size());
    ctx.qa.addTiming("textures.buildSeconds", ScopedTimer::now() - t0);
    if (meanError > 0.02)
        ctx.qa.fail("texture mean albedo deviates from the material by more than 2 %");

    MLOG_INFO("textures: %u materials, %u families, %u px tiles, %.1f MB of arrays "
              "(mean albedo error %.2f %%)",
              textured, static_cast<u32>(familyCount.size()), size,
              static_cast<f64>(bytes) / 1e6, meanError * 100.0);
    return true;
}

}  // namespace mumbai::pipeline
