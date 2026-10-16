// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/world_index.cpp
// =============================================================================
#include "engine/render/world_index.h"

#include "engine/core/geo.h"

#include "engine/core/json.h"

#include <algorithm>

namespace mumbai::render {
namespace {

f32 cellCentreX(const WorldCell& c) {
    return static_cast<f32>(c.cellX) * MACRO_CELL_SIZE + MACRO_CELL_SIZE * 0.5f;
}

f32 cellCentreZ(const WorldCell& c) {
    return static_cast<f32>(c.cellZ) * MACRO_CELL_SIZE + MACRO_CELL_SIZE * 0.5f;
}

}  // namespace

bool readWorldManifest(const std::string& dir, std::vector<WorldCell>& out, std::string* error) {
    out.clear();
    const std::string path = dir + "/world_manifest.json";
    const std::string text = readFileToString(path);
    if (text.empty()) {
        if (error) *error = "no world manifest at " + path;
        return false;
    }
    Json manifest;
    std::string parseError;
    if (!Json::parse(text, manifest, &parseError)) {
        if (error) *error = "world manifest: " + parseError;
        return false;
    }
    for (const Json& c : manifest["cells"].items()) {
        WorldCell cell;
        cell.file = c["file"].asString();
        if (cell.file.empty()) continue;
        cell.cellX = c["x"].asInt(0);
        cell.cellZ = c["z"].asInt(0);
        cell.bytes = static_cast<u32>(std::max<i64>(0, c["bytes"].asInt(0)));
        out.push_back(std::move(cell));
    }
    if (out.empty()) {
        if (error) *error = "world manifest at " + path + " lists no cells";
        return false;
    }
    return true;
}

u32 countCellsInRadius(const std::vector<WorldCell>& cells, Vec3 focusWorld, f32 radiusMetres) {
    u32 n = 0;
    const f32 r2 = radiusMetres * radiusMetres;
    for (const WorldCell& c : cells) {
        const f32 dx = cellCentreX(c) - focusWorld.x;
        const f32 dz = cellCentreZ(c) - focusWorld.z;
        if (dx * dx + dz * dz <= r2) ++n;
    }
    return n;
}

u32 selectCellsToLoad(const std::vector<WorldCell>& cells, const std::vector<std::string>& resident,
                      Vec3 focusWorld, f32 radiusMetres, u32 maxCells, std::vector<u32>& out) {
    if (maxCells == 0) return 0;
    struct Candidate {
        f32 dist2;
        u32 index;
    };
    // Reserved once per call rather than grown per cell: this runs at 1 Hz for the
    // length of a drive.
    std::vector<Candidate> candidates;
    candidates.reserve(cells.size());
    const f32 r2 = radiusMetres * radiusMetres;
    for (u32 i = 0; i < cells.size(); ++i) {
        const f32 dx = cellCentreX(cells[i]) - focusWorld.x;
        const f32 dz = cellCentreZ(cells[i]) - focusWorld.z;
        const f32 dist2 = dx * dx + dz * dz;
        if (dist2 > r2) continue;
        if (std::find(resident.begin(), resident.end(), cells[i].file) != resident.end()) continue;
        candidates.push_back({dist2, i});
    }
    // Ties broken by index so the choice is deterministic: a client that loads a
    // different cell on each run for the same camera would make every bug report
    // unreproducible.
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.dist2 != b.dist2) return a.dist2 < b.dist2;
        return a.index < b.index;
    });
    const u32 n = std::min<u32>(maxCells, static_cast<u32>(candidates.size()));
    for (u32 i = 0; i < n; ++i) out.push_back(candidates[i].index);
    return n;
}

}  // namespace mumbai::render
