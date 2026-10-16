// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/world_index.h
//
//  WHERE THE CELLS ARE, AND WHICH ONES TO LOAD NEXT.
//
//  This is deliberately Vulkan-free and separate from the renderer: it is the
//  policy half of streaming — parse the manifest, measure distance, choose the
//  nearest cells that fit a budget — and policy is exactly what can be verified
//  on a machine with no GPU.  The renderer keeps the other half: turning a chosen
//  cell into buffers.
//
//  Three rules are encoded here, and each one is a decision rather than an
//  implementation detail:
//
//  * **Nearest first, and only within a radius.**  The radius is the client's
//    statement of how far it can see; the sort makes the budget spend itself on
//    what is in front of the camera rather than on what the directory happens to
//    list first.
//  * **A per-call budget.**  A call that loads 924 cells blocks a phone for
//    seconds; a call that loads four costs a few milliseconds.  The hitch-free
//    streaming requirement is met by never doing the second thing.
//  * **Resident cells are skipped, not re-loaded.**  Re-adding a cell that is
//    already in the scene is the classic silent leak: the buffers are allocated
//    again and the old ones are never freed.
// =============================================================================
#pragma once
#include "engine/core/core.h"
#include "engine/core/math.h"

#include <string>
#include <vector>

namespace mumbai::render {

struct WorldCell {
    i64 cellX = 0, cellZ = 0;   // macro-cell indices
    u32 bytes = 0;              // the manifest's own size, for reporting
    std::string file;           // basename inside <dir>/cells
};

// Reads `<dir>/world_manifest.json`.  Returns false with a reason for a missing
// or unparseable manifest — the client reports both differently.
bool readWorldManifest(const std::string& dir, std::vector<WorldCell>& out, std::string* error);

// Cells whose *centre* is within the radius of the focus point, in world metres.
u32 countCellsInRadius(const std::vector<WorldCell>& cells, Vec3 focusWorld, f32 radiusMetres);

// The next cells to load: nearest first, no more than `maxCells`, skipping any
// name in `resident`.  Appends to `out` and returns how many it added.
u32 selectCellsToLoad(const std::vector<WorldCell>& cells, const std::vector<std::string>& resident,
                      Vec3 focusWorld, f32 radiusMetres, u32 maxCells, std::vector<u32>& out);

}  // namespace mumbai::render
