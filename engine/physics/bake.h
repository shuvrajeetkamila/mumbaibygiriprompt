// =============================================================================
//  Mumbai 1:1 EXACT — engine/physics/bake.h
//
//  Turning the built world into physics geometry.  There is no second authoring
//  of collision in this project: the world build already wrote everything a
//  physics engine needs, and this reads it through the same mmap'd reader the
//  runtime uses.
//
//    BuildingCollision  CollisionBox records — buildings, walls, hero meshes,
//                       the Sea Link deck slabs, its pylon legs, and the raked
//                       cable stays as oriented capsules (A2).
//    TerrainHeight      the per-cell heightfield, as a Jolt HeightFieldShape:
//                       ground everywhere, including under the sea.
//    RoadMesh           the carriageway/kerb/sidewalk triangles, as a MeshShape:
//                       this is the surface a vehicle actually drives on.  A
//                       city whose vehicles drive on the heightfield would have
//                       them sink a metre into the road and climb every kerb.
// =============================================================================
#pragma once

#include "engine/core/core.h"
#include "engine/core/math.h"
#include "engine/physics/world.h"

#include <string>
#include <vector>

namespace mumbai::phys {

struct BakeOptions {
    bool boxes = true;            // BuildingCollision compounds
    bool terrain = true;          // heightfield ground
    bool roads = false;           // road mesh surfaces (expensive: real triangles)
    bool roadMeshOnly = false;    // roads without the heightfield under them
    u32 maxBoxesPerBody = 1024;   // compounds are split at this size
    u32 maxBoxes = 400000;        // hard cap: refuses to build a runaway world
};

struct BakeStats {
    u64 chunksRead = 0;
    u64 boxes = 0;
    u64 capsules = 0;
    u64 staticBodies = 0;
    u64 compoundGroups = 0;
    u64 terrainShapes = 0;
    u64 roadShapes = 0;
    u64 roadTriangles = 0;
    u64 skippedChunks = 0;
    f64 seconds = 0.0;
    f64 minY = 1e30, maxY = -1e30;
    std::string firstError;
};

// Loads one `.mumbai_chunk` file into the physics world.  Returns false (with
// `stats.firstError` set) only for an unreadable file; a chunk with no collision
// data is not an error, it is an empty cell.
bool bakeChunk(World& world, const std::string& path, const BakeOptions& options,
               BakeStats& stats);

// The header of a chunk, without touching its payload: how the harness builds
// an index of the world (which cell a world point falls in, and the height band
// of that cell) without opening the geometry.
struct ChunkInfo {
    i64 cellX = 0, cellZ = 0;
    f32 cellSize = MACRO_CELL_SIZE;
    f32 minHeight = 0, maxHeight = 0;
    u64 bytes = 0;
};
bool readChunkInfo(const std::string& path, ChunkInfo& out);

// Loads every `*.mumbai_chunk` file in a directory — `data/processed/cells/`,
// where the world build writes them.  Sorted for determinism: the order bodies
// are created in must not depend on the filesystem.
bool bakeDirectory(World& world, const std::string& dir, const BakeOptions& options,
                   BakeStats& stats);

}  // namespace mumbai::phys
