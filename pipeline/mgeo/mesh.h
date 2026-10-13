// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/mesh.h
//
//  MeshBuilder: the single place where geometry becomes bytes.  Every stage
//  (terrain, roads, buildings, water, rail, hero assets) emits through this
//  class, so the on-disk layout, normal generation, UV rules, ambient-occlusion
//  baking and LOD tagging are guaranteed identical across the whole world.
// =============================================================================
#pragma once
#include "engine/world/chunk.h"
#include "pipeline/mgeo/geometry.h"

namespace mumbai::mesh {

// Axis-aligned-box helper used for roofs, medians, kerbs and props.
struct Quad {
    Vec3 p0, p1, p2, p3;
    Vec3 normal;
};

class MeshBuilder {
public:
    // --- vertex / index emission -------------------------------------------
    u32 addVertex(const Vec3& position, const Vec3& normal, const Vec2& uv,
                  u32 materialId, u32 zoneId, u8 flags, u8 ao = 255);
    u32 addVertex(const MeshVertex& v) {
        m_vertices.push_back(v);
        return static_cast<u32>(m_vertices.size() - 1);
    }
    void addTriangle(u32 a, u32 b, u32 c);
    void addQuad(u32 a, u32 b, u32 c, u32 d);
    void addQuad(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
                 const Vec3& normal, u32 materialId, u32 zoneId, u8 flags,
                 f32 uvScale = 0.25f, u8 ao = 255);
    // Emits both faces of a quad (used for canopies, tarpaulins, signage).
    void addDoubleSidedQuad(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
                            u32 materialId, u32 zoneId, u8 flags, f32 uvScale = 0.25f);

    // --- vertical wall extrusion -------------------------------------------
    // Extrudes a closed footprint ring between two heights, emitting outward
    // facing walls with correct per-face normals and world-scaled UVs.
    void extrudeWalls(const std::vector<Vec2d>& ring, f64 baseY, f64 topY,
                      u32 materialId, u32 zoneId, u8 flags, f32 uvScale = 0.25f);
    // Roof cap (ear-clipped), with an optional parapet lip.
    void addRoofCap(const std::vector<Vec2d>& ring, f64 y, u32 materialId, u32 zoneId,
                    u8 flags, f32 uvScale = 0.25f, f64 inset = 0.0, f64 lipHeight = 0.0);

    // --- submesh bookkeeping ------------------------------------------------
    void beginSubmesh(u32 materialId, u32 zoneId);
    void endSubmesh();
    void setCurrentLod(u32 lod) { m_lod = lod; }
    u32 currentLod() const { return m_lod; }

    // --- AO baking ----------------------------------------------------------
    // Applies a cheap curvature/occlusion estimate: vertices closer to the
    // ground and inside concavities are darkened.  Baked once offline so the
    // runtime pays nothing (Section 2A asks for baked AO in the chunks).
    void bakeAmbientOcclusion(f64 groundY, f64 falloff);

    size_t vertexCount() const { return m_vertices.size(); }
    size_t indexCount() const { return m_indices.size(); }
    size_t triangleCount() const { return m_indices.size() / 3; }
    bool empty() const { return m_vertices.empty(); }

    // Serialises as a MeshGroup payload (chunk.h layout).
    std::vector<u8> serialize(u32 lodLevel) const;

    // Appends this builder's contents into a growing per-cell buffer, applying
    // a translation (world -> cell-local) and remapping material/zone ids.
    void appendTranslated(MeshBuilder& dst, const Vec3& offset) const;

    // Read access for the group serialiser and for validation tooling.
    const std::vector<MeshVertex>& vertexData() const { return m_vertices; }
    const std::vector<u32>& indexData() const { return m_indices; }
    const std::vector<SubMesh>& submeshData() const { return m_submeshes; }

    // Reserved so that callers can reserve capacity without knowing internals.
    void reserve(size_t verts, size_t indices) {
        m_vertices.reserve(verts);
        m_indices.reserve(indices);
    }

private:
    struct OpenSubmesh { u32 materialId; u32 zoneId; u32 firstIndex; u32 firstVertex; bool open = false; };
    std::vector<MeshVertex> m_vertices;
    std::vector<u32> m_indices;
    std::vector<SubMesh> m_submeshes;
    std::vector<OpenSubmesh> m_open;
    u32 m_lod = 0;
};

// Serialises a set of LOD meshes into one mesh-group payload.
std::vector<u8> serializeMeshGroup(const std::vector<const MeshBuilder*>& meshes,
                                   const std::vector<u32>& lodLevels);

} // namespace mumbai::mesh
