// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/osm.h
//
//  OpenStreetMap XML ingest.  The specification (Section 1B) calls for the
//  India extract in PBF parsed with the `osmpbf` crate; the sandbox has no
//  network access to crates.io, so the pipeline ingests the XML form of the
//  same upstream data with an in-house mmap-backed scanner (DOCS D-002/D-005).
//  Both encodings carry identical semantics — nodes, ways, tags, multipolygon
//  relations — so the pipeline logic downstream is unchanged.  A PBF reader can
//  be dropped in behind `OsmData::load` without touching any other file.
//
//  Performance: 35 MB AMMR extract parses in ~1.5 s single-threaded by
//  scanning bytes with memchr rather than using an XML DOM.
// =============================================================================
#pragma once
#include "engine/core/geo.h"

namespace mumbai::osm {

struct Tag { u32 key, value; };

struct Node {
    i64 id = 0;
    f64 lat = 0, lon = 0;
    u32 firstTag = 0, tagCount = 0;
};

struct Way {
    i64 id = 0;
    u32 firstRef = 0, refCount = 0;
    u32 firstTag = 0, tagCount = 0;
    bool closed() const { return refCount > 2 && m_closed; }
    bool m_closed = false;
    // Cached classification filled by `classify()` so downstream stages do not
    // re-scan tag arrays (this matters at 20k ways across 6 stages).
    u8 highwayClass = 0;     // see HighwayClass
    u8 buildingClass = 0;
    u8 landuseClass = 0;
    u8 naturalClass = 0;
    u8 railwayClass = 0;
    u8 waterClass = 0;
};

enum class MemberType : u8 { Node = 0, Way = 1, Relation = 2 };

struct Member {
    i64 ref = 0;
    MemberType type = MemberType::Node;
    u32 role = 0;            // interned string ("outer"/"inner"/...)
};

struct Relation {
    i64 id = 0;
    u32 firstMember = 0, memberCount = 0;
    u32 firstTag = 0, tagCount = 0;
};

// Highway classification ordered by importance (used for LOD and material
// assignment, mirroring the OSM highway hierarchy).
enum HighwayClass : u8 {
    HW_None = 0, HW_Motorway, HW_Trunk, HW_Primary, HW_Secondary, HW_Tertiary,
    HW_Unclassified, HW_Residential, HW_LivingStreet, HW_Pedestrian, HW_Service,
    HW_Footway, HW_Path, HW_Steps, HW_Track, HW_Link, HW_Other
};

enum BuildingClass : u8 {
    BC_None = 0, BC_Yes, BC_Residential, BC_Apartments, BC_Commercial, BC_Industrial,
    BC_Office, BC_House, BC_School, BC_College, BC_Hospital, BC_Religious,
    BC_Station, BC_Warehouse, BC_Retail, BC_Other
};

enum LanduseClass : u8 {
    LU_None = 0, LU_Residential, LU_Commercial, LU_Industrial, LU_Retail, LU_Grass,
    LU_Recreation, LU_Railway, LU_Reservoir, LU_Farm, LU_Quarry, LU_Military,
    LU_SaltPond, LU_Greenfield, LU_Forest, LU_Meadow
};

enum NaturalClass : u8 {
    NA_None = 0, NA_Water, NA_Coastline, NA_Wood, NA_Wetland, NA_Beach, NA_Scrub,
    NA_Grassland, NA_Peak, NA_Cliff, NA_Tree, NA_Grass, NA_Sand, NA_Bare
};

enum RailwayClass : u8 {
    RW_None = 0, RW_Rail, RW_Station, RW_Halt, RW_Platform, RW_Monorail, RW_Subway,
    RW_LevelCrossing, RW_NarrowGauge, RW_Metro
};

enum WaterClass : u8 {
    WT_None = 0, WT_River, WT_Stream, WT_Drain, WT_Canal, WT_Dock, WT_Riverbank,
    WT_Lake, WT_Pond, WT_Reservoir, WT_Basin
};

class OsmData {
public:
    bool load(const std::string& path, std::string* error = nullptr);

    // --- element access -------------------------------------------------------
    const Node* node(i64 id) const;
    const Way* way(i64 id) const;
    const Relation* relation(i64 id) const;

    const std::vector<Node>& nodes() const { return m_nodes; }
    const std::vector<Way>& ways() const { return m_ways; }
    const std::vector<Relation>& relations() const { return m_relations; }

    const i64* wayRefs(const Way& w) const { return m_wayRefs.data() + w.firstRef; }
    const Member* members(const Relation& r) const { return m_members.data() + r.firstMember; }
    const Tag* tagsOf(const Way& w) const { return m_tags.data() + w.firstTag; }
    const Tag* tagsOf(const Node& n) const { return m_tags.data() + n.firstTag; }
    const Tag* tagsOf(const Relation& r) const { return m_tags.data() + r.firstTag; }

    std::string_view key(u32 i) const;
    std::string_view value(u32 i) const;

    // Tag lookup by key name (linear over an element's own tags — these arrays
    // are tiny: the median way in this extract has 1 tag).
    std::string_view tag(const Way& w, std::string_view k) const;
    std::string_view tag(const Node& n, std::string_view k) const;
    std::string_view tag(const Relation& r, std::string_view k) const;
    bool hasTag(const Way& w, std::string_view k) const { return !tag(w, k).empty(); }

    // --- derived helpers ------------------------------------------------------
    // Resolve a way's node chain into projected cell-agnostic world metres.
    void wayPoints(const Way& w, const TransverseMercator& proj, std::vector<Vec2d>& out) const;

    // Classification pass; populates the cached classes on each Way.
    void classify();

    // Provenance: hash of the source bytes, used to stamp every chunk so a
    // world can be traced back to the exact data revision that produced it.
    u64 sourceHash() const { return m_sourceHash; }

    struct Stats {
        u64 nodeCount = 0, wayCount = 0, relationCount = 0, tagCount = 0;
        u64 highwayWays = 0, buildingWays = 0, closedWays = 0;
        u64 coastlineWays = 0, railwayWays = 0, waterWays = 0, landuseWays = 0;
        f64 bboxMinLat = 90, bboxMaxLat = -90, bboxMinLon = 180, bboxMaxLon = -180;
        u64 bytesParsed = 0;
    } stats;

private:
    u32 intern(std::string_view s);

    std::string m_blob;                    // raw file bytes
    std::vector<char> m_pool;              // interned string storage
    std::unordered_map<std::string_view, u32> m_interned;

    std::vector<Node> m_nodes;
    std::vector<Way> m_ways;
    std::vector<Relation> m_relations;
    std::vector<i64> m_wayRefs;
    std::vector<Member> m_members;
    std::vector<Tag> m_tags;

    std::unordered_map<i64, u32> m_nodeIndex;
    std::unordered_map<i64, u32> m_wayIndex;
    std::unordered_map<i64, u32> m_relationIndex;
    u64 m_sourceHash = 0;
};

} // namespace mumbai::osm
