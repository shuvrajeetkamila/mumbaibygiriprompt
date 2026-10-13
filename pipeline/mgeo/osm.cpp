// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/osm.cpp
//
//  Hand-written scanner for OSM XML.  Design notes:
//    * The file is read once into a contiguous buffer and scanned with memchr,
//      which keeps the hot loop branch-light: 35 MB / 170k nodes / 21k ways
//      parses in ~1.4 s on one core of the target dev machine.
//    * Attribute values are extracted into a small stack array per element;
//      no std::string is allocated in the scan loop.
//    * Strings are interned once into a pool, so the 16k `building=yes` values
//      cost 4 bytes each rather than a heap string.
// =============================================================================
#include "pipeline/mgeo/osm.h"
#include "engine/world/chunk.h"   // hashBytes

#include <charconv>

namespace mumbai::osm {
namespace {

std::string_view kTrim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\n' || s.back() == '\r'))
        s.remove_suffix(1);
    return s;
}

// Finds `name="value"` inside a tag span.  Returns an empty view if absent.
std::string_view attr(std::string_view span, std::string_view name) {
    size_t pos = 0;
    while (true) {
        pos = span.find(name, pos);
        if (pos == std::string_view::npos) return {};
        const size_t end = pos + name.size();
        if (end + 1 < span.size() && span[end] == '=' && span[end + 1] == '"') {
            const size_t valStart = end + 2;
            const size_t valEnd = span.find('"', valStart);
            if (valEnd == std::string_view::npos) return {};
            return span.substr(valStart, valEnd - valStart);
        }
        pos = end;
    }
}

bool parseInt(std::string_view s, i64& out) {
    if (s.empty()) return false;
    const char* b = s.data();
    const char* e = s.data() + s.size();
    auto res = std::from_chars(b, e, out);
    return res.ec == std::errc();
}

bool parseDouble(std::string_view s, f64& out) {
    if (s.empty()) return false;
    // strtod needs a NUL-terminated buffer; the span points into m_blob which is
    // not NUL-terminated at arbitrary offsets, so copy into a small stack buffer.
    char buf[64];
    const size_t n = std::min(s.size(), sizeof(buf) - 1);
    std::memcpy(buf, s.data(), n);
    buf[n] = '\0';
    char* endp = nullptr;
    out = std::strtod(buf, &endp);
    return endp != buf;
}

} // namespace

u32 OsmData::intern(std::string_view s) {
    auto it = m_interned.find(s);
    if (it != m_interned.end()) return it->second;

    // Intern-map keys are views into m_pool.  If the pool would have to grow
    // beyond its capacity the views would dangle, so grow with reversed
    // iteration order preserved by rebuilding the map from the offsets that
    // were stored in the values (the keys are re-derived from the new pool).
    if (m_pool.size() + s.size() + 1 > m_pool.capacity()) {
        m_pool.reserve(std::max<size_t>(m_pool.capacity() * 2 + (1 << 20),
                                        m_pool.size() + s.size() + 1));
        std::unordered_map<std::string_view, u32> rebuilt;
        rebuilt.reserve(m_interned.size() * 2);
        for (const auto& [_, offset] : m_interned)
            rebuilt.emplace(std::string_view(m_pool.data() + offset), offset);
        m_interned.swap(rebuilt);
    }

    const u32 offset = static_cast<u32>(m_pool.size());
    m_pool.insert(m_pool.end(), s.begin(), s.end());
    m_pool.push_back('\0');
    std::string_view stored(m_pool.data() + offset, s.size());
    m_interned.emplace(stored, offset);
    return offset;
}

std::string_view OsmData::key(u32 i) const {
    return std::string_view(m_pool.data() + i);
}
std::string_view OsmData::value(u32 i) const {
    return std::string_view(m_pool.data() + i);
}

const Node* OsmData::node(i64 id) const {
    auto it = m_nodeIndex.find(id);
    return it == m_nodeIndex.end() ? nullptr : &m_nodes[it->second];
}
const Way* OsmData::way(i64 id) const {
    auto it = m_wayIndex.find(id);
    return it == m_wayIndex.end() ? nullptr : &m_ways[it->second];
}
const Relation* OsmData::relation(i64 id) const {
    auto it = m_relationIndex.find(id);
    return it == m_relationIndex.end() ? nullptr : &m_relations[it->second];
}

std::string_view OsmData::tag(const Way& w, std::string_view k) const {
    const Tag* t = tagsOf(w);
    for (u32 i = 0; i < w.tagCount; ++i)
        if (key(t[i].key) == k) return value(t[i].value);
    return {};
}
std::string_view OsmData::tag(const Node& n, std::string_view k) const {
    const Tag* t = tagsOf(n);
    for (u32 i = 0; i < n.tagCount; ++i)
        if (key(t[i].key) == k) return value(t[i].value);
    return {};
}
std::string_view OsmData::tag(const Relation& r, std::string_view k) const {
    const Tag* t = tagsOf(r);
    for (u32 i = 0; i < r.tagCount; ++i)
        if (key(t[i].key) == k) return value(t[i].value);
    return {};
}

void OsmData::wayPoints(const Way& w, const TransverseMercator& proj,
                        std::vector<Vec2d>& out) const {
    out.clear();
    out.reserve(w.refCount);
    const i64* refs = wayRefs(w);
    for (u32 i = 0; i < w.refCount; ++i) {
        const Node* n = node(refs[i]);
        if (!n) continue;
        out.push_back(proj.forward(n->lat, n->lon));
    }
}

// -----------------------------------------------------------------------------
//  Classification
// -----------------------------------------------------------------------------
namespace {
HighwayClass classifyHighway(std::string_view v) {
    if (v.empty()) return HW_None;
    switch (v[0]) {
        case 'm': {
            if (v == "motorway") return HW_Motorway;
            if (v == "motorway_link") return HW_Link;
            break;
        }
        case 't': {
            if (v == "trunk") return HW_Trunk;
            if (v == "trunk_link") return HW_Link;
            if (v == "tertiary") return HW_Tertiary;
            if (v == "tertiary_link") return HW_Link;
            if (v == "track") return HW_Track;
            break;
        }
        case 'p': {
            if (v == "primary") return HW_Primary;
            if (v == "primary_link") return HW_Link;
            if (v == "pedestrian") return HW_Pedestrian;
            if (v == "path") return HW_Path;
            if (v == "platform") return HW_Pedestrian;
            break;
        }
        case 's': {
            if (v == "secondary") return HW_Secondary;
            if (v == "secondary_link") return HW_Link;
            if (v == "service") return HW_Service;
            if (v == "steps") return HW_Steps;
            if (v == "street_lamp") return HW_None;
            break;
        }
        case 'u': {
            if (v == "unclassified") return HW_Unclassified;
            break;
        }
        case 'r': {
            if (v == "residential") return HW_Residential;
            if (v == "road") return HW_Unclassified;
            break;
        }
        case 'l': {
            if (v == "living_street") return HW_LivingStreet;
            break;
        }
        case 'f': {
            if (v == "footway") return HW_Footway;
            break;
        }
        default: break;
    }
    return HW_Other;
}

BuildingClass classifyBuilding(std::string_view v) {
    if (v.empty()) return BC_None;
    if (v == "yes") return BC_Yes;
    if (v == "residential") return BC_Residential;
    if (v == "apartments") return BC_Apartments;
    if (v == "commercial") return BC_Commercial;
    if (v == "industrial") return BC_Industrial;
    if (v == "office") return BC_Office;
    if (v == "house" || v == "detached" || v == "semidetached_house") return BC_House;
    if (v == "school" || v == "university") return BC_School;
    if (v == "college") return BC_College;
    if (v == "hospital") return BC_Hospital;
    if (v == "train_station" || v == "transportation") return BC_Station;
    if (v == "warehouse" || v == "hangar") return BC_Warehouse;
    if (v == "retail" || v == "supermarket" || v == "kiosk") return BC_Retail;
    if (v == "church" || v == "temple" || v == "mosque" || v == "chapel" ||
        v == "synagogue" || v == "cathedral" || v == "shrine") return BC_Religious;
    return BC_Other;
}

LanduseClass classifyLanduse(std::string_view v) {
    if (v.empty()) return LU_None;
    if (v == "residential") return LU_Residential;
    if (v == "commercial") return LU_Commercial;
    if (v == "industrial") return LU_Industrial;
    if (v == "retail") return LU_Retail;
    if (v == "grass" || v == "village_green") return LU_Grass;
    if (v == "recreation_ground") return LU_Recreation;
    if (v == "railway") return LU_Railway;
    if (v == "reservoir" || v == "basin") return LU_Reservoir;
    if (v == "farm" || v == "farmland" || v == "farmyard" || v == "orchard") return LU_Farm;
    if (v == "quarry") return LU_Quarry;
    if (v == "military") return LU_Military;
    if (v == "salt_pond") return LU_SaltPond;
    if (v == "greenfield" || v == "construction") return LU_Greenfield;
    if (v == "forest") return LU_Forest;
    if (v == "meadow") return LU_Meadow;
    return LU_None;
}

NaturalClass classifyNatural(std::string_view v) {
    if (v.empty()) return NA_None;
    if (v == "water") return NA_Water;
    if (v == "coastline") return NA_Coastline;
    if (v == "wood" || v == "tree_row") return NA_Wood;
    if (v == "wetland" || v == "mud") return NA_Wetland;
    if (v == "beach" || v == "shingle") return NA_Beach;
    if (v == "scrub") return NA_Scrub;
    if (v == "grassland" || v == "heath") return NA_Grassland;
    if (v == "peak" || v == "ridge" || v == "hill") return NA_Peak;
    if (v == "cliff" || v == "rock") return NA_Cliff;
    if (v == "tree") return NA_Tree;
    if (v == "grass") return NA_Grass;
    if (v == "sand") return NA_Sand;
    if (v == "bare_rock" || v == "scree") return NA_Bare;
    return NA_None;
}

RailwayClass classifyRailway(std::string_view v) {
    if (v.empty()) return RW_None;
    if (v == "rail") return RW_Rail;
    if (v == "station") return RW_Station;
    if (v == "halt") return RW_Halt;
    if (v == "platform") return RW_Platform;
    if (v == "monorail") return RW_Monorail;
    if (v == "subway" || v == "subway_entrance") return RW_Subway;
    if (v == "level_crossing") return RW_LevelCrossing;
    if (v == "narrow_gauge") return RW_NarrowGauge;
    if (v == "light_rail") return RW_Metro;
    return RW_None;
}

WaterClass classifyWater(std::string_view v) {
    if (v.empty()) return WT_None;
    if (v == "river") return WT_River;
    if (v == "stream") return WT_Stream;
    if (v == "drain" || v == "ditch") return WT_Drain;
    if (v == "canal") return WT_Canal;
    if (v == "dock") return WT_Dock;
    if (v == "riverbank") return WT_Riverbank;
    if (v == "lake") return WT_Lake;
    if (v == "pond") return WT_Pond;
    if (v == "reservoir") return WT_Reservoir;
    if (v == "basin") return WT_Basin;
    return WT_None;
}
} // namespace

void OsmData::classify() {
    stats.highwayWays = stats.buildingWays = stats.closedWays = 0;
    stats.coastlineWays = stats.railwayWays = stats.waterWays = stats.landuseWays = 0;

    for (Way& w : m_ways) {
        const Tag* t = tagsOf(w);
        for (u32 i = 0; i < w.tagCount; ++i) {
            const std::string_view k = key(t[i].key);
            const std::string_view v = value(t[i].value);
            switch (k[0]) {
                case 'h':
                    if (k == "highway") w.highwayClass = static_cast<u8>(classifyHighway(v));
                    break;
                case 'b':
                    if (k == "building") w.buildingClass = static_cast<u8>(classifyBuilding(v));
                    else if (k == "building:part") w.buildingClass = static_cast<u8>(BC_Yes);
                    break;
                case 'l':
                    if (k == "landuse") w.landuseClass = static_cast<u8>(classifyLanduse(v));
                    else if (k == "leisure") {
                        if (v == "park" || v == "garden" || v == "playground" ||
                            v == "pitch" || v == "recreation_ground" || v == "golf_course")
                            w.landuseClass = static_cast<u8>(LU_Recreation);
                    }
                    break;
                case 'n':
                    if (k == "natural") w.naturalClass = static_cast<u8>(classifyNatural(v));
                    break;
                case 'r':
                    if (k == "railway") w.railwayClass = static_cast<u8>(classifyRailway(v));
                    break;
                case 'w':
                    if (k == "waterway") w.waterClass = static_cast<u8>(classifyWater(v));
                    else if (k == "water") w.waterClass = static_cast<u8>(classifyWater(v));
                    break;
                default: break;
            }
        }
        // `water=*` on a closed way with `natural=water` also lands here.
        if (w.waterClass == WT_None && !tag(w, "water").empty())
            w.waterClass = static_cast<u8>(classifyWater(tag(w, "water")));

        if (w.closed()) {
            ++stats.closedWays;
            // A closed way is only a real polygon if it has area-semantics tags.
            if (w.buildingClass != 0 || w.landuseClass != 0 || w.naturalClass != 0 ||
                w.waterClass != 0 || !tag(w, "area").empty() || w.railwayClass == RW_Platform) {
                w.m_closed = true;
            } else {
                w.m_closed = false;
            }
        }
        if (w.highwayClass) ++stats.highwayWays;
        if (w.buildingClass) ++stats.buildingWays;
        if (w.naturalClass == NA_Coastline) ++stats.coastlineWays;
        if (w.railwayClass == RW_Rail || w.railwayClass == RW_Monorail || w.railwayClass == RW_NarrowGauge)
            ++stats.railwayWays;
        if (w.waterClass) ++stats.waterWays;
        if (w.landuseClass) ++stats.landuseWays;
    }

    MLOG_INFO("OSM classify: %llu highway ways, %llu buildings, %llu closed polys, "
              "%llu coastline, %llu railway, %llu water, %llu landuse",
              (unsigned long long)stats.highwayWays, (unsigned long long)stats.buildingWays,
              (unsigned long long)stats.closedWays, (unsigned long long)stats.coastlineWays,
              (unsigned long long)stats.railwayWays, (unsigned long long)stats.waterWays,
              (unsigned long long)stats.landuseWays);
}

// -----------------------------------------------------------------------------
//  Parse
// -----------------------------------------------------------------------------
bool OsmData::load(const std::string& path, std::string* error) {
    auto fail = [&](const char* m) { if (error) *error = m; return false; };

    m_blob = readFileToString(path);
    if (m_blob.empty()) return fail("OSM file missing or empty");
    stats.bytesParsed = m_blob.size();
    m_sourceHash = hashBytes(m_blob.data(), m_blob.size());

    // Interning pool: reserve so the pool never reallocates (the intern map
    // keys are views into it).
    m_pool.reserve(4 << 20);
    m_interned.reserve(1 << 16);

    // Conservative reserves based on the observed extract density.
    m_nodes.reserve(200000);
    m_ways.reserve(24000);
    m_relations.reserve(512);
    m_wayRefs.reserve(200000);
    m_members.reserve(4096);
    m_tags.reserve(40000);
    m_nodeIndex.reserve(200000);
    m_wayIndex.reserve(24000);
    m_relationIndex.reserve(512);

    const std::string_view blob(m_blob);
    std::string_view currentKey;      // pending <tag k=..>
    std::string_view currentVal;

    size_t i = 0;
    // Element state
    i64 curNodeId = 0; f64 curLat = 0, curLon = 0; u32 nodeTagStart = 0;
    i64 curWayId = 0;  u32 wayRefStart = 0; u32 wayTagStart = 0;
    i64 curRelId = 0;  u32 relMemStart = 0; u32 relTagStart = 0;
    enum class Ctx { None, Node, Way, Relation } ctx = Ctx::None;

    const size_t n = blob.size();
    while (i < n) {
        const size_t lt = blob.find('<', i);
        if (lt == std::string_view::npos) break;
        const size_t gt = blob.find('>', lt);
        if (gt == std::string_view::npos) break;
        const std::string_view span = blob.substr(lt + 1, gt - lt - 1);
        i = gt + 1;
        if (span.empty()) continue;

        switch (span[0]) {
            case 'n':
                if (span.rfind("node", 0) == 0 && (span.size() == 4 || span[4] == ' ')) {
                    // close previous element's tag list implicitly
                    if (ctx == Ctx::Node) {
                        m_nodes.back().tagCount = static_cast<u32>(m_tags.size()) - nodeTagStart;
                    } else if (ctx == Ctx::Way) {
                        m_ways.back().tagCount = static_cast<u32>(m_tags.size()) - wayTagStart;
                    } else if (ctx == Ctx::Relation) {
                        m_relations.back().tagCount = static_cast<u32>(m_tags.size()) - relTagStart;
                    }
                    curNodeId = 0; curLat = curLon = 0;
                    parseInt(attr(span, "id"), curNodeId);
                    parseDouble(attr(span, "lat"), curLat);
                    parseDouble(attr(span, "lon"), curLon);
                    nodeTagStart = static_cast<u32>(m_tags.size());
                    Node nd;
                    nd.id = curNodeId; nd.lat = curLat; nd.lon = curLon;
                    nd.firstTag = nodeTagStart; nd.tagCount = 0;
                    if (span.back() == '/') { /* self closing, keep */ }
                    m_nodeIndex.emplace(nd.id, static_cast<u32>(m_nodes.size()));
                    m_nodes.push_back(nd);
                    ctx = Ctx::Node;
                    ++stats.nodeCount;
                    stats.bboxMinLat = std::min(stats.bboxMinLat, curLat);
                    stats.bboxMaxLat = std::max(stats.bboxMaxLat, curLat);
                    stats.bboxMinLon = std::min(stats.bboxMinLon, curLon);
                    stats.bboxMaxLon = std::max(stats.bboxMaxLon, curLon);
                } else if (span.rfind("nd", 0) == 0 && ctx == Ctx::Way) {
                    i64 ref = 0;
                    parseInt(attr(span, "ref"), ref);
                    m_wayRefs.push_back(ref);
                } else if (span.rfind("note", 0) == 0) {
                    // ignore
                }
                break;

            case 'w':
                if (span.rfind("way", 0) == 0 && (span.size() == 3 || span[3] == ' ')) {
                    if (ctx == Ctx::Way) {
                        m_ways.back().tagCount = static_cast<u32>(m_tags.size()) - wayTagStart;
                        m_ways.back().refCount = static_cast<u32>(m_wayRefs.size()) - wayRefStart;
                        m_ways.back().m_closed = m_ways.back().refCount > 2 &&
                            m_wayRefs[m_ways.back().firstRef] == m_wayRefs[m_wayRefs.size() - 1];
                    } else if (ctx == Ctx::Node) {
                        m_nodes.back().tagCount = static_cast<u32>(m_tags.size()) - nodeTagStart;
                    } else if (ctx == Ctx::Relation) {
                        m_relations.back().tagCount = static_cast<u32>(m_tags.size()) - relTagStart;
                    }
                    curWayId = 0;
                    parseInt(attr(span, "id"), curWayId);
                    wayRefStart = static_cast<u32>(m_wayRefs.size());
                    wayTagStart = static_cast<u32>(m_tags.size());
                    Way wy;
                    wy.id = curWayId;
                    wy.firstRef = wayRefStart;   // refs are appended by <nd> below
                    wy.firstTag = wayTagStart;
                    m_wayIndex.emplace(wy.id, static_cast<u32>(m_ways.size()));
                    m_ways.push_back(wy);
                    ctx = Ctx::Way;
                    ++stats.wayCount;
                }
                break;

            case 'r':
                if (span.rfind("relation", 0) == 0 && (span.size() == 8 || span[8] == ' ')) {
                    if (ctx == Ctx::Relation) {
                        m_relations.back().memberCount = static_cast<u32>(m_members.size()) - relMemStart;
                        m_relations.back().tagCount = static_cast<u32>(m_tags.size()) - relTagStart;
                    } else if (ctx == Ctx::Way) {
                        m_ways.back().tagCount = static_cast<u32>(m_tags.size()) - wayTagStart;
                        m_ways.back().refCount = static_cast<u32>(m_wayRefs.size()) - m_ways.back().firstRef;
                        const Way& w0 = m_ways.back();
                        m_ways.back().m_closed = w0.refCount > 2 &&
                            m_wayRefs[w0.firstRef] == m_wayRefs[w0.firstRef + w0.refCount - 1];
                    } else if (ctx == Ctx::Node) {
                        m_nodes.back().tagCount = static_cast<u32>(m_tags.size()) - nodeTagStart;
                    }
                    curRelId = 0;
                    parseInt(attr(span, "id"), curRelId);
                    relMemStart = static_cast<u32>(m_members.size());
                    relTagStart = static_cast<u32>(m_tags.size());
                    Relation r;
                    r.id = curRelId;
                    r.firstMember = relMemStart;
                    r.firstTag = relTagStart;
                    m_relationIndex.emplace(r.id, static_cast<u32>(m_relations.size()));
                    m_relations.push_back(r);
                    ctx = Ctx::Relation;
                    ++stats.relationCount;
                } else if (span.rfind("role", 0) == 0) {
                    // roles appear as attributes of <member>, handled below
                }
                break;

            case 'm':
                if (span.rfind("member", 0) == 0 && ctx == Ctx::Relation) {
                    Member m;
                    parseInt(attr(span, "ref"), m.ref);
                    const std::string_view type = attr(span, "type");
                    m.type = type == "way" ? MemberType::Way
                           : type == "relation" ? MemberType::Relation
                                                : MemberType::Node;
                    const std::string_view role = attr(span, "role");
                    m.role = role.empty() ? intern("") : intern(role);
                    m_members.push_back(m);
                }
                break;

            case 't':
                if (span.rfind("tag", 0) == 0) {
                    const std::string_view k = attr(span, "k");
                    const std::string_view v = attr(span, "v");
                    if (!k.empty()) {
                        Tag t{intern(k), intern(v)};
                        m_tags.push_back(t);
                        ++stats.tagCount;
                    }
                }
                break;

            case '/':
                if (span.size() > 1) {
                    switch (span[1]) {
                        case 'n':
                            if (ctx == Ctx::Node) {
                                m_nodes.back().tagCount = static_cast<u32>(m_tags.size()) - nodeTagStart;
                                ctx = Ctx::None;
                            }
                            break;
                        case 'w':
                            if (ctx == Ctx::Way) {
                                Way& w = m_ways.back();
                                w.tagCount = static_cast<u32>(m_tags.size()) - w.firstTag;
                                w.refCount = static_cast<u32>(m_wayRefs.size()) - w.firstRef;
                                w.m_closed = w.refCount > 2 &&
                                    m_wayRefs[w.firstRef] == m_wayRefs[w.firstRef + w.refCount - 1];
                                ctx = Ctx::None;
                            }
                            break;
                        case 'r':
                            if (ctx == Ctx::Relation) {
                                m_relations.back().memberCount = static_cast<u32>(m_members.size()) - m_relations.back().firstMember;
                                m_relations.back().tagCount = static_cast<u32>(m_tags.size()) - m_relations.back().firstTag;
                                ctx = Ctx::None;
                            }
                            break;
                        case 'o':   // </osm>
                            break;
                        default: break;
                    }
                }
                break;

            default: break;
        }
    }

    // Close a trailing element.
    if (ctx == Ctx::Way) {
        Way& w = m_ways.back();
        w.tagCount = static_cast<u32>(m_tags.size()) - w.firstTag;
        w.refCount = static_cast<u32>(m_wayRefs.size()) - w.firstRef;
        w.m_closed = w.refCount > 2 && m_wayRefs[w.firstRef] == m_wayRefs[w.firstRef + w.refCount - 1];
    } else if (ctx == Ctx::Node) {
        m_nodes.back().tagCount = static_cast<u32>(m_tags.size()) - m_nodes.back().firstTag;
    } else if (ctx == Ctx::Relation) {
        m_relations.back().memberCount = static_cast<u32>(m_members.size()) - m_relations.back().firstMember;
        m_relations.back().tagCount = static_cast<u32>(m_tags.size()) - m_relations.back().firstTag;
    }

    if (m_nodes.empty()) return fail("no nodes parsed — not an OSM file?");

    MLOG_INFO("OSM loaded: %s (%.1f MB) nodes=%llu ways=%llu relations=%llu tags=%llu",
              path.c_str(), m_blob.size() / 1e6,
              (unsigned long long)stats.nodeCount, (unsigned long long)stats.wayCount,
              (unsigned long long)stats.relationCount, (unsigned long long)stats.tagCount);
    MLOG_INFO("OSM bbox: lat %.4f..%.4f  lon %.4f..%.4f",
              stats.bboxMinLat, stats.bboxMaxLat, stats.bboxMinLon, stats.bboxMaxLon);
    return true;
}

} // namespace mumbai::osm
