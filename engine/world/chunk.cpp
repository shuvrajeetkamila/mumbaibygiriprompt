// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/chunk.cpp
// =============================================================================
#include "engine/world/chunk.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace mumbai {

const char* sectionTypeName(SectionType t) {
    switch (t) {
        case SectionType::ChunkInfo:         return "ChunkInfo";
        case SectionType::TerrainHeight:     return "TerrainHeight";
        case SectionType::TerrainMesh:       return "TerrainMesh";
        case SectionType::TerrainCollision:  return "TerrainCollision";
        case SectionType::TerrainAO:         return "TerrainAO";
        case SectionType::MaterialRegions:   return "MaterialRegions";
        case SectionType::RoadMesh:          return "RoadMesh";
        case SectionType::RoadNavGraph:      return "RoadNavGraph";
        case SectionType::BuildingMesh:      return "BuildingMesh";
        case SectionType::BuildingCollision: return "BuildingCollision";
        case SectionType::BuildingImpostor:  return "BuildingImpostor";
        case SectionType::WaterMesh:         return "WaterMesh";
        case SectionType::RailMesh:          return "RailMesh";
        case SectionType::LandusePolys:      return "LandusePolys";
        case SectionType::InstanceTable:     return "InstanceTable";
        case SectionType::MaterialTable:     return "MaterialTable";
        case SectionType::HeroAssets:        return "HeroAssets";
        case SectionType::QAReport:          return "QAReport";
        default:                             return "Invalid";
    }
}

u64 hashBytes(const void* data, size_t bytes) {
    const u8* p = static_cast<const u8*>(data);
    u64 h = 1469598103934665603ull;
    for (size_t i = 0; i < bytes; ++i) { h ^= p[i]; h *= 1099511628211ull; }
    return h;
}

// -----------------------------------------------------------------------------
//  ChunkWriter
// -----------------------------------------------------------------------------
ChunkWriter::ChunkWriter(i64 cellX, i64 cellZ, u64 sourceHash)
    : m_cellX(cellX), m_cellZ(cellZ), m_sourceHash(sourceHash) {}

void ChunkWriter::addSection(SectionType type, const void* ptr, u64 bytes, u32 elementCount,
                             u32 stride, u16 format, u16 flags) {
    if (bytes == 0 || ptr == nullptr) return;
    Pending p;
    p.type = type;
    p.count = elementCount;
    p.stride = stride;
    p.format = format;
    p.flags = flags;
    p.bytes.resize(static_cast<size_t>(bytes));
    std::memcpy(p.bytes.data(), ptr, static_cast<size_t>(bytes));
    m_pending.push_back(std::move(p));
}

u64 ChunkWriter::payloadBytes() const {
    u64 total = 0;
    for (const auto& p : m_pending) total += p.bytes.size() + 16;   // + alignment slack
    return total;
}

u64 ChunkWriter::write(const std::string& path) {
    // Compute layout: header, section table, then 16-byte aligned payloads.
    const u64 tableBytes = static_cast<u64>(m_pending.size()) * SECTION_ENTRY_SIZE;
    u64 cursor = CHUNK_HEADER_SIZE + tableBytes;
    cursor = (cursor + 15) & ~15ull;

    std::vector<SectionEntry> entries;
    entries.reserve(m_pending.size());
    u64 totalBytes = cursor;
    for (const auto& p : m_pending) {
        SectionEntry e{};
        e.type = static_cast<u32>(p.type);
        e.format = p.format;
        e.flags = p.flags;
        e.offset = cursor;
        e.size = p.bytes.size();
        e.elementCount = p.count;
        e.stride = p.stride;
        e.hash = hashBytes(p.bytes.data(), p.bytes.size());
        entries.push_back(e);
        cursor = (cursor + p.bytes.size() + 15) & ~15ull;   // keep payloads aligned
        totalBytes = cursor;
    }

    std::vector<u8> buffer(static_cast<size_t>(totalBytes), 0);

    ChunkHeader hdr{};
    std::memcpy(hdr.magic, CHUNK_MAGIC, 8);
    hdr.version = CHUNK_VERSION;
    hdr.headerSize = CHUNK_HEADER_SIZE;
    hdr.cellX = m_cellX;
    hdr.cellZ = m_cellZ;
    hdr.cellSize = MACRO_CELL_SIZE;
    hdr.minHeight = m_minHeight;
    hdr.maxHeight = m_maxHeight;
    hdr.sectionCount = static_cast<u32>(entries.size());
    hdr.flags = m_flags;
    hdr.fileSize = totalBytes;
    hdr.sourceHash = m_sourceHash;
    hdr.buildSchema = 1;

    std::memcpy(buffer.data(), &hdr, sizeof(hdr));
    std::memcpy(buffer.data() + CHUNK_HEADER_SIZE, entries.data(),
                entries.size() * sizeof(SectionEntry));

    for (size_t i = 0; i < m_pending.size(); ++i) {
        std::memcpy(buffer.data() + entries[i].offset, m_pending[i].bytes.data(),
                    m_pending[i].bytes.size());
    }

    // Content hash covers everything except the hash field itself, so a chunk
    // can be verified without knowing the build provenance.
    ChunkHeader* h = reinterpret_cast<ChunkHeader*>(buffer.data());
    h->contentHash = 0;
    h->contentHash = hashBytes(buffer.data(), buffer.size());

    // Atomic publish: write to temp, fsync, rename.  A partially written chunk
    // must never be visible to the runtime.
    const std::string tmp = path + ".tmp";
    ensureDir(std::filesystem::path(path).parent_path().string());
    std::FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) { MLOG_ERR("chunk write failed: %s", tmp.c_str()); return 0; }
    const size_t wrote = std::fwrite(buffer.data(), 1, buffer.size(), f);
    std::fflush(f);
    ::fsync(::fileno(f));
    std::fclose(f);
    if (wrote != buffer.size()) { std::remove(tmp.c_str()); return 0; }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        MLOG_ERR("chunk rename failed: %s", path.c_str());
        std::remove(tmp.c_str());
        return 0;
    }
    return buffer.size();
}

// -----------------------------------------------------------------------------
//  ChunkReader
// -----------------------------------------------------------------------------
ChunkReader::~ChunkReader() { close(); }

void ChunkReader::close() {
    if (m_data) {
        ::munmap(m_data, static_cast<size_t>(m_fileBytes));
        m_data = nullptr;
    }
    m_fileBytes = 0;
    m_sections.clear();
    m_header = ChunkHeader{};
}

bool ChunkReader::open(const std::string& path, std::string* error) {
    auto fail = [&](const char* msg) {
        if (error) *error = std::string(msg) + " [" + path + "]";
        close();
        return false;
    };

    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return fail("cannot open chunk");

    struct stat st{};
    if (::fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(CHUNK_HEADER_SIZE)) {
        ::close(fd);
        return fail("chunk too small");
    }
    m_fileBytes = static_cast<u64>(st.st_size);

    void* p = ::mmap(nullptr, static_cast<size_t>(m_fileBytes), PROT_READ, MAP_PRIVATE, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) { m_fileBytes = 0; return fail("mmap failed"); }
    // Advise sequential access once; the streaming system re-advises per frame.
    ::madvise(p, static_cast<size_t>(m_fileBytes), MADV_WILLNEED);
    m_data = static_cast<u8*>(p);

    std::memcpy(&m_header, m_data, sizeof(ChunkHeader));
    if (std::memcmp(m_header.magic, CHUNK_MAGIC, 8) != 0) return fail("bad magic");
    if (m_header.version != CHUNK_VERSION) return fail("version mismatch");
    if (m_header.headerSize != CHUNK_HEADER_SIZE) return fail("header size mismatch");
    if (m_header.fileSize != m_fileBytes) return fail("file size mismatch (truncated chunk?)");

    const u64 tableBytes = static_cast<u64>(m_header.sectionCount) * SECTION_ENTRY_SIZE;
    if (CHUNK_HEADER_SIZE + tableBytes > m_fileBytes) return fail("section table overruns file");

    m_sections.resize(m_header.sectionCount);
    std::memcpy(m_sections.data(), m_data + CHUNK_HEADER_SIZE,
                static_cast<size_t>(tableBytes));

    // Validate every section lies inside the file and is aligned.
    for (const auto& e : m_sections) {
        if (e.offset + e.size > m_fileBytes) return fail("section out of bounds");
        if (e.offset % 16 != 0) return fail("section misaligned");
        // The section table must never share bytes with a payload: that
        // overlap is how a mis-sized table silently corrupted chunks.
        if (e.offset < CHUNK_HEADER_SIZE + tableBytes) return fail("section overlaps table");
    }
    return true;
}

const SectionEntry* ChunkReader::find(SectionType t) const {
    for (const auto& e : m_sections)
        if (e.type == static_cast<u32>(t)) return &e;
    return nullptr;
}

bool ChunkReader::verifyHashes(std::string* error) const {
    if (!m_data) { if (error) *error = "not open"; return false; }
    for (const auto& e : m_sections) {
        const u64 h = hashBytes(m_data + e.offset, static_cast<size_t>(e.size));
        if (h != e.hash) {
            if (error) {
                *error = std::string("section hash mismatch: ") +
                         sectionTypeName(static_cast<SectionType>(e.type));
            }
            return false;
        }
    }
    return true;
}

// -----------------------------------------------------------------------------
//  Mesh group access.  Every accessor re-validates offsets against the section
//  bounds: a corrupt or hostile chunk must never be able to steer the runtime
//  into reading outside its own mapping.  That is the concrete meaning of the
//  "zero rendering artifacts" requirement — the GPU can only ever be handed
//  bytes that were proven in-bounds.
// -----------------------------------------------------------------------------
Aabb ChunkReader::MeshView::bounds() const {
    Aabb b;
    for (u32 i = 0; i < vertexCount; ++i)
        b.expand(Vec3{vertices[i].px, vertices[i].py, vertices[i].pz});
    return b;
}

ChunkReader::NavGraphView ChunkReader::navGraph() const {
    NavGraphView v;
    const SectionEntry* e = find(SectionType::RoadNavGraph);
    if (!e || e->size < sizeof(NavGraphHeader)) return v;
    const u8* base = m_data + e->offset;
    NavGraphHeader h{};
    std::memcpy(&h, base, sizeof(h));
    const u64 nodesBytes   = static_cast<u64>(h.nodeCount) * sizeof(NavNode);
    const u64 edgesBytes   = static_cast<u64>(h.edgeCount) * sizeof(NavEdge);
    const u64 signalsBytes = static_cast<u64>(h.signalCount) * sizeof(NavSignalPhase);
    // Bounds first: a corrupt count must not be able to steer the runtime.
    if (nodesBytes + edgesBytes + signalsBytes > e->size - sizeof(NavGraphHeader)) return v;
    v.header = reinterpret_cast<const NavGraphHeader*>(base);
    v.nodes = reinterpret_cast<const NavNode*>(base + sizeof(NavGraphHeader));
    v.edges = reinterpret_cast<const NavEdge*>(reinterpret_cast<const u8*>(v.nodes) + nodesBytes);
    v.signals = reinterpret_cast<const NavSignalPhase*>(reinterpret_cast<const u8*>(v.edges) + edgesBytes);
    return v;
}

u32 ChunkReader::meshCount(SectionType t) const {
    const SectionEntry* e = find(t);
    if (!e || e->size < sizeof(MeshGroupHeader)) return 0;
    const auto* hdr = reinterpret_cast<const MeshGroupHeader*>(m_data + e->offset);
    const u64 needed = sizeof(MeshGroupHeader) +
                       static_cast<u64>(hdr->meshCount) * sizeof(MeshEntry);
    if (needed > e->size) return 0;
    return hdr->meshCount;
}

ChunkReader::MeshView ChunkReader::mesh(SectionType t, u32 index) const {
    MeshView v;
    const SectionEntry* e = find(t);
    if (!e || e->size < sizeof(MeshGroupHeader)) return v;
    const u8* base = m_data + e->offset;
    const auto* hdr = reinterpret_cast<const MeshGroupHeader*>(base);
    if (index >= hdr->meshCount) return v;
    const auto* entries = reinterpret_cast<const MeshEntry*>(base + sizeof(MeshGroupHeader));
    const MeshEntry& me = entries[index];

    // Every block must lie fully inside the section.
    auto fits = [&](u64 off, u64 bytes) { return off <= e->size && bytes <= e->size - off; };
    if (!fits(me.verticesOffset, static_cast<u64>(me.vertexCount) * sizeof(MeshVertex))) return v;
    if (!fits(me.indicesOffset, static_cast<u64>(me.indexCount) * sizeof(u32))) return v;
    if (!fits(me.submeshesOffset, static_cast<u64>(me.submeshCount) * sizeof(SubMesh))) return v;

    v.vertices = reinterpret_cast<const MeshVertex*>(base + me.verticesOffset);
    v.indices = reinterpret_cast<const u32*>(base + me.indicesOffset);
    v.submeshes = reinterpret_cast<const SubMesh*>(base + me.submeshesOffset);
    v.vertexCount = me.vertexCount;
    v.indexCount = me.indexCount;
    v.submeshCount = me.submeshCount;
    v.lodLevel = me.lodLevel;
    return v;
}

ChunkReader::MeshView ChunkReader::meshByLod(SectionType t, u32 lod) const {
    const u32 count = meshCount(t);
    if (count == 0) return {};
    const SectionEntry* e = find(t);
    const auto* entries = reinterpret_cast<const MeshEntry*>(m_data + e->offset + sizeof(MeshGroupHeader));
    // Exact LOD match preferred, then the nearest higher-detail level.
    u32 best = 0;
    u32 bestScore = 0xFFFFFFFFu;
    for (u32 i = 0; i < count; ++i) {
        const u32 level = entries[i].lodLevel;
        const u32 score = (level >= lod) ? (level - lod) : (lod - level) + 1000u;
        if (score < bestScore) { bestScore = score; best = i; }
    }
    return mesh(t, best);
}

} // namespace mumbai
