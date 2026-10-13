// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_chunk.cpp
//
//  The .mumbai_chunk contract: written by the offline pipeline, mapped by the
//  runtime.  A single wrong offset here corrupts the world silently, so the
//  layout is asserted rather than assumed.
// =============================================================================
#include "engine/world/chunk.h"
#include "tests/test_harness.h"

#include <cstdio>
#include <string>

using namespace mumbai;

namespace {
std::string tempPath(const char* name) { return std::string("/tmp/mumbai_test_") + name; }
}  // namespace

TEST(chunkStructsMatchTheDocumentedLayout) {
    CHECK_EQ(sizeof(ChunkHeader), 128u);
    CHECK_EQ(sizeof(SectionEntry), 40u);
    CHECK_EQ(sizeof(MeshVertex), 44u);
    CHECK_EQ(sizeof(MeshEntry), 40u);
    CHECK_EQ(sizeof(CollisionBox), 48u);
    CHECK_EQ(sizeof(NavNode), 24u);
    CHECK_EQ(sizeof(NavEdge), 56u);
    CHECK_EQ(sizeof(InstanceRecord), 52u);
    CHECK_EQ(sizeof(MaterialDef), 80u);
    CHECK_EQ(sizeof(CellInfo), 128u);
    CHECK_EQ(sizeof(BuildingProxy), 48u);
    CHECK_EQ(SECTION_ENTRY_SIZE, sizeof(SectionEntry));
}

TEST(chunkRoundTripsSectionsAndVerifiesHashes) {
    const std::string path = tempPath("roundtrip.mumbai_chunk");
    CellInfo info{};
    info.buildingCount = 7;
    std::vector<f32> heights(9, 1.5f);
    std::vector<MaterialDef> materials(2);
    std::snprintf(materials[0].name, sizeof(materials[0].name), "asphalt_smooth");
    materials[0].roughness = 0.82f;
    std::vector<BuildingProxy> proxies(3);
    for (u32 i = 0; i < proxies.size(); ++i) {
        proxies[i].cx = 10.0f * i;
        proxies[i].hx = 4.0f;
        proxies[i].hy = 6.0f;
        proxies[i].hz = 5.0f;
        proxies[i].yaw = 0.5f * i;
        proxies[i].materialId = 1;
    }

    {
        ChunkWriter w(-5, 26, 0x1234);
        w.addSection(SectionType::ChunkInfo, &info, sizeof(info), 1, sizeof(info));
        w.addSection(SectionType::TerrainHeight, heights.data(), heights.size() * sizeof(f32),
                     static_cast<u32>(heights.size()), sizeof(f32));
        w.addSection(SectionType::MaterialTable, materials.data(), materials.size() * sizeof(MaterialDef),
                     static_cast<u32>(materials.size()), sizeof(MaterialDef));
        w.addSection(SectionType::BuildingProxy, proxies.data(), proxies.size() * sizeof(BuildingProxy),
                     static_cast<u32>(proxies.size()), sizeof(BuildingProxy));
        CHECK(w.write(path) > 0);
    }

    ChunkReader r;
    std::string err;
    CHECK(r.open(path, &err));
    if (r.valid()) {
        CHECK_EQ(r.header().cellX, -5);
        CHECK_EQ(r.header().cellZ, 26);
        CHECK_EQ(r.header().sectionCount, 4u);
        CHECK(r.verifyHashes(&err));

        u32 n = 0;
        const f32* h = r.array<f32>(SectionType::TerrainHeight, &n);
        CHECK(h != nullptr);
        CHECK_EQ(n, 9u);
        if (h) CHECK_NEAR(h[3], 1.5, 1e-6);

        u32 mn = 0;
        const MaterialDef* m = r.array<MaterialDef>(SectionType::MaterialTable, &mn);
        CHECK(m != nullptr);
        CHECK_EQ(mn, 2u);
        if (m) CHECK_EQ(std::string(m[0].name), std::string("asphalt_smooth"));

        u32 pn = 0;
        const BuildingProxy* p = r.array<BuildingProxy>(SectionType::BuildingProxy, &pn);
        CHECK(p != nullptr);
        CHECK_EQ(pn, 3u);
        if (p) {
            CHECK_NEAR(p[2].cx, 20.0, 1e-6);
            CHECK_NEAR(p[2].yaw, 1.0, 1e-6);
        }

        // A stride mismatch must be refused rather than misread.  (The probing
        // type must differ in size from the stored stride: BuildingProxy and
        // CollisionBox are both 48 bytes, so a same-size reinterpretation is
        // legal and would be the caller's problem, not the reader's.)
        u32 wrong = 0;
        CHECK(r.array<NavNode>(SectionType::BuildingProxy, &wrong) == nullptr);
        CHECK_EQ(wrong, 0u);
    }
    std::remove(path.c_str());
}

TEST(chunkDetectsCorruption) {
    const std::string path = tempPath("corrupt.mumbai_chunk");
    std::vector<u8> payload(1024);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<u8>(i & 0xFF);
    {
        ChunkWriter w(0, 0);
        w.addSection(SectionType::TerrainMesh, payload.data(), payload.size(),
                     static_cast<u32>(payload.size()), 1);
        CHECK(w.write(path) > 0);
    }
    // Flip one byte in the middle of the payload, located through the reader's
    // own section table so the test cannot silently miss the payload.
    u64 flipAt = 0;
    {
        ChunkReader probe;
        std::string e;
        if (probe.open(path, &e)) {
            if (const SectionEntry* sec = probe.find(SectionType::TerrainMesh))
                flipAt = sec->offset + sec->size / 2;
        }
    }
    CHECK(flipAt != 0);
    {
        std::FILE* f = std::fopen(path.c_str(), "r+b");
        CHECK(f != nullptr);
        if (f) {
            std::fseek(f, static_cast<long>(flipAt), SEEK_SET);
            const u8 bad = 0x5A;
            CHECK_EQ(std::fwrite(&bad, 1, 1, f), 1u);
            std::fclose(f);
        }
    }
    ChunkReader r;
    std::string err;
    if (r.open(path, &err)) {
        // The per-section hash must fail after a byte flip.  A chunk that still
        // verifies would mean the hashes are decorative, and the runtime would
        // happily render corrupted geometry.
        const bool verified = r.verifyHashes(&err);
        CHECK_FALSE(verified);
        CHECK_FALSE(err.empty());
    } else {
        CHECK(true);   // refusing to open a damaged file is also acceptable
    }
    std::remove(path.c_str());
}
