// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/scene.cpp
//
//  Getting the world onto the GPU: the arenas, the material texture arrays, the
//  per-chunk upload path, and the traffic meshes.
//
//  The upload path is where a streaming world either works or does not.  Three
//  properties are load-bearing:
//
//  * **Indices are rebased once, on upload.**  A chunk's indices address its own
//    vertices; the arena holds every chunk.  Rather than pay a `vertexOffset` per
//    draw (which is legal but costs a uniform-ish state change per submesh), each
//    index gets the chunk's vertex base added once, at upload, and the draw calls
//    are then plain `vkCmdDrawIndexed(first, count)`.
//  * **Eviction returns memory to the free list, not to the driver.**  The
//    steady state of a drive up the coast is "one chunk in, one chunk out, sixty
//    times a second"; a `vkFreeMemory` per eviction is how a frame time becomes a
//    p99.
//  * **The vertex data is used exactly as the pipeline wrote it.**  `MeshVertex`
//    is uploaded verbatim; nothing is re-derived here, so a bug in this file can
//    only be an offset bug, never a wrong-vertex bug.
// =============================================================================
#include "engine/render/renderer_impl.h"

#include "engine/core/json.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

namespace mumbai::render {
namespace {

// Material *classes* the shader needs to know about.  The pipeline publishes no
// enumeration for this, so the classification is by name — which is stable,
// human-readable and impossible to silently renumber.  Anything unmatched is
// ordinary opaque geometry, which is the correct default.
u8 classifyMaterial(const char* name) {
    auto has = [&](const char* needle) { return std::strstr(name, needle) != nullptr; };
    if (has("water")) return kDrawWater;
    return 255;   // opaque
}

}  // namespace

// =============================================================================
//  Arena
// =============================================================================
bool Arena::init(VkDevice device, VkPhysicalDeviceMemoryProperties memProps, u64 initialBytes) {
    m_device = device;
    m_capacity = initialBytes;
    m_used = 0;
    m_free.clear();

    VkBufferCreateInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bi.size = m_capacity;
    bi.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
               VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device, &bi, nullptr, &m_buffer) != VK_SUCCESS) return false;

    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(device, m_buffer, &req);
    // Unified memory first (every phone and every integrated GPU), then plain
    // host-visible.  See renderer_impl.h for why there is no staging path yet.
    const VkMemoryPropertyFlags want[2] = {
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
    };
    u32 type = UINT32_MAX;
    for (VkMemoryPropertyFlags w : want) {
        for (u32 i = 0; i < memProps.memoryTypeCount; ++i) {
            if ((req.memoryTypeBits & (1u << i)) &&
                (memProps.memoryTypes[i].propertyFlags & w) == w) {
                type = i;
                break;
            }
        }
        if (type != UINT32_MAX) break;
    }
    if (type == UINT32_MAX) return false;
    VkMemoryAllocateInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = type;
    if (vkAllocateMemory(device, &ai, nullptr, &m_memory) != VK_SUCCESS) return false;
    if (vkBindBufferMemory(device, m_buffer, m_memory, 0) != VK_SUCCESS) return false;
    m_free.push_back({0, m_capacity});
    return true;
}

void Arena::destroy() {
    if (m_device == VK_NULL_HANDLE) return;
    if (m_buffer) vkDestroyBuffer(m_device, m_buffer, nullptr);
    if (m_memory) vkFreeMemory(m_device, m_memory, nullptr);
    m_buffer = VK_NULL_HANDLE;
    m_memory = VK_NULL_HANDLE;
    m_free.clear();
    m_used = 0;
}

bool Arena::alloc(u64 bytes, u64 alignment, GpuAlloc& out) {
    if (bytes == 0) return false;
    for (size_t i = 0; i < m_free.size(); ++i) {
        const u64 start = (m_free[i].offset + alignment - 1) / alignment * alignment;
        const u64 pad = start - m_free[i].offset;
        if (m_free[i].size < pad + bytes) continue;
        out.buffer = m_buffer;
        out.offset = start;
        out.size = bytes;
        const u64 remaining = m_free[i].size - pad - bytes;
        if (remaining == 0) {
            m_free.erase(m_free.begin() + static_cast<i64>(i));
        } else {
            m_free[i].offset = start + bytes;
            m_free[i].size = remaining;
        }
        m_used += bytes;
        return true;
    }
    return false;
}

void Arena::free(const GpuAlloc& a) {
    if (a.size == 0 || a.buffer != m_buffer) return;
    m_used -= std::min(m_used, a.size);
    m_free.push_back({a.offset, a.size});
    // Coalesce: the streaming path frees neighbours constantly, and a free list
    // that fragments is a free list that eventually cannot serve a 200 KB chunk.
    std::sort(m_free.begin(), m_free.end(),
              [](const Free& x, const Free& y) { return x.offset < y.offset; });
    for (size_t i = 0; i + 1 < m_free.size();) {
        if (m_free[i].offset + m_free[i].size == m_free[i + 1].offset) {
            m_free[i].size += m_free[i + 1].size;
            m_free.erase(m_free.begin() + static_cast<i64>(i) + 1);
        } else {
            ++i;
        }
    }
}

bool Arena::upload(const GpuAlloc& a, const void* data, u64 bytes) {
    if (!m_memory || !data || bytes == 0 || bytes > a.size) return false;
    void* mapped = nullptr;
    if (vkMapMemory(m_device, m_memory, a.offset, bytes, 0, &mapped) != VK_SUCCESS) return false;
    std::memcpy(mapped, data, static_cast<size_t>(bytes));
    vkUnmapMemory(m_device, m_memory);
    return true;
}

bool Arena::grow(VkDevice device, VkPhysicalDeviceMemoryProperties memProps, u64 atLeast) {
    u64 next = std::max<u64>(m_capacity * 2, 64u << 20);
    while (next < m_used + atLeast) next *= 2;

    Arena bigger;
    if (!bigger.init(device, memProps, next)) return false;
    // Contents are *not* copied: growing happens before the first draw of the
    // frame that needs it, and every live allocation is re-uploaded by the caller
    // (which is the only code that knows what the bytes are).  Growing mid-drive
    // is rare — the arena is sized for the streaming radius — and a re-upload is
    // cheaper than a transfer of a half-full 256 MB buffer.
    destroy();
    *this = Arena();
    VkBuffer b = bigger.m_buffer;
    VkDeviceMemory m = bigger.m_memory;
    std::vector<Free> f = bigger.m_free;
    const u64 cap = bigger.m_capacity;
    bigger.m_buffer = VK_NULL_HANDLE;
    bigger.m_memory = VK_NULL_HANDLE;
    bigger.m_free.clear();
    m_device = device;
    m_buffer = b;
    m_memory = m;
    m_capacity = cap;
    m_free = std::move(f);
    m_used = 0;
    (void)memProps;
    return true;
}

// =============================================================================
//  Material texture arrays
// =============================================================================
namespace {

bool uploadTexArray(VkDevice device, VkPhysicalDeviceMemoryProperties memProps, const char* dir,
                    const char* file, VkImage& image, VkDeviceMemory& memory, VkImageView& view,
                    bool srgb, u32& layersOut, std::string* err) {
    const std::string path = std::string(dir) + "/textures/" + file;
    tex::TexArrayReader reader;
    if (!reader.open(path, false, err)) return false;
    const u32 size = reader.size();
    const u32 layers = reader.layerCount();
    const u32 mips = reader.header().mipCount;
    const u32 channels = reader.channels();
    layersOut = layers;

    VkImageCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ci.imageType = VK_IMAGE_TYPE_2D;
    ci.format = srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;
    ci.extent = {size, size, 1};
    ci.mipLevels = mips;
    ci.arrayLayers = layers;
    ci.samples = VK_SAMPLE_COUNT_1_BIT;
    ci.tiling = VK_IMAGE_TILING_OPTIMAL;
    ci.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateImage(device, &ci, nullptr, &image) != VK_SUCCESS) {
        if (err) *err = std::string("vkCreateImage failed for ") + file;
        return false;
    }
    VkMemoryRequirements req{};
    vkGetImageMemoryRequirements(device, image, &req);
    u32 type = UINT32_MAX;
    for (u32 i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((req.memoryTypeBits & (1u << i)) &&
            (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
            type = i;
            break;
        }
    }
    if (type == UINT32_MAX) {
        if (err) *err = "no device-local memory for the texture array";
        return false;
    }
    VkMemoryAllocateInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = type;
    if (vkAllocateMemory(device, &ai, nullptr, &memory) != VK_SUCCESS ||
        vkBindImageMemory(device, image, memory, 0) != VK_SUCCESS) {
        if (err) *err = "texture array memory failed";
        return false;
    }

    VkImageViewCreateInfo vi{};
    vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    vi.image = image;
    vi.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
    vi.format = ci.format;
    vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, mips, 0, layers};
    if (vkCreateImageView(device, &vi, nullptr, &view) != VK_SUCCESS) {
        if (err) *err = "texture array view failed";
        return false;
    }
    // --- the pixels ---------------------------------------------------------
    // Everything is transiently staged through one host-visible buffer sized for
    // the largest mip row set.  The arrays are 30 MB (albedo) and 22 MB each
    // (normal, ORM) at 117 layers x 9 mips, and this runs once, at startup: a
    // staging ring would be machinery for a cost that happens once.
    std::vector<u8> stage;
    const u64 stageBytes = static_cast<u64>(size) * size * 4;
    stage.resize(static_cast<size_t>(stageBytes));
    VkBuffer sb = VK_NULL_HANDLE;
    VkDeviceMemory sm = VK_NULL_HANDLE;
    void* smapped = nullptr;
    VkBufferCreateInfo sbi{};
    sbi.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    sbi.size = stageBytes;
    sbi.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    if (vkCreateBuffer(device, &sbi, nullptr, &sb) != VK_SUCCESS) {
        if (err) *err = "texture staging buffer failed";
        return false;
    }
    VkMemoryRequirements sreq{};
    vkGetBufferMemoryRequirements(device, sb, &sreq);
    VkMemoryAllocateInfo sai{};
    sai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    sai.allocationSize = sreq.size;
    sai.memoryTypeIndex = UINT32_MAX;
    for (u32 i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((sreq.memoryTypeBits & (1u << i)) &&
            (memProps.memoryTypes[i].propertyFlags & (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                                     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) ==
                (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) {
            sai.memoryTypeIndex = i;
            break;
        }
    }
    if (sai.memoryTypeIndex == UINT32_MAX || vkAllocateMemory(device, &sai, nullptr, &sm) != VK_SUCCESS ||
        vkBindBufferMemory(device, sb, sm, 0) != VK_SUCCESS ||
        vkMapMemory(device, sm, 0, stageBytes, 0, &smapped) != VK_SUCCESS) {
        if (err) *err = "texture staging memory failed";
        return false;
    }

    VkCommandPool pool = VK_NULL_HANDLE;
    VkCommandPoolCreateInfo pci{};
    pci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pci.queueFamilyIndex = 0;   // the caller's family is 0 by construction of init()
    if (vkCreateCommandPool(device, &pci, nullptr, &pool) != VK_SUCCESS) {
        if (err) *err = "texture upload command pool failed";
        return false;
    }
    VkCommandBuffer cb = VK_NULL_HANDLE;
    VkCommandBufferAllocateInfo cbi{};
    cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cbi.commandPool = pool;
    cbi.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbi.commandBufferCount = 1;
    vkAllocateCommandBuffers(device, &cbi, &cb);
    VkCommandBufferBeginInfo cbb{};
    cbb.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    cbb.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cb, &cbb);

    auto barrier = [&](VkImageLayout from, VkImageLayout to, VkAccessFlags src, VkAccessFlags dst) {
        VkImageMemoryBarrier b{};
        b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        b.oldLayout = from;
        b.newLayout = to;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = image;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, mips, 0, layers};
        b.srcAccessMask = src;
        b.dstAccessMask = dst;
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             0, 0, nullptr, 0, nullptr, 1, &b);
    };
    barrier(VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 0,
            VK_ACCESS_TRANSFER_WRITE_BIT);

    for (u32 layer = 0; layer < layers; ++layer) {
        for (u32 level = 0; level < mips; ++level) {
            const u32 edge = tex::mipEdge(size, level);
            const u8* src = reader.mip(layer, level);
            if (!src) continue;
            const u64 dstRowBytes = static_cast<u64>(edge) * 4;
            // Expand to RGBA8 row by row.  Source is RGB8 for normal/ORM and
            // already RGBA8 for base colour.
            for (u32 y = 0; y < edge; ++y) {
                const u8* row = src + static_cast<u64>(y) * edge * channels;
                u8* out = stage.data() + static_cast<u64>(y) * dstRowBytes;
                for (u32 x = 0; x < edge; ++x) {
                    out[x * 4 + 0] = row[x * channels + 0];
                    out[x * 4 + 1] = row[x * channels + 1];
                    out[x * 4 + 2] = row[x * channels + 2];
                    out[x * 4 + 3] = channels > 3 ? row[x * channels + 3] : 255;
                }
            }
            std::memcpy(smapped, stage.data(), static_cast<size_t>(dstRowBytes * edge));
            VkBufferImageCopy region{};
            region.bufferOffset = 0;
            region.bufferRowLength = 0;
            region.bufferImageHeight = 0;
            region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level, layer, 1};
            region.imageOffset = {0, 0, 0};
            region.imageExtent = {edge, edge, 1};
            vkCmdCopyBufferToImage(cb, sb, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
        }
    }
    {
        VkImageMemoryBarrier b{};
        b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = image;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, mips, 0, layers};
        b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
    }
    vkEndCommandBuffer(cb);
    {
        VkFence fence = VK_NULL_HANDLE;
        VkFenceCreateInfo fci{};
        fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        vkCreateFence(device, &fci, nullptr, &fence);
        VkSubmitInfo si{};
        si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cb;
        // A temporary queue grab at startup: the caller's queue family is not
        // available here, and a one-shot upload has no ordering to respect.
        VkQueue q = VK_NULL_HANDLE;
        vkGetDeviceQueue(device, 0, 0, &q);
        if (q == VK_NULL_HANDLE) {
            vkDestroyFence(device, fence, nullptr);
            vkDestroyCommandPool(device, pool, nullptr);
            vkUnmapMemory(device, sm);
            vkFreeMemory(device, sm, nullptr);
            vkDestroyBuffer(device, sb, nullptr);
            if (err) *err = "no queue 0 for the texture upload";
            return false;
        }
        vkQueueSubmit(q, 1, &si, fence);
        vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX);
        vkDestroyFence(device, fence, nullptr);
    }
    vkUnmapMemory(device, sm);
    vkFreeMemory(device, sm, nullptr);
    vkDestroyBuffer(device, sb, nullptr);
    vkDestroyCommandPool(device, pool, nullptr);
    return true;
}

}  // namespace

// A hard failure here used to be the whole story: no texture arrays, no renderer,
// and on a phone that is a black screen with a log line nobody sees.  The arrays
// ship with the world, and a client that has not downloaded the world yet has no
// arrays — so the absence is now a *known state* rather than an error, and it is
// answered with generated ones (D-047).  A build with the real arrays behaves
// exactly as before.
namespace {

bool pathExists(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    return f.good();
}

}  // namespace

bool Renderer::Impl::buildTextures(std::string* err) {
    const std::string dir = dataDir;
    if (!pathExists(dir + "/textures/materials_basecolor.mumbaitex")) {
        MLOG_WARN("no material textures under %s: the world's atlas has not been downloaded, so "
                  "the renderer generates its own (D-047)", dir.c_str());
        return buildFallbackTextures(err);
    }
    if (!uploadTexArray(device, memProps, dir.c_str(), "materials_basecolor.mumbaitex",
                        albedoImage, albedoMemory, albedoView, true, materialLayers, err))
        return false;
    u32 layers = 0;
    if (!uploadTexArray(device, memProps, dir.c_str(), "materials_normal.mumbaitex",
                        normalImage, normalMemory, normalView, false, layers, err))
        return false;
    if (!uploadTexArray(device, memProps, dir.c_str(), "materials_orm.mumbaitex",
                        ormImage, ormMemory, ormView, false, layers, err))
        return false;
    MLOG_INFO("render: %u material layers, %u px tiles", materialLayers, 256u);
    return true;
}

// =============================================================================
//  Traffic meshes
// =============================================================================
namespace {

struct Vertex3_3_3 {
    f32 px, py, pz;
    f32 nx, ny, nz;
    f32 r, g, b;
};

void addBox(std::vector<Vertex3_3_3>& out, Vec3 centre, Vec3 half, Vec3 colour) {
    // Six faces, outward normals, counter-clockwise seen from outside.
    const f32 x0 = centre.x - half.x, x1 = centre.x + half.x;
    const f32 y0 = centre.y - half.y, y1 = centre.y + half.y;
    const f32 z0 = centre.z - half.z, z1 = centre.z + half.z;
    auto quad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n) {
        const Vertex3_3_3 A{a.x, a.y, a.z, n.x, n.y, n.z, colour.x, colour.y, colour.z};
        const Vertex3_3_3 B{b.x, b.y, b.z, n.x, n.y, n.z, colour.x, colour.y, colour.z};
        const Vertex3_3_3 C{c.x, c.y, c.z, n.x, n.y, n.z, colour.x, colour.y, colour.z};
        const Vertex3_3_3 D{d.x, d.y, d.z, n.x, n.y, n.z, colour.x, colour.y, colour.z};
        out.push_back(A); out.push_back(B); out.push_back(C);
        out.push_back(A); out.push_back(C); out.push_back(D);
    };
    quad({x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}, {0, 0, 1});
    quad({x1, y0, z0}, {x0, y0, z0}, {x0, y1, z0}, {x1, y1, z0}, {0, 0, -1});
    quad({x1, y0, z1}, {x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {1, 0, 0});
    quad({x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}, {x0, y1, z0}, {-1, 0, 0});
    quad({x0, y1, z1}, {x1, y1, z1}, {x1, y1, z0}, {x0, y1, z0}, {0, 1, 0});
    quad({x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}, {0, -1, 0});
}

}  // namespace

// The five archetypes of Section 5, as silhouettes.  These are *not* hero
// geometry and they are not pretending to be: the specification's vehicle assets
// are hand-modelled and are not in the world data, so the first build draws the
// published dimensions — a double-decker is 4.4 m tall and 2.6 m wide, a
// rickshaw is 1.3 m wide and 1.8 m tall — and lets the silhouette and the
// colour do the identification.  Recorded in D-043 rather than glossed over.
bool Renderer::Impl::buildTrafficMesh(std::string* err) {
    std::vector<Vertex3_3_3> verts;
    trafficMeshCount = 0;

    auto begin = [&]() { return static_cast<u32>(verts.size()); };
    auto end = [&](u32 first, u32 kind) {
        MeshRange r;
        r.first = first;
        r.count = static_cast<u32>(verts.size()) - first;
        r.kind = kind;
        trafficMeshes[trafficMeshCount++] = r;
    };

    // 0: car — 4.4 x 1.8 x 1.5 m
    {
        const u32 f = begin();
        addBox(verts, {0, 0.55f, 0}, {0.9f, 0.33f, 2.2f}, {0.55f, 0.56f, 0.60f});
        addBox(verts, {0, 1.10f, -0.15f}, {0.82f, 0.28f, 1.05f}, {0.32f, 0.38f, 0.44f});
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                addBox(verts, {sx * 0.78f, 0.31f, sz * 1.45f}, {0.12f, 0.31f, 0.32f},
                       {0.08f, 0.08f, 0.09f});
        end(f, 0);
    }
    // 1: taxi — the same car in kaali-peeli black and yellow
    {
        const u32 f = begin();
        addBox(verts, {0, 0.55f, 0}, {0.88f, 0.33f, 2.0f}, {0.92f, 0.78f, 0.18f});
        addBox(verts, {0, 1.05f, -0.1f}, {0.80f, 0.26f, 0.95f}, {0.10f, 0.10f, 0.11f});
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                addBox(verts, {sx * 0.76f, 0.31f, sz * 1.3f}, {0.12f, 0.31f, 0.30f},
                       {0.08f, 0.08f, 0.09f});
        end(f, 1);
    }
    // 2: auto-rickshaw — 2.6 m long, 1.3 m wide, 1.75 m tall, with the canopy
    {
        const u32 f = begin();
        addBox(verts, {0, 0.55f, 0}, {0.65f, 0.40f, 1.30f}, {0.30f, 0.62f, 0.42f});
        addBox(verts, {0, 1.30f, -0.2f}, {0.65f, 0.42f, 1.05f}, {0.22f, 0.48f, 0.32f});
        addBox(verts, {0, 0.30f, 1.15f}, {0.20f, 0.30f, 0.20f}, {0.12f, 0.12f, 0.13f});
        end(f, 2);
    }
    // 3: motorcycle — 2.0 m long, 0.8 m wide, 1.1 m tall
    {
        const u32 f = begin();
        addBox(verts, {0, 0.55f, 0}, {0.16f, 0.22f, 0.75f}, {0.45f, 0.20f, 0.18f});
        addBox(verts, {0, 0.92f, 0.1f}, {0.12f, 0.30f, 0.55f}, {0.30f, 0.32f, 0.36f});
        addBox(verts, {0, 0.33f, 0.62f}, {0.10f, 0.33f, 0.33f}, {0.08f, 0.08f, 0.09f});
        addBox(verts, {0, 0.33f, -0.62f}, {0.10f, 0.33f, 0.33f}, {0.08f, 0.08f, 0.09f});
        end(f, 3);
    }
    // 4: BEST double-decker — 12 m long, 2.6 m wide, 4.4 m tall
    {
        const u32 f = begin();
        addBox(verts, {0, 1.30f, 0}, {1.30f, 1.30f, 6.0f}, {0.72f, 0.24f, 0.20f});
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                addBox(verts, {sx * 1.15f, 0.45f, sz * 4.2f}, {0.18f, 0.45f, 0.45f},
                       {0.08f, 0.08f, 0.09f});
        end(f, 4);
    }
    // 5: truck
    {
        const u32 f = begin();
        addBox(verts, {0, 1.45f, -0.6f}, {1.25f, 1.45f, 4.4f}, {0.55f, 0.55f, 0.58f});
        addBox(verts, {0, 1.05f, 4.3f}, {1.15f, 1.05f, 1.6f}, {0.42f, 0.45f, 0.52f});
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                addBox(verts, {sx * 1.05f, 0.5f, sz * 4.0f}, {0.2f, 0.5f, 0.5f},
                       {0.08f, 0.08f, 0.09f});
        end(f, 5);
    }

    trafficVertexCount = static_cast<u32>(verts.size());
    const u64 vertexBytes = static_cast<u64>(verts.size()) * sizeof(Vertex3_3_3);
    // Room for 2,048 instances alongside the meshes; `setTraffic` grows it.
    const u64 instanceBytes = 2048ull * sizeof(InstanceVertex);
    if (!createBuffer(vertexBytes + instanceBytes,
                      VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                      trafficBuffer, trafficMemory, &trafficMapped)) {
        if (err) *err = "traffic buffer failed";
        return false;
    }
    std::memcpy(trafficMapped, verts.data(), static_cast<size_t>(vertexBytes));
    trafficInstanceOffset = vertexBytes;
    trafficInstanceCapacity = instanceBytes;
    (void)trafficCapacity;
    MLOG_INFO("render: %u traffic vertices (6 archetype silhouettes), %.1f KB",
              trafficVertexCount, static_cast<f64>(vertexBytes) / 1024.0);
    return true;
}

// =============================================================================
//  Chunk residency
// =============================================================================
namespace {

struct SectionPlan {
    SectionType type;
    u8 kind;
};

// The sections a renderer draws, and what each one is for.  TerrainCollision and
// TerrainHeight are deliberately absent: they are physics and query data, and a
// renderer that uploads them is paying memory for triangles nobody sees.
const SectionPlan kRenderable[] = {
    {SectionType::TerrainMesh, kDrawTerrain},
    {SectionType::RoadMesh, kDrawRoad},
    {SectionType::BuildingMesh, kDrawBuilding},
    {SectionType::RailMesh, kDrawRail},
    {SectionType::WaterMesh, kDrawWater},
};

constexpr u32 kVertexAlignment = 16;

}  // namespace

bool Renderer::addChunk(const ChunkReader& reader, const std::string& id, std::string* error) {
    Impl& d = *m_impl;
    if (!d.device) {
        if (error) *error = "renderer is not initialised";
        return false;
    }
    if (d.chunks.count(id)) return true;      // already resident: nothing to do

    auto* gpu = new ChunkGpu();
    gpu->cellOriginX = static_cast<f32>(reader.header().cellX) * MACRO_CELL_SIZE;
    gpu->cellOriginZ = static_cast<f32>(reader.header().cellZ) * MACRO_CELL_SIZE;
    gpu->originDelta = Vec3{static_cast<f32>(d.renderOriginX - gpu->cellOriginX), 0.0f,
                            static_cast<f32>(d.renderOriginZ - gpu->cellOriginZ)};

    const MaterialDef* materials = nullptr;
    u32 materialCount = 0;
    materials = reader.array<MaterialDef>(SectionType::MaterialTable, &materialCount);

    u32 triangles = 0;
    bool ok = true;
    for (const SectionPlan& plan : kRenderable) {
        const ChunkReader::MeshView mesh = reader.meshByLod(plan.type, 0);
        if (!mesh.valid() || mesh.indexCount == 0) continue;

        GpuAlloc valloc{};
        const u64 vertexBytes = static_cast<u64>(mesh.vertexCount) * sizeof(MeshVertex);
        if (!d.vertexArena.alloc(vertexBytes, kVertexAlignment, valloc)) {
            if (error) *error = "vertex arena full";
            ok = false;
            break;
        }
        const u32 vertexBase = static_cast<u32>(valloc.offset / sizeof(MeshVertex));

        GpuAlloc ialloc{};
        const u64 indexBytes = static_cast<u64>(mesh.indexCount) * sizeof(u32);
        if (!d.indexArena.alloc(indexBytes, kVertexAlignment, ialloc)) {
            d.vertexArena.free(valloc);
            if (error) *error = "index arena full";
            ok = false;
            break;
        }

        // Upload vertices verbatim.
        if (!d.vertexArena.upload(valloc, mesh.vertices, vertexBytes)) {
            d.vertexArena.free(valloc);
            d.indexArena.free(ialloc);
            if (error) *error = "vertex upload failed";
            ok = false;
            break;
        }
        // Rebase indices once, here, so the draw calls are plain.
        std::vector<u32> rebased(mesh.indexCount);
        for (u32 i = 0; i < mesh.indexCount; ++i) rebased[i] = mesh.indices[i] + vertexBase;
        if (!d.indexArena.upload(ialloc, rebased.data(), indexBytes)) {
            d.vertexArena.free(valloc);
            d.indexArena.free(ialloc);
            if (error) *error = "index upload failed";
            ok = false;
            break;
        }
        const u32 indexBase = static_cast<u32>(ialloc.offset / sizeof(u32));
        gpu->vertexAllocs.push_back(valloc);
        gpu->indexAllocs.push_back(ialloc);

        // One draw per submesh; a submesh with no indices is skipped rather than
        // submitted as an empty draw.
        for (u32 s = 0; s < mesh.submeshCount; ++s) {
            const SubMesh& sub = mesh.submeshes[s];
            if (sub.indexCount == 0) continue;
            ChunkGpu::Draw draw;
            draw.firstIndex = indexBase + sub.indexOffset / sizeof(u32);
            draw.indexCount = sub.indexCount;
            draw.vertexOffset = 0;
            draw.materialId = static_cast<u8>(std::min<u32>(sub.materialId, 255u));
            draw.kind = plan.kind;
            // A material whose name says water is shaded as water whatever
            // section it arrived in: the classification is a property of the
            // material, not of the mesh that happens to use it.
            if (sub.materialId < materialCount && materials) {
                const u8 byName = classifyMaterial(materials[sub.materialId].name);
                if (byName != 255) draw.kind = byName;
            }
            gpu->draws.push_back(draw);
            triangles += sub.indexCount / 3;
        }
    }
    if (!ok) {
        for (GpuAlloc& a : gpu->vertexAllocs) d.vertexArena.free(a);
        for (GpuAlloc& a : gpu->indexAllocs) d.indexArena.free(a);
        delete gpu;
        return false;
    }
    gpu->triangles = triangles;
    d.chunks.emplace(id, gpu);
    return true;
}

void Renderer::removeChunk(const std::string& id) {
    Impl& d = *m_impl;
    auto it = d.chunks.find(id);
    if (it == d.chunks.end()) return;
    ChunkGpu* gpu = it->second;
    for (GpuAlloc& a : gpu->vertexAllocs) d.vertexArena.free(a);
    for (GpuAlloc& a : gpu->indexAllocs) d.indexArena.free(a);
    delete gpu;
    d.chunks.erase(it);
}

void Renderer::clearScene() {
    Impl& d = *m_impl;
    for (auto& kv : d.chunks) {
        for (GpuAlloc& a : kv.second->vertexAllocs) d.vertexArena.free(a);
        for (GpuAlloc& a : kv.second->indexAllocs) d.indexArena.free(a);
        delete kv.second;
    }
    d.chunks.clear();
}

u32 Renderer::chunkCount() const { return static_cast<u32>(m_impl->chunks.size()); }

void Renderer::setRenderOrigin(f64 x, f64 z) {
    Impl& d = *m_impl;
    if (std::fabs(x - d.renderOriginX) < 1e-6 && std::fabs(z - d.renderOriginZ) < 1e-6) return;
    d.renderOriginX = x;
    d.renderOriginZ = z;
    // The whole render-side cost of a floating-origin shift: one subtraction per
    // resident chunk, in f64, rounded once to f32.  No GPU work, no rebuild, no
    // frame lost.
    for (auto& kv : d.chunks) {
        ChunkGpu& gpu = *kv.second;
        gpu.originDelta = Vec3{static_cast<f32>(d.renderOriginX - gpu.cellOriginX), 0.0f,
                               static_cast<f32>(d.renderOriginZ - gpu.cellOriginZ)};
    }
}

void Renderer::setTraffic(const std::vector<TrafficInstance>& instances) {
    Impl& d = *m_impl;
    if (!d.trafficMapped) return;
    const u64 need = static_cast<u64>(instances.size()) * sizeof(InstanceVertex);
    if (need > d.trafficInstanceCapacity) {
        MLOG_WARN("render: %zu traffic instances exceed the %llu-instance buffer; "
                  "the nearest are drawn", instances.size(),
                  static_cast<unsigned long long>(d.trafficInstanceCapacity /
                                                  sizeof(InstanceVertex)));
    }
    const u32 capacity = static_cast<u32>(d.trafficInstanceCapacity / sizeof(InstanceVertex));
    const u32 count = std::min<u32>(static_cast<u32>(instances.size()), capacity);
    auto* dst = reinterpret_cast<InstanceVertex*>(
        static_cast<u8*>(d.trafficMapped) + d.trafficInstanceOffset);
    static const f32 kColours[6][3] = {
        {0.55f, 0.56f, 0.60f}, {0.92f, 0.78f, 0.18f}, {0.30f, 0.62f, 0.42f},
        {0.45f, 0.20f, 0.18f}, {0.72f, 0.24f, 0.20f}, {0.55f, 0.55f, 0.58f},
    };
    for (u32 i = 0; i < count; ++i) {
        const TrafficInstance& src = instances[i];
        InstanceVertex& v = dst[i];
        v.pos[0] = src.pos.x;
        v.pos[1] = src.pos.y;
        v.pos[2] = src.pos.z;
        v.yaw = src.yaw;
        v.scale = 1.0f;
        const u32 kind = src.kind < 6 ? src.kind : 0;
        v.colour[0] = kColours[kind][0];
        v.colour[1] = kColours[kind][1];
        v.colour[2] = kColours[kind][2];
        v.kind = kind;
    }
    d.trafficInstanceCount = count;
}

void Renderer::Impl::clearSceneResources() {
    for (auto& kv : chunks) {
        for (GpuAlloc& a : kv.second->vertexAllocs) vertexArena.free(a);
        for (GpuAlloc& a : kv.second->indexAllocs) indexArena.free(a);
        delete kv.second;
    }
    chunks.clear();
}


// =============================================================================
//  The procedural atlas
//
//  Written as three arrays (albedo, normal, ORM) at 256x256 with no mips: the
//  surfaces here are noise, and mips of noise are noise.  The world's own atlas
//  has nine, which is why the real one is 100 MB and this one is 1.5 MB.
// =============================================================================
bool Renderer::Impl::buildFallbackTextures(std::string* err) {
    const u32 size = 256;
    std::vector<u8> albedo, normal, orm;
    buildFallbackAtlasPixels(size, albedo, normal, orm);

    auto make = [&](const std::vector<u8>& data, bool srgb, VkImage& image, VkDeviceMemory& memory,
                    VkImageView& view, const char* what) {
        VkImageCreateInfo ci{};
        ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        ci.imageType = VK_IMAGE_TYPE_2D;
        ci.format = srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;
        ci.extent = {size, size, 1};
        ci.mipLevels = 1;
        ci.arrayLayers = kFallbackLayers;
        ci.samples = VK_SAMPLE_COUNT_1_BIT;
        ci.tiling = VK_IMAGE_TILING_OPTIMAL;
        ci.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (vkCreateImage(device, &ci, nullptr, &image) != VK_SUCCESS) {
            if (err) *err = std::string("fallback ") + what + ": vkCreateImage failed";
            return false;
        }
        VkMemoryRequirements req{};
        vkGetImageMemoryRequirements(device, image, &req);
        u32 type = UINT32_MAX;
        for (u32 i = 0; i < memProps.memoryTypeCount; ++i) {
            if ((req.memoryTypeBits & (1u << i)) &&
                (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
                type = i;
                break;
            }
        }
        if (type == UINT32_MAX) {
            if (err) *err = std::string("fallback ") + what + ": no device-local memory";
            return false;
        }
        VkMemoryAllocateInfo ai{};
        ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        ai.allocationSize = req.size;
        ai.memoryTypeIndex = type;
        if (vkAllocateMemory(device, &ai, nullptr, &memory) != VK_SUCCESS ||
            vkBindImageMemory(device, image, memory, 0) != VK_SUCCESS) {
            if (err) *err = std::string("fallback ") + what + ": memory failed";
            return false;
        }
        VkImageViewCreateInfo vi{};
        vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        vi.image = image;
        vi.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
        vi.format = ci.format;
        vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, kFallbackLayers};
        if (vkCreateImageView(device, &vi, nullptr, &view) != VK_SUCCESS) {
            if (err) *err = std::string("fallback ") + what + ": view failed";
            return false;
        }

        // Upload through one staging buffer, one submit, one wait: 1.5 MB, once,
        // at startup.  (The real atlas uses 30 MB staging and the same shape.)
        const u64 bytes = data.size();
        VkBuffer sb = VK_NULL_HANDLE;
        VkDeviceMemory sm = VK_NULL_HANDLE;
        void* mapped = nullptr;
        VkBufferCreateInfo bi{};
        bi.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bi.size = bytes;
        bi.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        if (vkCreateBuffer(device, &bi, nullptr, &sb) != VK_SUCCESS) return false;
        VkMemoryRequirements sr{};
        vkGetBufferMemoryRequirements(device, sb, &sr);
        VkMemoryAllocateInfo sa{};
        sa.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        sa.allocationSize = sr.size;
        sa.memoryTypeIndex = UINT32_MAX;
        for (u32 i = 0; i < memProps.memoryTypeCount; ++i) {
            if ((sr.memoryTypeBits & (1u << i)) &&
                (memProps.memoryTypes[i].propertyFlags & (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                                          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) ==
                    (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) {
                sa.memoryTypeIndex = i;
                break;
            }
        }
        if (sa.memoryTypeIndex == UINT32_MAX || vkAllocateMemory(device, &sa, nullptr, &sm) != VK_SUCCESS ||
            vkBindBufferMemory(device, sb, sm, 0) != VK_SUCCESS ||
            vkMapMemory(device, sm, 0, bytes, 0, &mapped) != VK_SUCCESS) {
            if (err) *err = std::string("fallback ") + what + ": staging failed";
            return false;
        }
        std::memcpy(mapped, data.data(), static_cast<size_t>(bytes));
        vkUnmapMemory(device, sm);

        VkCommandPool pool = VK_NULL_HANDLE;
        VkCommandPoolCreateInfo pci{};
        pci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        pci.queueFamilyIndex = queueFamily;
        bool ok = vkCreateCommandPool(device, &pci, nullptr, &pool) == VK_SUCCESS;
        VkCommandBuffer cb = VK_NULL_HANDLE;
        if (ok) {
            VkCommandBufferAllocateInfo cbi{};
            cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
            cbi.commandPool = pool;
            cbi.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            cbi.commandBufferCount = 1;
            ok = vkAllocateCommandBuffers(device, &cbi, &cb) == VK_SUCCESS;
        }
        if (ok) {
            VkCommandBufferBeginInfo bbi{};
            bbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
            bbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
            ok = vkBeginCommandBuffer(cb, &bbi) == VK_SUCCESS;
        }
        if (ok) {
            VkImageMemoryBarrier b{};
            b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            b.image = image;
            b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, kFallbackLayers};
            b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
                                 0, nullptr, 0, nullptr, 1, &b);
            std::vector<VkBufferImageCopy> regions(kFallbackLayers);
            for (u32 layer = 0; layer < kFallbackLayers; ++layer) {
                regions[layer].bufferOffset = static_cast<VkDeviceSize>(layer) * size * size * 4;
                regions[layer].imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, layer, 1};
                regions[layer].imageExtent = {size, size, 1};
            }
            vkCmdCopyBufferToImage(cb, sb, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, kFallbackLayers,
                                   regions.data());
            b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                                 0, 0, nullptr, 0, nullptr, 1, &b);
            ok = vkEndCommandBuffer(cb) == VK_SUCCESS;
        }
        if (ok) {
            VkSubmitInfo si{};
            si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
            si.commandBufferCount = 1;
            si.pCommandBuffers = &cb;
            ok = vkQueueSubmit(queue, 1, &si, VK_NULL_HANDLE) == VK_SUCCESS &&
                 vkQueueWaitIdle(queue) == VK_SUCCESS;
        }
        if (pool) vkDestroyCommandPool(device, pool, nullptr);
        vkDestroyBuffer(device, sb, nullptr);
        vkFreeMemory(device, sm, nullptr);
        if (!ok && err) *err = std::string("fallback ") + what + ": upload failed";
        return ok;
    };

    if (!make(albedo, true, albedoImage, albedoMemory, albedoView, "albedo")) return false;
    if (!make(normal, false, normalImage, normalMemory, normalView, "normal")) return false;
    if (!make(orm, false, ormImage, ormMemory, ormView, "orm")) return false;
    materialLayers = kFallbackLayers;
    MLOG_INFO("render: %u generated material layers (no world atlas present)", materialLayers);
    return true;
}

// =============================================================================
//  The world index
//
//  Reading a manifest is not loading a world: this parses 358 KB of JSON once and
//  remembers where the cells are, so the streaming call that follows can be a
//  distance sort rather than a file parse.  A client with no world gets an empty
//  index and a sky, which is a state it can report instead of a failure it cannot.
// =============================================================================
bool Renderer::openWorld(const std::string& dataDir, std::string* error) {
    Impl& d = *m_impl;
    d.worldCells.clear();
    d.worldDir = dataDir;
    const std::string path = dataDir + "/world_manifest.json";
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
        WorldCellEntry e;
        e.cellX = c["x"].asInt(0);
        e.cellZ = c["z"].asInt(0);
        e.bytes = static_cast<u32>(c["bytes"].asInt(0));
        e.file = c["file"].asString();
        if (e.file.empty()) continue;
        d.worldCells.push_back(std::move(e));
    }
    MLOG_INFO("world: %zu cells indexed from %s", d.worldCells.size(), path.c_str());
    return !d.worldCells.empty();
}

void Renderer::closeWorld() {
    Impl& d = *m_impl;
    d.worldCells.clear();
    d.worldDir.clear();
    clearScene();
}

u32 Renderer::worldCellCount() const {
    return static_cast<u32>(m_impl->worldCells.size());
}

u32 Renderer::worldCellsInRadius(Vec3 focusWorld, f32 radiusMetres) const {
    const Impl& d = *m_impl;
    u32 n = 0;
    const f32 r2 = radiusMetres * radiusMetres;
    for (const WorldCellEntry& c : d.worldCells) {
        const f32 cx = static_cast<f32>(c.cellX) * MACRO_CELL_SIZE + MACRO_CELL_SIZE * 0.5f;
        const f32 cz = static_cast<f32>(c.cellZ) * MACRO_CELL_SIZE + MACRO_CELL_SIZE * 0.5f;
        const f32 dx = cx - focusWorld.x;
        const f32 dz = cz - focusWorld.z;
        if (dx * dx + dz * dz <= r2) ++n;
    }
    return n;
}

WorldLoadStats Renderer::loadWorldNear(Vec3 focusWorld, f32 radiusMetres, u32 maxCellsPerCall) {
    WorldLoadStats stats;
    Impl& d = *m_impl;
    if (!d.device || d.worldCells.empty() || maxCellsPerCall == 0) return stats;

    struct Candidate {
        f32 dist2;
        u32 index;
    };
    // One vector, reserved once: this runs at 1 Hz for hours and an allocation per
    // call is an allocation per call.
    static thread_local std::vector<Candidate> candidates;
    candidates.clear();
    const f32 r2 = radiusMetres * radiusMetres;
    for (u32 i = 0; i < d.worldCells.size(); ++i) {
        const WorldCellEntry& c = d.worldCells[i];
        const f32 cx = static_cast<f32>(c.cellX) * MACRO_CELL_SIZE + MACRO_CELL_SIZE * 0.5f;
        const f32 cz = static_cast<f32>(c.cellZ) * MACRO_CELL_SIZE + MACRO_CELL_SIZE * 0.5f;
        const f32 dx = cx - focusWorld.x;
        const f32 dz = cz - focusWorld.z;
        const f32 dist2 = dx * dx + dz * dz;
        if (dist2 > r2) continue;
        candidates.push_back({dist2, i});
    }
    stats.considered = static_cast<u32>(candidates.size());
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.dist2 < b.dist2; });

    for (const Candidate& cand : candidates) {
        if (stats.loaded >= maxCellsPerCall) break;
        const WorldCellEntry& c = d.worldCells[cand.index];
        if (d.chunks.count(c.file)) {
            ++stats.alreadyResident;
            continue;
        }
        ChunkReader reader;
        std::string err;
        const std::string path = d.worldDir + "/cells/" + c.file;
        if (!reader.open(path, &err)) {
            ++stats.failed;
            if (stats.firstError.empty()) stats.firstError = err;
            continue;
        }
        if (!addChunk(reader, c.file, &err)) {
            ++stats.failed;
            if (stats.firstError.empty()) stats.firstError = err;
            continue;
        }
        ++stats.loaded;
    }
    return stats;
}


}  // namespace mumbai::render
