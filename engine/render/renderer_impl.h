// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/renderer_impl.h
//
//  The Vulkan objects behind `Renderer`.  Nothing in here is public: the app,
//  the harness and the tests all speak `renderer.h`.  Two things in this file
//  are worth a reader's attention because they are decisions, not plumbing:
//
//  1. **One arena per resource kind, not one buffer per chunk.**  A chunk's
//     vertex and index data go into shared device buffers through a first-fit
//     free list.  A streaming world evicts and re-adds the same cells for hours;
//     allocating a `VkBuffer` per chunk per stream-in is how a driver's
//     allocator becomes the frame-time p99.
//  2. **The arena memory is host-visible.**  On every target this project has
//     (Adreno, Mali, and any unified-memory desktop GPU) that is device-local
//     memory and the upload is a `memcpy` with no staging copy at all.  On a
//     discrete GPU it is a slower path, which is recorded here rather than
//     discovered later: the fix is a staging ring, and it is not needed yet.
// =============================================================================
#pragma once
#include <vulkan/vulkan.h>

#include "engine/render/fallback_atlas.h"
#include "engine/render/renderer.h"
#include "engine/world/chunk.h"
#include "engine/world/texture.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace mumbai::render {

inline constexpr u32 kFramesInFlight = 2;
inline constexpr u32 kMaterialArrayLayers = 256;   // the descriptor's array size
// The generated layer count (kFallbackLayers) lives in fallback_atlas.h with the
// pixels it describes.  A material table that references more layers than that
// simply repeats them, which is a wrong surface and never a wrong byte.
inline constexpr u32 kVertexArenaBytes = 256u << 20;
inline constexpr u32 kIndexArenaBytes = 128u << 20;

// -----------------------------------------------------------------------------
//  GPU memory
// -----------------------------------------------------------------------------
struct GpuAlloc {
    VkBuffer buffer = VK_NULL_HANDLE;
    u64 offset = 0;
    u64 size = 0;
};

// A growable, host-visible device buffer with a first-fit free list.
class Arena {
public:
    bool init(VkDevice device, VkPhysicalDeviceMemoryProperties memProps, u64 initialBytes);
    void destroy();
    // Returns false when out of room; the caller grows and retries.
    bool alloc(u64 bytes, u64 alignment, GpuAlloc& out);
    void free(const GpuAlloc& a);
    bool grow(VkDevice device, VkPhysicalDeviceMemoryProperties memProps, u64 atLeast);
    bool upload(const GpuAlloc& a, const void* data, u64 bytes);
    VkBuffer buffer() const { return m_buffer; }
    u64 capacity() const { return m_capacity; }
    u64 used() const { return m_used; }

private:
    struct Free { u64 offset, size; };
    VkDevice m_device = VK_NULL_HANDLE;
    VkBuffer m_buffer = VK_NULL_HANDLE;
    VkDeviceMemory m_memory = VK_NULL_HANDLE;
    u64 m_capacity = 0, m_used = 0;
    std::vector<Free> m_free;
};

// -----------------------------------------------------------------------------
//  Per-chunk GPU residency
// -----------------------------------------------------------------------------
enum DrawKind : u8 {
    kDrawTerrain = 0,
    kDrawRoad = 1,
    kDrawBuilding = 2,
    kDrawRail = 3,
    kDrawWater = 4,
};

struct ChunkGpu {
    // The chunk's macro-cell origin relative to the current render origin, in
    // metres.  This one vector is the whole render-side floating origin.
    Vec3 originDelta{0, 0, 0};
    f32 cellOriginX = 0, cellOriginZ = 0;   // for recomputing the delta on a shift
    std::vector<GpuAlloc> vertexAllocs;
    std::vector<GpuAlloc> indexAllocs;
    // Flattened draw list, one entry per (mesh, submesh) pair, indices only: a
    // submesh bound to its own material is a draw call, and a chunk's whole
    // vertex buffer is bound once.
    struct Draw {
        u32 firstIndex = 0;
        u32 indexCount = 0;
        int32_t vertexOffset = 0;
        u8 materialId = 0;
        u8 kind = 0;            // 0 terrain, 1 road, 2 building, 3 rail, 4 water
    };
    std::vector<Draw> draws;
    u32 triangles = 0;
};

// -----------------------------------------------------------------------------
//  Uniform block (std140).  Keep it 16-byte aligned per member and it matches
//  the GLSL block in world.vert without a single padding surprise.
// -----------------------------------------------------------------------------
struct FrameUbo {
    f32 viewProj[16];
    f32 cameraPos[4];        // xyz + time
    f32 sunDir[4];           // xyz + intensity
    f32 sunColour[4];        // rgb + ambient
    f32 fog[4];              // rgb + density
    // Names match the shaders' block members exactly, on purpose: the layout check
    // (tools/verify_shader_layout.py) compares them member by member, and an alias
    // would have to be taught to the checker, which is how layout checks rot.
    f32 envParams[4];        // humidity, pollution, exposure, wetness
    f32 skyParams[4];        // renderWidth, renderHeight, time, unused
};
static_assert(sizeof(FrameUbo) == 160, "FrameUbo must stay 160 bytes");

struct PushConstants {
    f32 originDelta[4];      // xyz = cellOrigin - renderOrigin, w = flags
};
static_assert(sizeof(PushConstants) == 16, "push constants are one vec4");

// The HUD's push constant block: the viewport in pixels, then two floats that a
// single-atlas HUD does not use yet but that keep the block 16-byte aligned, which
// is what std430 wants a push constant block to be on every driver.
struct HudPush {
    f32 viewportW, viewportH;
    f32 reserved0, reserved1;
};
static_assert(sizeof(HudPush) == 16, "the HUD push constant block is 16 bytes");

// One line of the world index: where a cell is, and how big it is.
struct WorldCellEntry {
    i64 cellX = 0, cellZ = 0;
    u32 bytes = 0;
    std::string file;
};

// A vehicle's instance data: its own transform, packed small.
struct InstanceVertex {
    f32 pos[3];
    f32 yaw;
    f32 scale;
    f32 colour[3];
    u32 kind;
};


// =============================================================================
//  The implementation state.
//
//  This used to sit inside renderer.cpp, which meant frame.cpp, pipeline.cpp and
//  scene.cpp each saw an incomplete private type and could not define a single
//  member of it.  The type is still private to `Renderer` — nothing outside this
//  header should name it — but it has to be complete everywhere its members are
//  written, and that is four translation units.  (Thirty-eight errors on the
//  first compile of A1 said so.)
// =============================================================================
struct Renderer::Impl {
    // --- device -------------------------------------------------------------
    VkInstance instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkPhysicalDevice physical = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    u32 queueFamily = 0;
    VkPhysicalDeviceMemoryProperties memProps{};
    VkPhysicalDeviceProperties props{};

    // --- swapchain ----------------------------------------------------------
    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swapFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D swapExtent{0, 0};
    std::vector<VkImage> swapImages;
    std::vector<VkImageView> swapViews;
    std::vector<VkFramebuffer> swapFramebuffers;
    u32 imageCount = 0;

    // --- offscreen ----------------------------------------------------------
    VkImage hdrImage = VK_NULL_HANDLE;
    VkDeviceMemory hdrMemory = VK_NULL_HANDLE;
    VkImageView hdrView = VK_NULL_HANDLE;
    VkImage depthImage = VK_NULL_HANDLE;
    VkDeviceMemory depthMemory = VK_NULL_HANDLE;
    VkImageView depthView = VK_NULL_HANDLE;
    VkExtent2D renderExtent{0, 0};      // == swapExtent * renderScale

    // --- passes and pipelines ------------------------------------------------
    VkRenderPass prepass = VK_NULL_HANDLE;
    VkRenderPass forwardPass = VK_NULL_HANDLE;
    VkRenderPass postPass = VK_NULL_HANDLE;
    VkFramebuffer prepassFb = VK_NULL_HANDLE;
    VkFramebuffer forwardFb = VK_NULL_HANDLE;

    VkPipelineLayout worldLayout = VK_NULL_HANDLE;
    VkPipelineLayout postLayout = VK_NULL_HANDLE;
    VkPipeline depthPipeline = VK_NULL_HANDLE;
    VkPipeline opaquePipeline = VK_NULL_HANDLE;
    VkPipeline waterPipeline = VK_NULL_HANDLE;
    VkPipeline skyPipeline = VK_NULL_HANDLE;
    VkPipeline tonemapPipeline = VK_NULL_HANDLE;
    VkPipeline trafficPipeline = VK_NULL_HANDLE;
    VkPipeline trafficDepthPipeline = VK_NULL_HANDLE;

    VkDescriptorSetLayout setLayout = VK_NULL_HANDLE;
    VkDescriptorPool descPool = VK_NULL_HANDLE;
    VkDescriptorSet sets[kFramesInFlight]{};

    // --- per-frame -----------------------------------------------------------
    VkCommandPool cmdPool = VK_NULL_HANDLE;
    VkCommandBuffer cmd[kFramesInFlight]{};
    VkSemaphore imageReady[kFramesInFlight]{};
    VkSemaphore renderDone[kFramesInFlight]{};
    VkFence inFlight[kFramesInFlight]{};
    VkBuffer uniformBuffers[kFramesInFlight]{};
    VkDeviceMemory uniformMemory[kFramesInFlight]{};
    void* uniformMapped[kFramesInFlight]{};
    u32 frame = 0;
    u32 imageIndex = 0;
    bool frameActive = false;

    // --- scene resources (defined in scene.cpp) -----------------------------
    Arena vertexArena;
    Arena indexArena;
    std::unordered_map<std::string, ChunkGpu*> chunks;
    VkBuffer trafficBuffer = VK_NULL_HANDLE;
    VkDeviceMemory trafficMemory = VK_NULL_HANDLE;
    void* trafficMapped = nullptr;
    u32 trafficVertexCount = 0;
    u32 trafficInstanceCount = 0;
    u32 trafficInstanceOffset = 0;   // instances start after the mesh vertices
    u32 trafficCapacity = 0;                  // bytes reserved for one mesh set
    u64 trafficInstanceCapacity = 0;          // bytes reserved for instances
    struct MeshRange { u32 first, count; u32 kind; };
    MeshRange trafficMeshes[8]{};
    u32 trafficMeshCount = 0;

    // --- the on-screen panel -------------------------------------------------
    // Its own tiny pipeline and its own descriptor set: one 96x48 single-channel
    // image, one sampler, no uniforms.  It shares nothing with the world's
    // pipeline layout on purpose — a panel that could break when the material
    // array changes size would be a panel that breaks exactly when something has
    // already gone wrong.
    VkPipeline hudPipeline = VK_NULL_HANDLE;
    VkPipelineLayout hudLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout hudSetLayout = VK_NULL_HANDLE;
    VkDescriptorSet hudSet = VK_NULL_HANDLE;
    VkDescriptorPool hudPool = VK_NULL_HANDLE;
    VkImage fontImage = VK_NULL_HANDLE;
    VkDeviceMemory fontMemory = VK_NULL_HANDLE;
    VkImageView fontView = VK_NULL_HANDLE;
    VkBuffer hudBuffer = VK_NULL_HANDLE;
    VkDeviceMemory hudMemory = VK_NULL_HANDLE;
    void* hudMapped = nullptr;
    u64 hudCapacity = 0;
    u32 hudVertexCount = 0;
    HudFrame hud;
    std::vector<HudVertex> hudVertices;   // rebuilt from `hud` when it changes
    bool hudDirty = true;

    // --- the world index (filled by openWorld) -------------------------------
    std::vector<WorldCellEntry> worldCells;
    std::string worldDir;

    // --- textures -----------------------------------------------------------
    VkImage albedoImage = VK_NULL_HANDLE, normalImage = VK_NULL_HANDLE, ormImage = VK_NULL_HANDLE;
    VkDeviceMemory albedoMemory = VK_NULL_HANDLE, normalMemory = VK_NULL_HANDLE, ormMemory = VK_NULL_HANDLE;
    VkImageView albedoView = VK_NULL_HANDLE, normalView = VK_NULL_HANDLE, ormView = VK_NULL_HANDLE;
    VkSampler sampler = VK_NULL_HANDLE;
    u32 materialLayers = 0;

    // --- floating origin and dynamic resolution ------------------------------
    // The render origin is kept in f64 even though the GPU sees f32: it is a
    // whole number of macro-cells (multiples of 1024 m), and the *difference* to
    // a chunk's cell origin is what has to be exact.  f32 would do for today's
    // 45 km world and would stop being exact later; the cast happens once, where
    // the difference is taken.
    f64 renderOriginX = 0.0, renderOriginZ = 0.0;
    f32 renderScale = 1.0f;

    std::string dataDir;
    // Per-frame statistics, filled during recording and copied to FrameStats
    // after present.  They are members rather than locals because the prepass and
    // the forward pass each contribute to them.
    u32 frameDraws = 0;
    u32 frameTriangles = 0;

    FrameUbo ubo{};
    bool swapchainDirty = false;

    // --- helpers ------------------------------------------------------------
    u32 findMemoryType(u32 typeBits, VkMemoryPropertyFlags want) const;
    bool createBuffer(u64 bytes, VkBufferUsageFlags usage, VkMemoryPropertyFlags want,
                      VkBuffer& outBuffer, VkDeviceMemory& outMemory, void** mapped = nullptr);
    VkShaderModule module(const u32* code, size_t words);
    bool buildSwapchain(u32 width, u32 height, std::string* err);
    void destroySwapchain();
    bool buildOffscreen(std::string* err);
    void destroyOffscreen();
    bool buildPasses(std::string* err);
    bool buildPipelines(std::string* err);
    bool buildDescriptors(std::string* err);
    // Written after a resize, not only at init: the tonemap's input is the HDR
    // image, and a new HDR image is a stale descriptor until this runs.
    void writeHdrDescriptor();
    bool buildUniforms(std::string* err);
    bool buildTextures(std::string* err);
    bool buildFallbackTextures(std::string* err);
    bool buildHud(std::string* err);
    void destroyHud();
    void uploadHudVertices();
    bool buildTrafficMesh(std::string* err);
    void recordPrepass(VkCommandBuffer cb);
    void recordForward(VkCommandBuffer cb);
    void recordPost(VkCommandBuffer cb, VkFramebuffer target);
    void clearSceneResources();      // defined in scene.cpp
};

}  // namespace mumbai::render
