// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/renderer.h
//
//  Roadmap A1: the Vulkan renderer.  Written against the *verified* chunk
//  format, so everything it consumes has already been proved by the pipeline's
//  own round-trip tests — the renderer is the last consumer in the chain and the
//  first one that needs a GPU.
//
//  WHAT THIS FIRST BUILD DOES (and what it deliberately does not)
//  --------------------------------------------------------------
//  In:   a Vulkan 1.1 device, swapchain, depth prepass + forward pass with the
//        engine's PBR materials sampled from the shipped `.mumbaitex` arrays,
//        an analytic sky, water with a wave-perturbed normal, a directional sun
//        with hemispheric ambient and the terrain's baked AO, dynamic resolution
//        with a 4-tap downsample, ACES filmic tonemap, and instanced traffic
//        drawn from the same model the physics and audio layers run.
//  Out:  clustered lights, WBOIT, TAA and compute post-processing.  They are
//        named here rather than implied: the specification asks for all four,
//        and a reader of this file should be able to see exactly which parts of
//        A1 are still open.  The tone-mapped output is *clean* (no aliasing
//        beyond what the render scale removes), but WBOIT is what will make
//        glass and water compose correctly where they overlap, and TAA is what
//        will let the render scale drop to 0.7 without the shimmer.
//
//  THREE DESIGN RULES, all of them consequences of Section 0
//  ---------------------------------------------------------
//  1. **Nothing is allocated during a frame.**  Every buffer the renderer needs
//     (vertex arenas, instance staging, descriptor pools, command buffers) is
//     created up front and recycled.  A frame that allocates is a frame that can
//     stall, and a stall is a dropped frame, which is an artifact.
//  2. **The floating origin is a renderer-side offset, never a rebase of the
//     data.**  Chunk vertices are cell-local metres (Section 4A); the only place
//     world metres exist is the push constant's `originDelta`, computed once per
//     frame as `cellOrigin - renderOrigin`.  So a macro-cell crossing changes one
//     f32 per draw, and there is no numerical event to pop.
//  3. **A chunk that is not resident is not drawn** — and the renderer never
//     guesses at its contents.  What is not in the arena has no draw call.
//
//  The renderer owns no threads and does not know what a phone is: it takes a
//  window handle as a `void*`, so the Android surface, a Wayland surface or a
//  headless test surface are the same code path from here down.
// =============================================================================
#pragma once
#include "engine/core/core.h"
#include "engine/core/math.h"
#include "engine/render/hud.h"

#include <string>
#include <vector>

namespace mumbai {

class ChunkReader;

namespace render {

// -----------------------------------------------------------------------------
//  Frame inputs
// -----------------------------------------------------------------------------
struct Camera {
    Vec3 pos{0, 30, 0};        // *render-space* metres (already origin-relative)
    f32 yaw = 0.0f;            // radians, world convention: atan2(dx, dz)
    f32 pitch = -0.25f;        // radians, positive looks up
    f32 fovY = 1.0472f;        // 60 degrees
    f32 zNear = 0.25f;
    f32 zFar = 6000.0f;

    // Unit vector the camera looks along, from yaw and pitch.  Exposed because
    // the harness and the tests both need it, and because there is exactly one
    // definition of it in the project (see camera.cpp on handedness).
    Vec3 forward() const;
    Mat4 view() const;
    Mat4 proj(f32 aspect) const;
};

struct Sun {
    // Direction *towards* the sun, unit.  At golden hour the specification's
    // motorcycle shot has the sun ~6 degrees above the horizon and the sky doing
    // most of the lighting, which is what `intensity` and `skyBoost` carry.
    Vec3 direction{0.32f, 0.12f, 0.94f};
    Vec3 colour{1.0f, 0.72f, 0.42f};
    f32 intensity = 3.1f;
    f32 ambient = 0.35f;       // hemispheric fill from the sky
};

struct Environment {
    Sun sun;
    f32 humidity = 0.72f;      // Mie/forward-scatter drive
    f32 pollution = 0.35f;     // turbidity
    f32 exposure = 1.0f;
    Vec3 fogColour{0.62f, 0.68f, 0.78f};
    f32 fogDensity = 0.00028f; // metres^-1, monsoon haze
    f32 wetness = 0.0f;        // 0 dry .. 1 standing water: drives roughness
    f32 time = 0.0f;           // seconds, for the water's own motion
    f32 renderScale = 1.0f;    // dynamic resolution, clamped to [0.6, 2.0]
};

// One traffic vehicle, as the renderer needs it: where, which way, what kind.
// The model's `lanePoint`/`laneHeading` produce the first two.
struct TrafficInstance {
    Vec3 pos{0, 0, 0};         // *render-space* metres
    f32 yaw = 0.0f;
    u32 kind = 0;              // sim::Archetype
};

// What one incremental chunk-streaming call did.  The client calls this every
// second, not every frame: a cell is 1.4 MB of geometry and loading it inside a
// frame is a hitch, which is a thing the acceptance criteria explicitly forbid.
struct WorldLoadStats {
    u32 considered = 0;      // cells inside the radius
    u32 loaded = 0;          // cells added to the scene this call
    u32 failed = 0;          // cells that opened but did not verify
    u32 alreadyResident = 0; // cells counted again
    std::string firstError;
};

struct FrameStats {
    u32 chunksResident = 0;
    u32 drawCalls = 0;
    u32 triangles = 0;
    u32 instances = 0;
    u32 renderWidth = 0, renderHeight = 0;
    u64 gpuBytes = 0;
    f64 cpuFrameMs = 0.0;
};

// -----------------------------------------------------------------------------
//  Renderer
// -----------------------------------------------------------------------------
class Renderer {
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    // `window` is an ANativeWindow* on Android, a VkSurfaceKHR source elsewhere.
    // `dataDir` is where world_manifest.json and cells/ live — on Android that is
    // the external files directory the first-run download populates, because a
    // 1.3 GB world is not an APK payload (D-043).
    bool init(void* window, u32 width, u32 height, const std::string& dataDir,
              std::string* error = nullptr);
    void shutdown();
    bool valid() const;

    // Swapchain recreation.  The platform tells us the surface changed; a resize
    // is not an error and must not lose the scene.
    void resize(u32 width, u32 height);

    // --- scene -----------------------------------------------------------------
    // Registers a chunk's GPU buffers.  The reader must outlive the call; the
    // data is copied into the arenas here and the reader may be closed after.
    bool addChunk(const ChunkReader& reader, const std::string& id, std::string* error = nullptr);
    // Sets the render origin (a multiple of the macro-cell size, normally the
    // camera's own cell origin).  Every chunk's delta is recomputed here, which
    // is the whole cost of a floating-origin shift on the render side: a few
    // thousand vector subtractions, no GPU work, no rebuild.
    void setRenderOrigin(f64 x, f64 z);
    void removeChunk(const std::string& id);
    void clearScene();
    void setTraffic(const std::vector<TrafficInstance>& instances);

    // --- frame -----------------------------------------------------------------
    void setCamera(const Camera& camera) { m_camera = camera; }

    // --- the world index ------------------------------------------------------
    // `openWorld` reads world_manifest.json and remembers where every cell is;
    // `loadWorldNear` then streams in the nearest cells that are not resident yet,
    // bounded by both a radius and a per-call count so the caller stays in control
    // of how long it is willing to spend.  Both are safe to call with no world at
    // all: the index is simply empty and the sky is what gets drawn.
    bool openWorld(const std::string& dataDir, std::string* error = nullptr);
    void closeWorld();
    u32 worldCellCount() const;
    u32 worldCellsInRadius(Vec3 focusWorld, f32 radiusMetres) const;
    WorldLoadStats loadWorldNear(Vec3 focusWorld, f32 radiusMetres, u32 maxCellsPerCall);

    // --- the on-screen panel --------------------------------------------------
    // Copied, not referenced: the client rebuilds its lines once a second and must
    // not have to keep the strings alive.  Drawn last, over the tonemapped frame,
    // with no depth test — it is an interface, not a surface in the world.
    void setHud(const HudFrame& frame);
    // The environment carries the render scale, so a change to it rebuilds the
    // offscreen targets — which is exactly what dynamic resolution has to do,
    // and it happens between frames, never inside one.
    void setEnvironment(const Environment& env);
    // Returns false if the frame could not be presented (a lost swapchain is not
    // a failure: the caller resizes and tries again next frame).
    bool drawFrame(std::string* error = nullptr);

    const FrameStats& stats() const { return m_stats; }
    u32 chunkCount() const;

private:
    struct Impl;
    Impl* m_impl = nullptr;
    Camera m_camera;
    Environment m_environment;
    FrameStats m_stats;
};

}  // namespace render
}  // namespace mumbai
