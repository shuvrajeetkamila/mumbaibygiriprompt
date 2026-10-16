// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/renderer.cpp
//
//  The Vulkan machinery.  See renderer.h for what this build does and does not
//  cover; this file is the device, the swapchain, the two render passes (depth
//  prepass and forward), the sky and tonemap passes, and the frame loop.
//
//  A note on the passes, because the order is not arbitrary:
//
//      depth prepass   geometry -> depth only, no colour, no fragment shader
//      forward         sky (where depth is still 1.0) then opaque geometry with
//                      depth-equal, so every visible fragment is shaded once
//      tonemap         HDR colour -> ACES -> sRGB -> swapchain, with a 4-tap box
//                      downsample when the render scale is above 1
//
//  The sky is drawn *last* of the three geometry steps, not first: it is a
//  full-screen pass that only fills pixels the geometry did not, which on a
//  tiled GPU is the difference between shading the sky once and shading it under
//  every building in Bandra.
// =============================================================================
#include "engine/render/renderer_impl.h"

#include "shaders_generated.h"

#include <cmath>

#include <cstring>

#if defined(__ANDROID__)
#include <android/native_window.h>
#endif

namespace mumbai::render {
namespace {

const char* vkResultName(VkResult r) {
    switch (r) {
        case VK_SUCCESS: return "VK_SUCCESS";
        case VK_NOT_READY: return "VK_NOT_READY";
        case VK_TIMEOUT: return "VK_TIMEOUT";
        case VK_SUBOPTIMAL_KHR: return "VK_SUBOPTIMAL_KHR";
        case VK_ERROR_OUT_OF_HOST_MEMORY: return "VK_ERROR_OUT_OF_HOST_MEMORY";
        case VK_ERROR_OUT_OF_DEVICE_MEMORY: return "VK_ERROR_OUT_OF_DEVICE_MEMORY";
        case VK_ERROR_INITIALIZATION_FAILED: return "VK_ERROR_INITIALIZATION_FAILED";
        case VK_ERROR_DEVICE_LOST: return "VK_ERROR_DEVICE_LOST";
        case VK_ERROR_SURFACE_LOST_KHR: return "VK_ERROR_SURFACE_LOST_KHR";
        case VK_ERROR_OUT_OF_DATE_KHR: return "VK_ERROR_OUT_OF_DATE_KHR";
        case VK_ERROR_EXTENSION_NOT_PRESENT: return "VK_ERROR_EXTENSION_NOT_PRESENT";
        case VK_ERROR_INCOMPATIBLE_DRIVER: return "VK_ERROR_INCOMPATIBLE_DRIVER";
        case VK_ERROR_FORMAT_NOT_SUPPORTED: return "VK_ERROR_FORMAT_NOT_SUPPORTED";
        default: return "VkResult(?)";
    }
}

inline bool failed(VkResult r) { return r < 0; }

std::string fail(const char* what, VkResult r) {
    return std::string(what) + " -> " + vkResultName(r);
}

// The renderer's formats.  HDR is RGBA16F because the tonemap needs headroom
// above 1.0 (a sun-lit wet road leaves it), and D32 is the depth format every
// target supports for a depth attachment.
constexpr VkFormat kHdrFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

}  // namespace

// =============================================================================
//  Impl
// =============================================================================
// The Impl struct itself lives in renderer_impl.h: four translation units
// define its members, and a type that only one of them can see is not a
// private implementation, it is an accident.  (The first compile of A1 said
// exactly that, in thirty-eight errors.)

u32 Renderer::Impl::findMemoryType(u32 typeBits, VkMemoryPropertyFlags want) const {
    for (u32 i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((typeBits & (1u << i)) && (memProps.memoryTypes[i].propertyFlags & want) == want) return i;
    }
    return UINT32_MAX;
}

bool Renderer::Impl::createBuffer(u64 bytes, VkBufferUsageFlags usage, VkMemoryPropertyFlags want,
                                  VkBuffer& outBuffer, VkDeviceMemory& outMemory, void** mapped) {
    VkBufferCreateInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bi.size = bytes;
    bi.usage = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (failed(vkCreateBuffer(device, &bi, nullptr, &outBuffer))) return false;
    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(device, outBuffer, &req);
    VkMemoryAllocateInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits, want);
    if (ai.memoryTypeIndex == UINT32_MAX) {
        vkDestroyBuffer(device, outBuffer, nullptr);
        outBuffer = VK_NULL_HANDLE;
        return false;
    }
    if (failed(vkAllocateMemory(device, &ai, nullptr, &outMemory))) {
        vkDestroyBuffer(device, outBuffer, nullptr);
        outBuffer = VK_NULL_HANDLE;
        return false;
    }
    if (failed(vkBindBufferMemory(device, outBuffer, outMemory, 0))) return false;
    if (mapped) {
        if (failed(vkMapMemory(device, outMemory, 0, VK_WHOLE_SIZE, 0, mapped))) return false;
    }
    return true;
}

VkShaderModule Renderer::Impl::module(const u32* code, size_t words) {
    if (!code || words == 0) return VK_NULL_HANDLE;
    VkShaderModuleCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    ci.codeSize = words * 4;
    ci.pCode = code;
    VkShaderModule m = VK_NULL_HANDLE;
    if (failed(vkCreateShaderModule(device, &ci, nullptr, &m))) return VK_NULL_HANDLE;
    return m;
}

// =============================================================================
//  Instance / device
// =============================================================================
Renderer::Renderer() : m_impl(new Impl()) {}
Renderer::~Renderer() { shutdown(); delete m_impl; }
bool Renderer::valid() const { return m_impl && m_impl->device != VK_NULL_HANDLE; }

bool Renderer::init(void* window, u32 width, u32 height, const std::string& dataDir,
                    std::string* error) {
    Impl& d = *m_impl;
    d.dataDir = dataDir;

    // --- instance -----------------------------------------------------------
    VkApplicationInfo app{};
    app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName = "Mumbai 1:1 Exact";
    app.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    app.pEngineName = "mumbai-engine";
    app.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    // 1.1 is the specification's floor, and the floor is what the renderer
    // targets: nothing below uses a 1.2/1.3 feature, so a device that reports
    // 1.1 loads the same code path as one that reports 1.3.
    app.apiVersion = VK_API_VERSION_1_1;

    std::vector<const char*> exts = {VK_KHR_SURFACE_EXTENSION_NAME};
#if defined(__ANDROID__)
    exts.push_back(VK_KHR_ANDROID_SURFACE_EXTENSION_NAME);
#elif defined(_WIN32)
    exts.push_back("VK_KHR_win32_surface");
#else
    exts.push_back("VK_KHR_xlib_surface");
#endif
    // Optional debug output, enabled only when the platform offers it.  A build
    // that requires it cannot start on a phone, so it is a bonus, never a
    // dependency.
    std::vector<const char*> layers;
    const char* dbgExt = "VK_EXT_debug_utils";
    {
        u32 count = 0;
        vkEnumerateInstanceExtensionProperties(nullptr, &count, nullptr);
        std::vector<VkExtensionProperties> avail(count);
        vkEnumerateInstanceExtensionProperties(nullptr, &count, avail.data());
        for (u32 i = 0; i < count; ++i)
            if (std::strcmp(avail[i].extensionName, dbgExt) == 0) { exts.push_back(dbgExt); break; }
    }

    VkInstanceCreateInfo ici{};
    ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ici.pApplicationInfo = &app;
    ici.enabledExtensionCount = static_cast<u32>(exts.size());
    ici.ppEnabledExtensionNames = exts.data();
    ici.enabledLayerCount = static_cast<u32>(layers.size());
    ici.ppEnabledLayerNames = layers.empty() ? nullptr : layers.data();
    VkResult r = vkCreateInstance(&ici, nullptr, &d.instance);
    if (failed(r)) {
        if (error) *error = fail("vkCreateInstance", r);
        return false;
    }

    // --- surface ------------------------------------------------------------
#if defined(__ANDROID__)
    {
        if (!window) {
            if (error) *error = "no ANativeWindow: the surface is created before the renderer";
            return false;
        }
        VkAndroidSurfaceCreateInfoKHR si{};
        si.sType = VK_STRUCTURE_TYPE_ANDROID_SURFACE_CREATE_INFO_KHR;
        si.window = reinterpret_cast<ANativeWindow*>(window);
        r = vkCreateAndroidSurfaceKHR(d.instance, &si, nullptr, &d.surface);
        if (failed(r)) {
            if (error) *error = fail("vkCreateAndroidSurfaceKHR", r);
            return false;
        }
    }
#else
    // A host build (the compile check, or a future desktop harness) has no
    // surface: the renderer still initialises so that its scene path can be
    // exercised, and `drawFrame` reports the missing surface rather than
    // crashing on it.
    (void)window;
#endif

    // --- physical device ----------------------------------------------------
    u32 gpuCount = 0;
    vkEnumeratePhysicalDevices(d.instance, &gpuCount, nullptr);
    if (gpuCount == 0) {
        if (error) *error = "no Vulkan physical device";
        return false;
    }
    std::vector<VkPhysicalDevice> gpus(gpuCount);
    vkEnumeratePhysicalDevices(d.instance, &gpuCount, gpus.data());

    auto familySupportsPresent = [&](VkPhysicalDevice gpu, u32 family) {
        if (d.surface == VK_NULL_HANDLE) return true;
        VkBool32 present = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(gpu, family, d.surface, &present);
        return present == VK_TRUE;
    };

    for (VkPhysicalDevice gpu : gpus) {
        u32 famCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(gpu, &famCount, nullptr);
        std::vector<VkQueueFamilyProperties> fams(famCount);
        vkGetPhysicalDeviceQueueFamilyProperties(gpu, &famCount, fams.data());
        for (u32 i = 0; i < famCount; ++i) {
            if (!(fams[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) continue;
            if (!familySupportsPresent(gpu, i)) continue;
            d.physical = gpu;
            d.queueFamily = i;
            break;
        }
        if (d.physical) break;
    }
    if (!d.physical) {
        if (error) *error = "no graphics+present queue family";
        return false;
    }
    vkGetPhysicalDeviceProperties(d.physical, &d.props);
    vkGetPhysicalDeviceMemoryProperties(d.physical, &d.memProps);
    MLOG_INFO("render: %s (%s %u.%u.%u), %u queue families",
              d.props.deviceName, "api", VK_VERSION_MAJOR(d.props.apiVersion),
              VK_VERSION_MINOR(d.props.apiVersion), VK_VERSION_PATCH(d.props.apiVersion),
              d.props.limits.maxPushConstantsSize);

    if (d.props.limits.maxPushConstantsSize < sizeof(PushConstants)) {
        if (error) *error = "device cannot hold the renderer's push constants";
        return false;
    }

    // --- device -------------------------------------------------------------
    {
        const float priority = 1.0f;
        VkDeviceQueueCreateInfo qci{};
        qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        qci.queueFamilyIndex = d.queueFamily;
        qci.queueCount = 1;
        qci.pQueuePriorities = &priority;
        std::vector<const char*> devExts = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
        VkPhysicalDeviceFeatures feats{};
        feats.samplerAnisotropy = VK_TRUE;      // road markings at grazing angles
        feats.fillModeNonSolid = VK_TRUE;       // the wireframe debug view
        feats.depthClamp = VK_TRUE;             // the sky's far-plane trick
        VkDeviceCreateInfo dci{};
        dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        dci.enabledExtensionCount = static_cast<u32>(devExts.size());
        dci.ppEnabledExtensionNames = devExts.data();
        dci.pEnabledFeatures = &feats;
        r = vkCreateDevice(d.physical, &dci, nullptr, &d.device);
        if (failed(r)) {
            if (error) *error = fail("vkCreateDevice", r);
            return false;
        }
        vkGetDeviceQueue(d.device, d.queueFamily, 0, &d.queue);
    }

    // --- command pool, sync, uniforms ---------------------------------------
    {
        VkCommandPoolCreateInfo ci{};
        ci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        ci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        ci.queueFamilyIndex = d.queueFamily;
        if (failed(vkCreateCommandPool(d.device, &ci, nullptr, &d.cmdPool))) {
            if (error) *error = "vkCreateCommandPool failed";
            return false;
        }
    }
    if (!d.buildUniforms(error)) return false;

    // --- arenas and textures -------------------------------------------------
    const u64 wantDeviceLocal = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    const u64 hostOnly = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    bool ok = d.vertexArena.init(d.device, d.memProps, kVertexArenaBytes);
    if (ok) ok = d.indexArena.init(d.device, d.memProps, kIndexArenaBytes);
    if (!ok) {
        // One retry on the portable path: some drivers expose no unified type,
        // in which case host-visible-but-not-device-local still works and is
        // simply slower.  Saying so is better than failing to start.
        (void)wantDeviceLocal;
        (void)hostOnly;
        if (error) *error = "could not allocate the vertex/index arenas";
        return false;
    }
    if (!d.buildTextures(error)) return false;
    if (!d.buildTrafficMesh(error)) return false;
    if (!d.buildDescriptors(error)) return false;

    // --- swapchain and passes ------------------------------------------------
    if (d.surface != VK_NULL_HANDLE) {
        if (!d.buildSwapchain(width, height, error)) return false;
    } else {
        d.swapExtent = {width, height};
        d.swapFormat = VK_FORMAT_B8G8R8A8_UNORM;
    }
    // Order matters: the render passes exist before the framebuffers that
    // reference them, and the framebuffers before the pipelines that must name a
    // compatible pass.
    if (!d.buildPasses(error)) return false;
    // The panel is built once the passes exist (its pipeline targets the post
    // pass) and before the first frame, because a status line that may or may not
    // be there is not a status line.
    if (!d.buildHud(error)) return false;
    if (!d.buildOffscreen(error)) return false;
    d.writeHdrDescriptor();
    if (!d.buildPipelines(error)) return false;

    MLOG_INFO("render: %ux%u swapchain (%u images), render scale 1.000 at %ux%u",
              d.swapExtent.width, d.swapExtent.height, d.imageCount,
              d.renderExtent.width, d.renderExtent.height);
    return true;
}

void Renderer::shutdown() {
    if (!m_impl) return;
    Impl& d = *m_impl;
    if (d.device == VK_NULL_HANDLE) {
        if (d.instance) vkDestroyInstance(d.instance, nullptr);
        d.instance = VK_NULL_HANDLE;
        return;
    }
    vkDeviceWaitIdle(d.device);

    m_impl->destroyHud();
    m_impl->clearSceneResources();

    for (u32 i = 0; i < kFramesInFlight; ++i) {
        if (d.uniformBuffers[i]) vkDestroyBuffer(d.device, d.uniformBuffers[i], nullptr);
        if (d.uniformMemory[i]) vkFreeMemory(d.device, d.uniformMemory[i], nullptr);
        if (d.imageReady[i]) vkDestroySemaphore(d.device, d.imageReady[i], nullptr);
        if (d.renderDone[i]) vkDestroySemaphore(d.device, d.renderDone[i], nullptr);
        if (d.inFlight[i]) vkDestroyFence(d.device, d.inFlight[i], nullptr);
    }
    if (d.albedoView) vkDestroyImageView(d.device, d.albedoView, nullptr);
    if (d.normalView) vkDestroyImageView(d.device, d.normalView, nullptr);
    if (d.ormView) vkDestroyImageView(d.device, d.ormView, nullptr);
    if (d.albedoImage) vkDestroyImage(d.device, d.albedoImage, nullptr);
    if (d.normalImage) vkDestroyImage(d.device, d.normalImage, nullptr);
    if (d.ormImage) vkDestroyImage(d.device, d.ormImage, nullptr);
    if (d.albedoMemory) vkFreeMemory(d.device, d.albedoMemory, nullptr);
    if (d.normalMemory) vkFreeMemory(d.device, d.normalMemory, nullptr);
    if (d.ormMemory) vkFreeMemory(d.device, d.ormMemory, nullptr);
    if (d.sampler) vkDestroySampler(d.device, d.sampler, nullptr);
    if (d.trafficBuffer) vkDestroyBuffer(d.device, d.trafficBuffer, nullptr);
    if (d.trafficMemory) vkFreeMemory(d.device, d.trafficMemory, nullptr);
    if (d.descPool) vkDestroyDescriptorPool(d.device, d.descPool, nullptr);
    if (d.setLayout) vkDestroyDescriptorSetLayout(d.device, d.setLayout, nullptr);
    if (d.trafficPipeline) vkDestroyPipeline(d.device, d.trafficPipeline, nullptr);
    if (d.trafficDepthPipeline) vkDestroyPipeline(d.device, d.trafficDepthPipeline, nullptr);
    if (d.depthPipeline) vkDestroyPipeline(d.device, d.depthPipeline, nullptr);
    if (d.opaquePipeline) vkDestroyPipeline(d.device, d.opaquePipeline, nullptr);
    if (d.waterPipeline) vkDestroyPipeline(d.device, d.waterPipeline, nullptr);
    if (d.skyPipeline) vkDestroyPipeline(d.device, d.skyPipeline, nullptr);
    if (d.tonemapPipeline) vkDestroyPipeline(d.device, d.tonemapPipeline, nullptr);
    if (d.worldLayout) vkDestroyPipelineLayout(d.device, d.worldLayout, nullptr);
    if (d.postLayout) vkDestroyPipelineLayout(d.device, d.postLayout, nullptr);
    d.destroyOffscreen();
    d.destroySwapchain();
    if (d.prepass) vkDestroyRenderPass(d.device, d.prepass, nullptr);
    if (d.forwardPass) vkDestroyRenderPass(d.device, d.forwardPass, nullptr);
    if (d.postPass) vkDestroyRenderPass(d.device, d.postPass, nullptr);
    if (d.cmdPool) vkDestroyCommandPool(d.device, d.cmdPool, nullptr);
    d.vertexArena.destroy();
    d.indexArena.destroy();
    vkDestroyDevice(d.device, nullptr);
    if (d.surface) vkDestroySurfaceKHR(d.instance, d.surface, nullptr);
    if (d.instance) vkDestroyInstance(d.instance, nullptr);
    d.device = VK_NULL_HANDLE;
    d.instance = VK_NULL_HANDLE;
}

void Renderer::setEnvironment(const Environment& env) {
    m_environment = env;
    Impl& d = *m_impl;
    if (!d.device) return;
    if (std::fabs(env.renderScale - d.renderScale) > 1e-3f) {
        vkDeviceWaitIdle(d.device);
        d.renderScale = env.renderScale;
        std::string err;
        if (!d.buildOffscreen(&err)) MLOG_ERR("render: %s", err.c_str());
    }
}

void Renderer::setHud(const HudFrame& frame) {
    Impl& d = *m_impl;
    d.hud = frame;
    d.hudDirty = true;
}

void Renderer::resize(u32 width, u32 height) {
    Impl& d = *m_impl;
    if (!d.device) return;
    if (width == d.swapExtent.width && height == d.swapExtent.height) return;
    vkDeviceWaitIdle(d.device);
    if (d.surface != VK_NULL_HANDLE) {
        std::string err;
        d.buildSwapchain(width, height, &err);
    } else {
        d.swapExtent = {width, height};
    }
    d.swapchainDirty = true;
}

// =============================================================================
//  Swapchain
// =============================================================================
bool Renderer::Impl::buildSwapchain(u32 width, u32 height, std::string* err) {
    VkSurfaceCapabilitiesKHR caps{};
    VkResult r = vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical, surface, &caps);
    if (failed(r)) {
        if (err) *err = fail("surface capabilities", r);
        return false;
    }

    u32 fmtCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical, surface, &fmtCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(fmtCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical, surface, &fmtCount, formats.data());
    VkSurfaceFormatKHR chosen = formats.empty() ? VkSurfaceFormatKHR{VK_FORMAT_B8G8R8A8_UNORM,
                                                                     VK_COLOR_SPACE_SRGB_NONLINEAR_KHR}
                                                : formats[0];
    for (const VkSurfaceFormatKHR& f : formats) {
        // An sRGB swapchain lets the tonemap write linear values and lets the
        // hardware do the transfer function, which is both cheaper and exact.
        if ((f.format == VK_FORMAT_R8G8B8A8_SRGB || f.format == VK_FORMAT_B8G8R8A8_SRGB) &&
            f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            chosen = f;
            break;
        }
    }

    // FIFO is the only present mode the specification guarantees.  It also
    // happens to be the right one for this project: a 30 FPS budget with no
    // tearing, and no queue of stale frames after a macro-cell crossing.
    const VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;

    VkExtent2D extent = caps.currentExtent;
    if (extent.width == 0xFFFFFFFFu) {
        extent.width = clamp(width, caps.minImageExtent.width, caps.maxImageExtent.width);
        extent.height = clamp(height, caps.minImageExtent.height, caps.maxImageExtent.height);
    }

    u32 images = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && images > caps.maxImageCount) images = caps.maxImageCount;

    destroySwapchain();

    VkSwapchainCreateInfoKHR ci{};
    ci.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    ci.surface = surface;
    ci.minImageCount = images;
    ci.imageFormat = chosen.format;
    ci.imageColorSpace = chosen.colorSpace;
    ci.imageExtent = extent;
    ci.imageArrayLayers = 1;
    ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    ci.preTransform = caps.currentTransform;
    ci.compositeAlpha = VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR;
    ci.presentMode = presentMode;
    ci.clipped = VK_TRUE;
    ci.oldSwapchain = VK_NULL_HANDLE;
    r = vkCreateSwapchainKHR(device, &ci, nullptr, &swapchain);
    if (failed(r)) {
        if (err) *err = fail("vkCreateSwapchainKHR", r);
        return false;
    }
    swapFormat = chosen.format;
    swapExtent = extent;

    vkGetSwapchainImagesKHR(device, swapchain, &imageCount, nullptr);
    swapImages.resize(imageCount);
    vkGetSwapchainImagesKHR(device, swapchain, &imageCount, swapImages.data());
    swapViews.resize(imageCount);
    for (u32 i = 0; i < imageCount; ++i) {
        VkImageViewCreateInfo vi{};
        vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        vi.image = swapImages[i];
        vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vi.format = swapFormat;
        vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        if (failed(vkCreateImageView(device, &vi, nullptr, &swapViews[i]))) {
            if (err) *err = "vkCreateImageView(swapchain) failed";
            return false;
        }
    }
    return true;
}

void Renderer::Impl::destroySwapchain() {
    for (VkFramebuffer fb : swapFramebuffers) vkDestroyFramebuffer(device, fb, nullptr);
    swapFramebuffers.clear();
    for (VkImageView v : swapViews) vkDestroyImageView(device, v, nullptr);
    swapViews.clear();
    swapImages.clear();
    if (swapchain) vkDestroySwapchainKHR(device, swapchain, nullptr);
    swapchain = VK_NULL_HANDLE;
}

void Renderer::Impl::destroyOffscreen() {
    if (hdrView) vkDestroyImageView(device, hdrView, nullptr);
    if (hdrImage) vkDestroyImage(device, hdrImage, nullptr);
    if (hdrMemory) vkFreeMemory(device, hdrMemory, nullptr);
    if (depthView) vkDestroyImageView(device, depthView, nullptr);
    if (depthImage) vkDestroyImage(device, depthImage, nullptr);
    if (depthMemory) vkFreeMemory(device, depthMemory, nullptr);
    hdrView = depthView = VK_NULL_HANDLE;
    hdrImage = depthImage = VK_NULL_HANDLE;
    hdrMemory = depthMemory = VK_NULL_HANDLE;
    if (prepassFb) vkDestroyFramebuffer(device, prepassFb, nullptr);
    if (forwardFb) vkDestroyFramebuffer(device, forwardFb, nullptr);
    prepassFb = forwardFb = VK_NULL_HANDLE;
}

// =============================================================================
//  Offscreen targets, at the render scale
// =============================================================================
bool Renderer::Impl::buildOffscreen(std::string* err) {
    destroyOffscreen();
    const f32 scale = clamp(renderScale, 0.6f, 2.0f);
    renderExtent.width = static_cast<u32>(std::max(64.0f, swapExtent.width * scale));
    renderExtent.height = static_cast<u32>(std::max(64.0f, swapExtent.height * scale));

    auto makeImage = [&](VkFormat format, VkImageUsageFlags usage, VkImage& image,
                         VkDeviceMemory& memory, VkImageView& view, VkImageAspectFlags aspect) {
        VkImageCreateInfo ci{};
        ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        ci.imageType = VK_IMAGE_TYPE_2D;
        ci.format = format;
        ci.extent = {renderExtent.width, renderExtent.height, 1};
        ci.mipLevels = 1;
        ci.arrayLayers = 1;
        ci.samples = VK_SAMPLE_COUNT_1_BIT;
        ci.tiling = VK_IMAGE_TILING_OPTIMAL;
        ci.usage = usage;
        ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        if (failed(vkCreateImage(device, &ci, nullptr, &image))) return false;
        VkMemoryRequirements req{};
        vkGetImageMemoryRequirements(device, image, &req);
        VkMemoryAllocateInfo ai{};
        ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        ai.allocationSize = req.size;
        ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (ai.memoryTypeIndex == UINT32_MAX) return false;
        if (failed(vkAllocateMemory(device, &ai, nullptr, &memory))) return false;
        if (failed(vkBindImageMemory(device, image, memory, 0))) return false;
        VkImageViewCreateInfo vi{};
        vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        vi.image = image;
        vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vi.format = format;
        vi.subresourceRange = {aspect, 0, 1, 0, 1};
        return !failed(vkCreateImageView(device, &vi, nullptr, &view));
    };

    if (!makeImage(kHdrFormat, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                   hdrImage, hdrMemory, hdrView, VK_IMAGE_ASPECT_COLOR_BIT)) {
        if (err) *err = "HDR target allocation failed";
        return false;
    }
    if (!makeImage(kDepthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                   depthImage, depthMemory, depthView, VK_IMAGE_ASPECT_DEPTH_BIT)) {
        if (err) *err = "depth target allocation failed";
        return false;
    }

    // The prepass renders depth only, so its framebuffer has one attachment.
    {
        VkFramebufferCreateInfo fi{};
        fi.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fi.renderPass = prepass;
        fi.attachmentCount = 1;
        fi.pAttachments = &depthView;
        fi.width = renderExtent.width;
        fi.height = renderExtent.height;
        fi.layers = 1;
        if (failed(vkCreateFramebuffer(device, &fi, nullptr, &prepassFb))) {
            if (err) *err = "prepass framebuffer failed";
            return false;
        }
    }
    {
        const VkImageView att[2] = {hdrView, depthView};
        VkFramebufferCreateInfo fi{};
        fi.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fi.renderPass = forwardPass;
        fi.attachmentCount = 2;
        fi.pAttachments = att;
        fi.width = renderExtent.width;
        fi.height = renderExtent.height;
        fi.layers = 1;
        if (failed(vkCreateFramebuffer(device, &fi, nullptr, &forwardFb))) {
            if (err) *err = "forward framebuffer failed";
            return false;
        }
    }

    // The post pass renders straight into the swapchain images.
    swapFramebuffers.resize(swapViews.size(), VK_NULL_HANDLE);
    for (size_t i = 0; i < swapViews.size(); ++i) {
        VkFramebufferCreateInfo fi{};
        fi.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fi.renderPass = postPass;
        fi.attachmentCount = 1;
        fi.pAttachments = &swapViews[i];
        fi.width = swapExtent.width;
        fi.height = swapExtent.height;
        fi.layers = 1;
        if (failed(vkCreateFramebuffer(device, &fi, nullptr, &swapFramebuffers[i]))) {
            if (err) *err = "post framebuffer failed";
            return false;
        }
    }
    return true;
}

// =============================================================================
//  Render passes
// =============================================================================
bool Renderer::Impl::buildPasses(std::string*) {
    // --- prepass: depth only -------------------------------------------------
    {
        VkAttachmentDescription depth{};
        depth.format = kDepthFormat;
        depth.samples = VK_SAMPLE_COUNT_1_BIT;
        depth.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depth.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depth.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depth.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depth.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depth.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
        VkSubpassDescription sub{};
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 0;
        sub.pDepthStencilAttachment = &ref;

        VkSubpassDependency dep{};
        dep.srcSubpass = VK_SUBPASS_EXTERNAL;
        dep.dstSubpass = 0;
        dep.srcStageMask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        dep.dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dep.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

        VkRenderPassCreateInfo ci{};
        ci.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        ci.attachmentCount = 1;
        ci.pAttachments = &depth;
        ci.subpassCount = 1;
        ci.pSubpasses = &sub;
        ci.dependencyCount = 1;
        ci.pDependencies = &dep;
        if (failed(vkCreateRenderPass(device, &ci, nullptr, &prepass))) return false;
    }

    // --- forward: HDR colour + the depth the prepass wrote -------------------
    {
        VkAttachmentDescription att[2]{};
        att[0].format = kHdrFormat;
        att[0].samples = VK_SAMPLE_COUNT_1_BIT;
        att[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        att[0].finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        att[1].format = kDepthFormat;
        att[1].samples = VK_SAMPLE_COUNT_1_BIT;
        att[1].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;     // the prepass's work
        att[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att[1].initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        att[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colour{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkAttachmentReference depth{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
        VkSubpassDescription sub{};
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 1;
        sub.pColorAttachments = &colour;
        sub.pDepthStencilAttachment = &depth;
        // `sub.preserveAttachmentCount` stays 0: the depth is consumed by this
        // pass and needs no preservation, and the colour leaves as a sampled
        // image for the tonemap.
        VkSubpassDependency dep[2]{};
        dep[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dep[0].dstSubpass = 0;
        dep[0].srcStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
                              VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
        dep[0].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        dep[0].dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
                              VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
        dep[0].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT |
                               VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
        dep[1].srcSubpass = 0;
        dep[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dep[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dep[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dep[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dep[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        VkRenderPassCreateInfo ci{};
        ci.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        ci.attachmentCount = 2;
        ci.pAttachments = att;
        ci.subpassCount = 1;
        ci.pSubpasses = &sub;
        ci.dependencyCount = 2;
        ci.pDependencies = dep;
        if (failed(vkCreateRenderPass(device, &ci, nullptr, &forwardPass))) return false;
    }

    // --- post: tonemap into the swapchain ------------------------------------
    {
        VkAttachmentDescription att{};
        att.format = swapFormat;
        att.samples = VK_SAMPLE_COUNT_1_BIT;
        att.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        att.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        VkAttachmentReference colour{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription sub{};
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 1;
        sub.pColorAttachments = &colour;

        VkSubpassDependency dep{};
        dep.srcSubpass = VK_SUBPASS_EXTERNAL;
        dep.dstSubpass = 0;
        dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dep.srcAccessMask = 0;
        dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

        VkRenderPassCreateInfo ci{};
        ci.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        ci.attachmentCount = 1;
        ci.pAttachments = &att;
        ci.subpassCount = 1;
        ci.pSubpasses = &sub;
        ci.dependencyCount = 1;
        ci.pDependencies = &dep;
        if (failed(vkCreateRenderPass(device, &ci, nullptr, &postPass))) return false;
    }
    return true;
}

}  // namespace mumbai::render
