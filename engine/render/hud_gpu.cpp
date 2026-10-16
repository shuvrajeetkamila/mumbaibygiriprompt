// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/hud_gpu.cpp
//
//  The panel's GPU side: one image, one sampler, one descriptor, one pipeline and
//  one small vertex buffer that is rewritten only when the text changes.
//
//  Two decisions are worth stating, because both go against the obvious choice:
//
//  1. **The vertex buffer is host-visible and persistent, not an arena
//     allocation.**  The panel is rewritten once a second and drawn once a frame;
//     there is nothing to stream, nothing to evict, and putting a 60 KB buffer in
//     the middle of a 256 MB arena would make the arena's free list and the
//     panel's lifetime depend on each other for no reason.
//  2. **The atlas is an R8 image, not an RGBA one.**  The glyph mask is one
//     channel; colour comes from the vertex.  Four times less data to upload at
//     startup and no colour-space question about glyph antialiasing, because
//     there is no colour in the texture to have a colour space.
// =============================================================================
#include "engine/render/renderer_impl.h"

#include <cstring>

namespace mumbai::render {
namespace {

VkShaderModule makeModule(VkDevice device, const u32* code, size_t words) {
    if (!code || words == 0) return VK_NULL_HANDLE;
    VkShaderModuleCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    ci.codeSize = words * sizeof(u32);
    ci.pCode = code;
    VkShaderModule m = VK_NULL_HANDLE;
    if (vkCreateShaderModule(device, &ci, nullptr, &m) != VK_SUCCESS) return VK_NULL_HANDLE;
    return m;
}

VkPipelineShaderStageCreateInfo stage(VkShaderStageFlagBits s, VkShaderModule m) {
    VkPipelineShaderStageCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    ci.stage = s;
    ci.module = m;
    ci.pName = "main";
    return ci;
}

}  // namespace

// =============================================================================
//  Building it
// =============================================================================
bool Renderer::Impl::buildHud(std::string* err) {
    // --- the atlas ------------------------------------------------------------
    std::vector<u8> pixels;
    buildFontAtlas(pixels);

    VkImageCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ci.imageType = VK_IMAGE_TYPE_2D;
    ci.format = VK_FORMAT_R8_UNORM;
    ci.extent = {kAtlasW, kAtlasH, 1};
    ci.mipLevels = 1;
    ci.arrayLayers = 1;
    ci.samples = VK_SAMPLE_COUNT_1_BIT;
    ci.tiling = VK_IMAGE_TILING_OPTIMAL;
    ci.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateImage(device, &ci, nullptr, &fontImage) != VK_SUCCESS) {
        if (err) *err = "font atlas image failed";
        return false;
    }
    VkMemoryRequirements req{};
    vkGetImageMemoryRequirements(device, fontImage, &req);
    u32 type = UINT32_MAX;
    for (u32 i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((req.memoryTypeBits & (1u << i)) &&
            (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
            type = i;
            break;
        }
    }
    if (type == UINT32_MAX) {
        if (err) *err = "no device-local memory for the font atlas";
        return false;
    }
    VkMemoryAllocateInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = type;
    if (vkAllocateMemory(device, &ai, nullptr, &fontMemory) != VK_SUCCESS ||
        vkBindImageMemory(device, fontImage, fontMemory, 0) != VK_SUCCESS) {
        if (err) *err = "font atlas memory failed";
        return false;
    }
    VkImageViewCreateInfo vi{};
    vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    vi.image = fontImage;
    vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vi.format = ci.format;
    vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    if (vkCreateImageView(device, &vi, nullptr, &fontView) != VK_SUCCESS) {
        if (err) *err = "font atlas view failed";
        return false;
    }

    // --- the pixels (one staging buffer, one submit, one wait) ----------------
    {
        const u64 bytes = static_cast<u64>(kAtlasW) * kAtlasH;
        VkBuffer sb = VK_NULL_HANDLE;
        VkDeviceMemory sm = VK_NULL_HANDLE;
        VkBufferCreateInfo bi{};
        bi.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bi.size = bytes;
        bi.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        if (vkCreateBuffer(device, &bi, nullptr, &sb) != VK_SUCCESS) {
            if (err) *err = "font staging buffer failed";
            return false;
        }
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
        void* mapped = nullptr;
        if (sa.memoryTypeIndex == UINT32_MAX || vkAllocateMemory(device, &sa, nullptr, &sm) != VK_SUCCESS ||
            vkBindBufferMemory(device, sb, sm, 0) != VK_SUCCESS ||
            vkMapMemory(device, sm, 0, bytes, 0, &mapped) != VK_SUCCESS) {
            if (err) *err = "font staging memory failed";
            return false;
        }
        std::memcpy(mapped, pixels.data(), static_cast<size_t>(bytes));
        vkUnmapMemory(device, sm);

        // A one-shot command buffer: the atlas exists before the first frame, and
        // a renderer whose text might or might not be there on frame one is a
        // renderer with a bug waiting for the slowest device.
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
            b.image = fontImage;
            b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
            b.srcAccessMask = 0;
            b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 0, 0, nullptr, 0, nullptr, 1, &b);
            VkBufferImageCopy region{};
            region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            region.imageExtent = {kAtlasW, kAtlasH, 1};
            vkCmdCopyBufferToImage(cb, sb, fontImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
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
        if (!ok) {
            if (err) *err = "font atlas upload failed";
            return false;
        }
    }

    // --- descriptor -----------------------------------------------------------
    {
        VkDescriptorSetLayoutBinding b{};
        b.binding = 0;
        b.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        b.descriptorCount = 1;
        b.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo li{};
        li.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        li.bindingCount = 1;
        li.pBindings = &b;
        if (vkCreateDescriptorSetLayout(device, &li, nullptr, &hudSetLayout) != VK_SUCCESS) {
            if (err) *err = "HUD descriptor layout failed";
            return false;
        }
        VkDescriptorPoolSize ps{};
        ps.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        ps.descriptorCount = 1;
        VkDescriptorPoolCreateInfo pi{};
        pi.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        pi.maxSets = 1;
        pi.poolSizeCount = 1;
        pi.pPoolSizes = &ps;
        VkDescriptorPool pool = VK_NULL_HANDLE;
        if (vkCreateDescriptorPool(device, &pi, nullptr, &pool) != VK_SUCCESS) {
            if (err) *err = "HUD descriptor pool failed";
            return false;
        }
        VkDescriptorSetAllocateInfo ai{};
        ai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        ai.descriptorPool = pool;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts = &hudSetLayout;
        if (vkAllocateDescriptorSets(device, &ai, &hudSet) != VK_SUCCESS) {
            if (err) *err = "HUD descriptor set failed";
            return false;
        }
        VkDescriptorImageInfo ii{};
        ii.sampler = sampler;
        ii.imageView = fontView;
        ii.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkWriteDescriptorSet w{};
        w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w.dstSet = hudSet;
        w.dstBinding = 0;
        w.descriptorCount = 1;
        w.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w.pImageInfo = &ii;
        vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
        // The pool is intentionally leaked into the device: it owns one set and
        // lives exactly as long as the device does, and destroying it here would
        // free the set we just wrote.
        hudPool = pool;
    }

    // --- the vertex buffer ----------------------------------------------------
    hudCapacity = 256u << 10;   // 256 KB is about 13 000 glyph-pixel quads
    if (!createBuffer(hudCapacity, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                      hudBuffer, hudMemory, &hudMapped)) {
        if (err) *err = "HUD vertex buffer failed";
        return false;
    }

    // --- the pipeline ---------------------------------------------------------
    {
        VkPipelineLayoutCreateInfo li{};
        li.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        li.setLayoutCount = 1;
        li.pSetLayouts = &hudSetLayout;
        VkPushConstantRange pcr{};
        pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        pcr.offset = 0;
        pcr.size = sizeof(HudPush);
        li.pushConstantRangeCount = 1;
        li.pPushConstantRanges = &pcr;
        if (vkCreatePipelineLayout(device, &li, nullptr, &hudLayout) != VK_SUCCESS) {
            if (err) *err = "HUD pipeline layout failed";
            return false;
        }

        VkShaderModule vs = makeModule(device, kHudVertSpv, kHudVertSpvWords);
        VkShaderModule fs = makeModule(device, kHudFragSpv, kHudFragSpvWords);
        if (!vs || !fs) {
            if (vs) vkDestroyShaderModule(device, vs, nullptr);
            if (fs) vkDestroyShaderModule(device, fs, nullptr);
            // A build with no GLSL compiler embeds empty arrays.  Saying which
            // shader is missing is the difference between a five-minute fix and a
            // black screen on a phone.
            if (err) *err = std::string("HUD shaders are missing") +
                            (vs ? "" : " (hud.vert)") + (fs ? "" : " (hud.frag)");
            return false;
        }
        VkPipelineShaderStageCreateInfo stages[2] = {stage(VK_SHADER_STAGE_VERTEX_BIT, vs),
                                                     stage(VK_SHADER_STAGE_FRAGMENT_BIT, fs)};

        VkVertexInputBindingDescription vb{};
        vb.binding = 0;
        vb.stride = sizeof(HudVertex);
        vb.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        VkVertexInputAttributeDescription va[3]{};
        va[0] = {0, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(HudVertex, x)};
        va[1] = {1, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(HudVertex, u)};
        va[2] = {2, 0, VK_FORMAT_R8G8B8A8_UNORM, offsetof(HudVertex, rgba)};
        VkPipelineVertexInputStateCreateInfo vin{};
        vin.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vin.vertexBindingDescriptionCount = 1;
        vin.pVertexBindingDescriptions = &vb;
        vin.vertexAttributeDescriptionCount = 3;
        vin.pVertexAttributeDescriptions = va;

        VkPipelineInputAssemblyStateCreateInfo ia{};
        ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo vps{};
        vps.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        vps.viewportCount = 1;
        vps.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rs{};
        rs.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rs.polygonMode = VK_POLYGON_MODE_FILL;
        rs.cullMode = VK_CULL_MODE_NONE;
        rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rs.lineWidth = 1.0f;

        VkPipelineMultisampleStateCreateInfo ms{};
        ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo dss{};
        dss.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        dss.depthTestEnable = VK_FALSE;
        dss.depthWriteEnable = VK_FALSE;

        VkPipelineColorBlendAttachmentState ba{};
        ba.blendEnable = VK_TRUE;
        ba.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        ba.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        ba.colorBlendOp = VK_BLEND_OP_ADD;
        ba.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        ba.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        ba.alphaBlendOp = VK_BLEND_OP_ADD;
        ba.colorWriteMask = 0xF;
        VkPipelineColorBlendStateCreateInfo cb{};
        cb.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        cb.attachmentCount = 1;
        cb.pAttachments = &ba;

        const VkDynamicState dyn[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo ds{};
        ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        ds.dynamicStateCount = 2;
        ds.pDynamicStates = dyn;

        VkGraphicsPipelineCreateInfo gci{};
        gci.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        gci.stageCount = 2;
        gci.pStages = stages;
        gci.pVertexInputState = &vin;
        gci.pInputAssemblyState = &ia;
        gci.pViewportState = &vps;
        gci.pRasterizationState = &rs;
        gci.pMultisampleState = &ms;
        gci.pDepthStencilState = &dss;
        gci.pColorBlendState = &cb;
        gci.pDynamicState = &ds;
        gci.layout = hudLayout;
        gci.renderPass = postPass;   // drawn over the tonemapped frame
        gci.subpass = 0;
        const VkResult r = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &gci, nullptr, &hudPipeline);
        vkDestroyShaderModule(device, vs, nullptr);
        vkDestroyShaderModule(device, fs, nullptr);
        if (r != VK_SUCCESS) {
            if (err) *err = "HUD pipeline creation failed";
            return false;
        }
    }

    MLOG_INFO("hud: %ux%u atlas, %u glyphs, %.1f KiB vertex buffer", kAtlasW, kAtlasH, kGlyphCount,
              static_cast<f64>(hudCapacity) / 1024.0);
    return true;
}

void Renderer::Impl::destroyHud() {
    if (device == VK_NULL_HANDLE) return;
    hud = HudFrame{};
    hudVertices.clear();
    hudVertexCount = 0;
    hudDirty = true;
    if (hudPipeline) vkDestroyPipeline(device, hudPipeline, nullptr);
    if (hudLayout) vkDestroyPipelineLayout(device, hudLayout, nullptr);
    if (hudSetLayout) vkDestroyDescriptorSetLayout(device, hudSetLayout, nullptr);
    if (hudPool) vkDestroyDescriptorPool(device, hudPool, nullptr);
    if (fontView) vkDestroyImageView(device, fontView, nullptr);
    if (fontImage) vkDestroyImage(device, fontImage, nullptr);
    if (fontMemory) vkFreeMemory(device, fontMemory, nullptr);
    if (hudBuffer) vkDestroyBuffer(device, hudBuffer, nullptr);
    if (hudMemory) vkFreeMemory(device, hudMemory, nullptr);
    hudPipeline = VK_NULL_HANDLE;
    hudLayout = VK_NULL_HANDLE;
    hudSetLayout = VK_NULL_HANDLE;
    hudPool = VK_NULL_HANDLE;
    hudSet = VK_NULL_HANDLE;
    fontView = VK_NULL_HANDLE;
    fontImage = VK_NULL_HANDLE;
    fontMemory = VK_NULL_HANDLE;
    hudBuffer = VK_NULL_HANDLE;
    hudMemory = VK_NULL_HANDLE;
    hudMapped = nullptr;
    hudCapacity = 0;
}

void Renderer::Impl::uploadHudVertices() {
    if (!hudDirty) return;
    hudDirty = false;
    hudVertices.clear();
    const f32 w = static_cast<f32>(swapExtent.width);
    const f32 h = static_cast<f32>(swapExtent.height);
    buildHudVertices(hud, w, h, hudVertices);

    const u64 bytes = hudVertices.size() * sizeof(HudVertex);
    if (bytes > hudCapacity) {
        // Grow once, generously: a panel that resizes its buffer every time the
        // frame counter gains a digit would allocate during play.
        u64 want = hudCapacity ? hudCapacity : (64u << 10);
        while (want < bytes) want *= 2;
        if (hudBuffer) vkDestroyBuffer(device, hudBuffer, nullptr);
        if (hudMemory) vkFreeMemory(device, hudMemory, nullptr);
        hudBuffer = VK_NULL_HANDLE;
        hudMemory = VK_NULL_HANDLE;
        hudMapped = nullptr;
        if (!createBuffer(want, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          hudBuffer, hudMemory, &hudMapped)) {
            hudCapacity = 0;
            hudVertexCount = 0;
            MLOG_ERR("hud: vertex buffer grow to %llu bytes failed; the panel is off",
                     static_cast<unsigned long long>(want));
            return;
        }
        hudCapacity = want;
    }
    if (bytes > 0 && hudMapped) std::memcpy(hudMapped, hudVertices.data(), static_cast<size_t>(bytes));
    hudVertexCount = static_cast<u32>(hudVertices.size());
}

}  // namespace mumbai::render
