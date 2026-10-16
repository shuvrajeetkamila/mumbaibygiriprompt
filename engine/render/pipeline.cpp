// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/pipeline.cpp
//
//  Pipelines, descriptors, the uniform block, and the frame loop.
//
//  Two choices in here are worth defending.
//
//  **Cull mode is NONE.**  The pipeline's mesh writers do not record a winding
//  convention in the chunk format, and a renderer that guesses wrong does not
//  produce a slightly-wrong image — it produces a world with the buildings
//  missing, which reads as a loader bug and would be chased in the wrong file
//  for a day.  With the depth prepass in front, backfaces cost a raster but no
//  shading, so the honest default is to draw both sides and revisit when the
//  winding is written into the format.  It is recorded here rather than left for
//  someone to rediscover.
//
//  **One vertex buffer and one index buffer are bound for the whole frame.**
//  Every chunk lives in the two shared arenas (see renderer_impl.h), so the
//  frame's bind state is set twice, not per chunk.  Per-chunk state is a single
//  push-constant update: the chunk's cell origin relative to the render origin,
//  which is the entire floating-origin implementation on the render side.
// =============================================================================
#include "engine/render/renderer_impl.h"

#include "shaders_generated.h"

#include <cstring>

namespace mumbai::render {
namespace {

VkShaderModule makeModule(VkDevice device, const u32* code, size_t words) {
    if (!code || words == 0) return VK_NULL_HANDLE;
    VkShaderModuleCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    ci.codeSize = words * 4;
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

// The world's vertex layout, straight from `MeshVertex` (44 bytes).  A change to
// that struct is a change to this function and to `world.vert`; the static
// asserts in chunk.h are what keep the two honest.
VkVertexInputBindingDescription worldBinding() {
    VkVertexInputBindingDescription b{};
    b.binding = 0;
    b.stride = sizeof(MeshVertex);
    b.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return b;
}

void worldAttributes(VkVertexInputAttributeDescription out[5]) {
    out[0] = {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(MeshVertex, px)};
    out[1] = {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(MeshVertex, nx)};
    out[2] = {2, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(MeshVertex, u)};
    out[3] = {3, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(MeshVertex, u2)};
    // materialId, zoneId, flags and ao arrive as one 4-byte unsigned word; the
    // shader unpacks them, which keeps the attribute count (and the vertex
    // fetch cost on a phone) at five.
    out[4] = {4, 0, VK_FORMAT_R8G8B8A8_UINT, offsetof(MeshVertex, materialId)};
}

struct Vertex3_3_3 {
    f32 px, py, pz;
    f32 nx, ny, nz;
    f32 r, g, b;
};

}  // namespace

// =============================================================================
//  Uniforms and descriptors
// =============================================================================
bool Renderer::Impl::buildUniforms(std::string* err) {
    for (u32 i = 0; i < kFramesInFlight; ++i) {
        if (!createBuffer(sizeof(FrameUbo), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          uniformBuffers[i], uniformMemory[i], &uniformMapped[i])) {
            if (err) *err = "uniform buffer allocation failed";
            return false;
        }
        VkCommandBufferAllocateInfo ai{};
        ai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        ai.commandPool = cmdPool;
        ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        ai.commandBufferCount = 1;
        if (vkAllocateCommandBuffers(device, &ai, &cmd[i]) != VK_SUCCESS) {
            if (err) *err = "command buffer allocation failed";
            return false;
        }
        VkSemaphoreCreateInfo si{};
        si.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        VkFenceCreateInfo fi{};
        fi.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        if (vkCreateSemaphore(device, &si, nullptr, &imageReady[i]) != VK_SUCCESS ||
            vkCreateSemaphore(device, &si, nullptr, &renderDone[i]) != VK_SUCCESS ||
            vkCreateFence(device, &fi, nullptr, &inFlight[i]) != VK_SUCCESS) {
            if (err) *err = "frame synchronisation allocation failed";
            return false;
        }
    }
    return true;
}

bool Renderer::Impl::buildDescriptors(std::string* err) {
    VkDescriptorSetLayoutBinding b[5]{};
    b[0].binding = 0;
    b[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    b[0].descriptorCount = 1;
    b[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    // Binding 4 is the HDR target the tonemap reads; it is written by
    // `writeHdrDescriptor()` once the offscreen image exists, which is *after*
    // this function runs.
    for (u32 i = 1; i < 5; ++i) {
        b[i].binding = i;
        b[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        b[i].descriptorCount = 1;
        b[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    VkDescriptorSetLayoutCreateInfo li{};
    li.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    li.bindingCount = 5;
    li.pBindings = b;
    if (vkCreateDescriptorSetLayout(device, &li, nullptr, &setLayout) != VK_SUCCESS) {
        if (err) *err = "descriptor set layout failed";
        return false;
    }

    VkDescriptorPoolSize sizes[2]{};
    sizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    sizes[0].descriptorCount = kFramesInFlight;
    sizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    sizes[1].descriptorCount = kFramesInFlight * 4;
    VkDescriptorPoolCreateInfo pi{};
    pi.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pi.maxSets = kFramesInFlight;
    pi.poolSizeCount = 2;
    pi.pPoolSizes = sizes;
    if (vkCreateDescriptorPool(device, &pi, nullptr, &descPool) != VK_SUCCESS) {
        if (err) *err = "descriptor pool failed";
        return false;
    }
    VkDescriptorSetLayout layouts[kFramesInFlight];
    for (u32 i = 0; i < kFramesInFlight; ++i) layouts[i] = setLayout;
    VkDescriptorSetAllocateInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    ai.descriptorPool = descPool;
    ai.descriptorSetCount = kFramesInFlight;
    ai.pSetLayouts = layouts;
    if (vkAllocateDescriptorSets(device, &ai, sets) != VK_SUCCESS) {
        if (err) *err = "descriptor set allocation failed";
        return false;
    }

    VkSamplerCreateInfo smp{};
    smp.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    smp.magFilter = VK_FILTER_LINEAR;
    smp.minFilter = VK_FILTER_LINEAR;
    smp.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    smp.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    smp.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    smp.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    smp.maxLod = 16.0f;
    // 4x anisotropy: the single cheapest fix for a road surface turning to mush
    // at a grazing angle, which is most of the view from a motorcycle.
    smp.anisotropyEnable = VK_TRUE;
    smp.maxAnisotropy = 4.0f;
    if (vkCreateSampler(device, &smp, nullptr, &sampler) != VK_SUCCESS) {
        if (err) *err = "sampler creation failed";
        return false;
    }

    if (!sampler) return true;
    for (u32 i = 0; i < kFramesInFlight; ++i) {
        VkDescriptorBufferInfo ub{};
        ub.buffer = uniformBuffers[i];
        ub.offset = 0;
        ub.range = sizeof(FrameUbo);
        VkDescriptorImageInfo ii[3]{};
        const VkImageView views[3] = {albedoView, normalView, ormView};
        for (u32 k = 0; k < 3; ++k) {
            ii[k].sampler = sampler;
            ii[k].imageView = views[k];
            ii[k].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        }
        VkWriteDescriptorSet w[4]{};
        for (u32 k = 0; k < 4; ++k) {
            w[k].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w[k].dstSet = sets[i];
            w[k].dstBinding = k;
            w[k].descriptorCount = 1;
        }
        w[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        w[0].pBufferInfo = &ub;
        for (u32 k = 0; k < 3; ++k) {
            w[k + 1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            w[k + 1].pImageInfo = &ii[k];
        }
        vkUpdateDescriptorSets(device, 4, w, 0, nullptr);
    }
    return true;
}

void Renderer::Impl::writeHdrDescriptor() {
    if (!hdrView || !sampler) return;
    for (u32 i = 0; i < kFramesInFlight; ++i) {
        VkDescriptorImageInfo ii{};
        ii.sampler = sampler;
        ii.imageView = hdrView;
        ii.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkWriteDescriptorSet w{};
        w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w.dstSet = sets[i];
        w.dstBinding = 4;
        w.descriptorCount = 1;
        w.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w.pImageInfo = &ii;
        vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
    }
}

// =============================================================================
//  Pipelines
// =============================================================================
bool Renderer::Impl::buildPipelines(std::string* err) {
    auto need = [&](VkShaderModule m, const char* what) {
        if (m != VK_NULL_HANDLE) return true;
        if (err) *err = std::string("shader missing: ") + what +
                        " (SPIR-V is generated by glslc at build time)";
        return false;
    };

    VkShaderModule worldVert = makeModule(device, kWorldVertSpv, kWorldVertSpvWords);
    VkShaderModule worldFrag = makeModule(device, kWorldFragSpv, kWorldFragSpvWords);
    VkShaderModule waterFrag = makeModule(device, kWaterFragSpv, kWaterFragSpvWords);
    VkShaderModule skyVert = makeModule(device, kSkyVertSpv, kSkyVertSpvWords);
    VkShaderModule skyFrag = makeModule(device, kSkyFragSpv, kSkyFragSpvWords);
    VkShaderModule postVert = makeModule(device, kPostVertSpv, kPostVertSpvWords);
    VkShaderModule postFrag = makeModule(device, kPostFragSpv, kPostFragSpvWords);
    VkShaderModule trafficVert = makeModule(device, kTrafficVertSpv, kTrafficVertSpvWords);
    VkShaderModule trafficFrag = makeModule(device, kTrafficFragSpv, kTrafficFragSpvWords);

    if (!need(worldVert, "world.vert") || !need(worldFrag, "world.frag") ||
        !need(waterFrag, "water.frag") || !need(skyVert, "sky.vert") ||
        !need(skyFrag, "sky.frag") || !need(postVert, "post.vert") ||
        !need(postFrag, "post.frag") || !need(trafficVert, "traffic.vert") ||
        !need(trafficFrag, "traffic.frag"))
        return false;

    // --- layouts -------------------------------------------------------------
    {
        VkPushConstantRange pc{};
        pc.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        pc.offset = 0;
        pc.size = sizeof(PushConstants);
        VkPipelineLayoutCreateInfo li{};
        li.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        li.setLayoutCount = 1;
        li.pSetLayouts = &setLayout;
        li.pushConstantRangeCount = 1;
        li.pPushConstantRanges = &pc;
        if (vkCreatePipelineLayout(device, &li, nullptr, &worldLayout) != VK_SUCCESS) {
            if (err) *err = "world pipeline layout failed";
            return false;
        }
        // The tonemap reads the same uniform block (exposure, resolution) and the
        // HDR colour attachment, so it shares the set layout rather than
        // duplicating it.
        li.pushConstantRangeCount = 1;
        if (vkCreatePipelineLayout(device, &li, nullptr, &postLayout) != VK_SUCCESS) {
            if (err) *err = "post pipeline layout failed";
            return false;
        }
    }

    const VkVertexInputBindingDescription wb = worldBinding();
    VkVertexInputAttributeDescription wa[5];
    worldAttributes(wa);
    VkPipelineVertexInputStateCreateInfo viWorld{};
    viWorld.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    viWorld.vertexBindingDescriptionCount = 1;
    viWorld.pVertexBindingDescriptions = &wb;
    viWorld.vertexAttributeDescriptionCount = 5;
    viWorld.pVertexAttributeDescriptions = wa;

    VkPipelineVertexInputStateCreateInfo viEmpty{};
    viEmpty.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

    VkPipelineInputAssemblyStateCreateInfo ia{};
    ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp{};
    vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vp.viewportCount = 1;
    vp.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rs{};
    rs.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = VK_CULL_MODE_NONE;       // see the header comment
    rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth = 1.0f;
    rs.depthClampEnable = VK_FALSE;
    rs.rasterizerDiscardEnable = VK_FALSE;

    VkPipelineMultisampleStateCreateInfo ms{};
    ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState blendOff{};
    blendOff.colorWriteMask = 0xF;
    blendOff.blendEnable = VK_FALSE;

    VkPipelineColorBlendStateCreateInfo cb{};
    cb.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    cb.attachmentCount = 1;
    cb.pAttachments = &blendOff;

    // Two dynamic states, and only two: the viewport follows the render scale and
    // the scissor follows the swapchain, so neither is baked into a pipeline that
    // would then have to be rebuilt on every resolution change.
    const VkDynamicState dyn[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo ds{};
    ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    ds.dynamicStateCount = 2;
    ds.pDynamicStates = dyn;

    auto build = [&](VkPipelineLayout layout, const VkPipelineShaderStageCreateInfo* stages,
                     u32 stageCount, const VkPipelineVertexInputStateCreateInfo& vin,
                     const VkPipelineDepthStencilStateCreateInfo& dss,
                     const VkPipelineColorBlendStateCreateInfo& cbs, VkRenderPass pass,
                     VkPipeline& out) {
        VkGraphicsPipelineCreateInfo ci{};
        ci.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        ci.stageCount = stageCount;
        ci.pStages = stages;
        ci.pVertexInputState = &vin;
        ci.pInputAssemblyState = &ia;
        ci.pViewportState = &vp;
        ci.pRasterizationState = &rs;
        ci.pMultisampleState = &ms;
        ci.pDepthStencilState = &dss;
        ci.pColorBlendState = &cbs;
        ci.pDynamicState = &ds;
        ci.layout = layout;
        ci.renderPass = pass;
        ci.subpass = 0;
        return vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &ci, nullptr, &out) == VK_SUCCESS;
    };

    // --- depth prepass -------------------------------------------------------
    {
        VkPipelineShaderStageCreateInfo stages[1] = {stage(VK_SHADER_STAGE_VERTEX_BIT, worldVert)};
        VkPipelineDepthStencilStateCreateInfo dss{};
        dss.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        dss.depthTestEnable = VK_TRUE;
        dss.depthWriteEnable = VK_TRUE;
        dss.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        VkPipelineColorBlendStateCreateInfo none = cb;
        none.attachmentCount = 0;
        none.pAttachments = nullptr;
        if (!build(worldLayout, stages, 1, viWorld, dss, none, prepass, depthPipeline)) {
            if (err) *err = "depth prepass pipeline failed";
            return false;
        }
    }

    // --- forward -------------------------------------------------------------
    {
        VkPipelineShaderStageCreateInfo stages[2] = {stage(VK_SHADER_STAGE_VERTEX_BIT, worldVert),
                                                     stage(VK_SHADER_STAGE_FRAGMENT_BIT, worldFrag)};
        VkPipelineDepthStencilStateCreateInfo dss{};
        dss.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        dss.depthTestEnable = VK_TRUE;
        dss.depthWriteEnable = VK_TRUE;
        dss.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        if (!build(worldLayout, stages, 2, viWorld, dss, cb, forwardPass, opaquePipeline)) {
            if (err) *err = "forward pipeline failed";
            return false;
        }

        // Water: same geometry, its own shading, blended and writing no depth.
        VkPipelineShaderStageCreateInfo wstages[2] = {stage(VK_SHADER_STAGE_VERTEX_BIT, worldVert),
                                                      stage(VK_SHADER_STAGE_FRAGMENT_BIT, waterFrag)};
        VkPipelineDepthStencilStateCreateInfo wdss = dss;
        wdss.depthWriteEnable = VK_FALSE;
        VkPipelineColorBlendAttachmentState wblend{};
        wblend.blendEnable = VK_TRUE;
        wblend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        wblend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        wblend.colorBlendOp = VK_BLEND_OP_ADD;
        wblend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        wblend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        wblend.alphaBlendOp = VK_BLEND_OP_ADD;
        wblend.colorWriteMask = 0xF;
        VkPipelineColorBlendStateCreateInfo wcb = cb;
        wcb.pAttachments = &wblend;
        if (!build(worldLayout, wstages, 2, viWorld, wdss, wcb, forwardPass, waterPipeline)) {
            if (err) *err = "water pipeline failed";
            return false;
        }

        // Sky: a full-screen triangle, no vertex buffer, drawn where depth is
        // still the cleared value.
        VkPipelineShaderStageCreateInfo sstages[2] = {stage(VK_SHADER_STAGE_VERTEX_BIT, skyVert),
                                                      stage(VK_SHADER_STAGE_FRAGMENT_BIT, skyFrag)};
        VkPipelineDepthStencilStateCreateInfo sdss{};
        sdss.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        sdss.depthTestEnable = VK_TRUE;
        sdss.depthWriteEnable = VK_FALSE;
        sdss.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        if (!build(worldLayout, sstages, 2, viEmpty, sdss, cb, forwardPass, skyPipeline)) {
            if (err) *err = "sky pipeline failed";
            return false;
        }
    }

    // --- traffic (instanced) -------------------------------------------------
    {
        VkVertexInputBindingDescription tb[2]{};
        tb[0].binding = 0;
        tb[0].stride = sizeof(Vertex3_3_3);
        tb[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        tb[1].binding = 1;
        tb[1].stride = sizeof(InstanceVertex);
        tb[1].inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;
        VkVertexInputAttributeDescription ta[6]{};
        ta[0] = {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex3_3_3, px)};
        ta[1] = {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex3_3_3, nx)};
        ta[2] = {2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex3_3_3, r)};
        ta[3] = {3, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceVertex, pos)};
        ta[4] = {4, 1, VK_FORMAT_R32G32_SFLOAT, offsetof(InstanceVertex, yaw)};
        ta[5] = {5, 1, VK_FORMAT_R32G32B32_SFLOAT,
                 static_cast<u32>(offsetof(InstanceVertex, colour))};
        VkPipelineVertexInputStateCreateInfo tvi{};
        tvi.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        tvi.vertexBindingDescriptionCount = 2;
        tvi.pVertexBindingDescriptions = tb;
        tvi.vertexAttributeDescriptionCount = 6;
        tvi.pVertexAttributeDescriptions = ta;

        VkPipelineShaderStageCreateInfo stages[2] = {stage(VK_SHADER_STAGE_VERTEX_BIT, trafficVert),
                                                     stage(VK_SHADER_STAGE_FRAGMENT_BIT, trafficFrag)};
        VkPipelineDepthStencilStateCreateInfo dss{};
        dss.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        dss.depthTestEnable = VK_TRUE;
        dss.depthWriteEnable = VK_TRUE;
        dss.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        if (!build(worldLayout, stages, 2, tvi, dss, cb, forwardPass, trafficPipeline)) {
            if (err) *err = "traffic pipeline failed";
            return false;
        }
        VkPipelineShaderStageCreateInfo dstages[1] = {stage(VK_SHADER_STAGE_VERTEX_BIT, trafficVert)};
        VkPipelineColorBlendStateCreateInfo none = cb;
        none.attachmentCount = 0;
        none.pAttachments = nullptr;
        if (!build(worldLayout, dstages, 1, tvi, dss, none, prepass, trafficDepthPipeline)) {
            if (err) *err = "traffic depth pipeline failed";
            return false;
        }
    }

    // --- tonemap -------------------------------------------------------------
    {
        VkPipelineShaderStageCreateInfo stages[2] = {stage(VK_SHADER_STAGE_VERTEX_BIT, postVert),
                                                     stage(VK_SHADER_STAGE_FRAGMENT_BIT, postFrag)};
        VkPipelineDepthStencilStateCreateInfo dss{};
        dss.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        dss.depthTestEnable = VK_FALSE;
        dss.depthWriteEnable = VK_FALSE;
        dss.depthCompareOp = VK_COMPARE_OP_ALWAYS;
        if (!build(postLayout, stages, 2, viEmpty, dss, cb, postPass, tonemapPipeline)) {
            if (err) *err = "tonemap pipeline failed";
            return false;
        }
    }

    vkDestroyShaderModule(device, worldVert, nullptr);
    vkDestroyShaderModule(device, worldFrag, nullptr);
    vkDestroyShaderModule(device, waterFrag, nullptr);
    vkDestroyShaderModule(device, skyVert, nullptr);
    vkDestroyShaderModule(device, skyFrag, nullptr);
    vkDestroyShaderModule(device, postVert, nullptr);
    vkDestroyShaderModule(device, postFrag, nullptr);
    vkDestroyShaderModule(device, trafficVert, nullptr);
    vkDestroyShaderModule(device, trafficFrag, nullptr);
    return true;
}

}  // namespace mumbai::render
