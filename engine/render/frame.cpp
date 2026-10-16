// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/frame.cpp
//
//  The frame: uniform update, command recording, submission, present.
//
//  THE FLOATING ORIGIN, CONCRETELY
//  -------------------------------
//  `drawFrame` receives a camera in *render-space* metres.  The app sets
//  `renderOrigin` to the camera's macro-cell origin (a multiple of 1024 m), so
//  the camera's own coordinates never leave the range [0, 1024).  A chunk's
//  vertices are cell-local metres, so a chunk needs one number to land in that
//  space: its macro-cell origin minus the render origin, which is at most a few
//  kilometres and therefore exactly representable in f32 to a fraction of a
//  millimetre.  When the camera crosses a cell boundary the app changes
//  `renderOrigin` by a whole number of cells, every chunk's delta changes by the
//  same whole number of metres, and every vertex lands in the same place.  There
//  is no rebase, no rebuild and no pop — and the audio layer never sees any of
//  it because it works in f64 world metres (D-042).
// =============================================================================
#include "engine/render/renderer_impl.h"

#include <cstring>

namespace mumbai::render {

// =============================================================================
//  Recording
// =============================================================================
void Renderer::Impl::recordPrepass(VkCommandBuffer cb) {
    VkClearValue clear{};
    clear.depthStencil = {1.0f, 0};
    VkRenderPassBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    bi.renderPass = prepass;
    bi.framebuffer = prepassFb;
    bi.renderArea = {{0, 0}, renderExtent};
    bi.clearValueCount = 1;
    bi.pClearValues = &clear;
    vkCmdBeginRenderPass(cb, &bi, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport vp{0.0f, 0.0f, static_cast<f32>(renderExtent.width),
                  static_cast<f32>(renderExtent.height), 0.0f, 1.0f};
    VkRect2D sc{{0, 0}, renderExtent};
    vkCmdSetViewport(cb, 0, 1, &vp);
    vkCmdSetScissor(cb, 0, 1, &sc);

    VkDeviceSize zero = 0;
    if (depthPipeline != VK_NULL_HANDLE && !chunks.empty()) {
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, depthPipeline);
        vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, worldLayout, 0, 1,
                               &sets[frame], 0, nullptr);
        const VkBuffer vb = vertexArena.buffer();
        vkCmdBindVertexBuffers(cb, 0, 1, &vb, &zero);
        vkCmdBindIndexBuffer(cb, indexArena.buffer(), 0, VK_INDEX_TYPE_UINT32);
    }
    for (const auto& entry : chunks) {
        const ChunkGpu& gpu = *entry.second;
        PushConstants pc{};
        pc.originDelta[0] = gpu.originDelta.x;
        pc.originDelta[1] = gpu.originDelta.y;
        pc.originDelta[2] = gpu.originDelta.z;
        pc.originDelta[3] = 0.0f;
        vkCmdPushConstants(cb, worldLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(PushConstants), &pc);
        for (const ChunkGpu::Draw& d : gpu.draws) {
            vkCmdDrawIndexed(cb, d.indexCount, 1, d.firstIndex, 0, 0);
        }
    }

    // Traffic, in the same prepass: vehicles are opaque and occlude each other.
    if (trafficDepthPipeline != VK_NULL_HANDLE && trafficInstanceCount > 0) {
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, trafficDepthPipeline);
        const VkBuffer bufs[2] = {trafficBuffer, trafficBuffer};
        const VkDeviceSize offs[2] = {0, trafficInstanceOffset};
        vkCmdBindVertexBuffers(cb, 0, 2, bufs, offs);
        PushConstants pc{};
        vkCmdPushConstants(cb, worldLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
        for (u32 m = 0; m < trafficMeshCount; ++m) {
            const MeshRange& r = trafficMeshes[m];
            vkCmdDraw(cb, r.count, trafficInstanceCount, r.first, r.kind);
        }
    }
    vkCmdEndRenderPass(cb);
}

void Renderer::Impl::recordForward(VkCommandBuffer cb) {
    VkClearValue clears[2]{};
    clears[0].color = {{0.05f, 0.06f, 0.08f, 1.0f}};
    clears[1].depthStencil = {1.0f, 0};
    VkRenderPassBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    bi.renderPass = forwardPass;
    bi.framebuffer = forwardFb;
    bi.renderArea = {{0, 0}, renderExtent};
    bi.clearValueCount = 2;
    bi.pClearValues = clears;
    vkCmdBeginRenderPass(cb, &bi, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport vp{0.0f, 0.0f, static_cast<f32>(renderExtent.width),
                  static_cast<f32>(renderExtent.height), 0.0f, 1.0f};
    VkRect2D sc{{0, 0}, renderExtent};
    vkCmdSetViewport(cb, 0, 1, &vp);
    vkCmdSetScissor(cb, 0, 1, &sc);

    PushConstants pc{};

    // --- sky: one triangle, only where nothing was drawn ---------------------
    if (skyPipeline != VK_NULL_HANDLE) {
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipeline);
        vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, worldLayout, 0, 1,
                                &sets[frame], 0, nullptr);
        vkCmdPushConstants(cb, worldLayout, VK_SHADER_STAGE_VERTEX_BIT |
                           VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);
        vkCmdDraw(cb, 3, 1, 0, 0);
    }

    // --- world geometry ------------------------------------------------------
    const bool haveWorld = opaquePipeline != VK_NULL_HANDLE && !chunks.empty();
    if (haveWorld) {
        VkDeviceSize zero = 0;
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, opaquePipeline);
        vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, worldLayout, 0, 1,
                                &sets[frame], 0, nullptr);
        const VkBuffer vb = vertexArena.buffer();
        vkCmdBindVertexBuffers(cb, 0, 1, &vb, &zero);
        vkCmdBindIndexBuffer(cb, indexArena.buffer(), 0, VK_INDEX_TYPE_UINT32);
        for (const auto& entry : chunks) {
            const ChunkGpu& gpu = *entry.second;
            PushConstants wpc{};
            wpc.originDelta[0] = gpu.originDelta.x;
            wpc.originDelta[1] = gpu.originDelta.y;
            wpc.originDelta[2] = gpu.originDelta.z;
            vkCmdPushConstants(cb, worldLayout, VK_SHADER_STAGE_VERTEX_BIT |
                               VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(wpc), &wpc);
            for (const ChunkGpu::Draw& d : gpu.draws) {
                if (d.kind == kDrawWater) continue;      // water is its own pass
                vkCmdDrawIndexed(cb, d.indexCount, 1, d.firstIndex, 0, 0);
            }
        }
    }

    // --- traffic -------------------------------------------------------------
    if (trafficPipeline != VK_NULL_HANDLE && trafficInstanceCount > 0) {
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, trafficPipeline);
        const VkBuffer bufs[2] = {trafficBuffer, trafficBuffer};
        const VkDeviceSize offs[2] = {0, trafficInstanceOffset};
        vkCmdBindVertexBuffers(cb, 0, 2, bufs, offs);
        vkCmdPushConstants(cb, worldLayout, VK_SHADER_STAGE_VERTEX_BIT |
                           VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);
        for (u32 m = 0; m < trafficMeshCount; ++m) {
            const MeshRange& r = trafficMeshes[m];
            vkCmdDraw(cb, r.count, trafficInstanceCount, r.first, 0);
        }
    }

    // --- water last: it blends against everything already shaded -------------
    if (waterPipeline != VK_NULL_HANDLE && !chunks.empty()) {
        VkDeviceSize zero = 0;
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, waterPipeline);
        const VkBuffer vb = vertexArena.buffer();
        vkCmdBindVertexBuffers(cb, 0, 1, &vb, &zero);
        vkCmdBindIndexBuffer(cb, indexArena.buffer(), 0, VK_INDEX_TYPE_UINT32);
        for (const auto& entry : chunks) {
            const ChunkGpu& gpu = *entry.second;
            PushConstants wpc{};
            wpc.originDelta[0] = gpu.originDelta.x;
            wpc.originDelta[1] = gpu.originDelta.y;
            wpc.originDelta[2] = gpu.originDelta.z;
            vkCmdPushConstants(cb, worldLayout, VK_SHADER_STAGE_VERTEX_BIT |
                               VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(wpc), &wpc);
            for (const ChunkGpu::Draw& d : gpu.draws) {
                if (d.kind != kDrawWater) continue;
                vkCmdDrawIndexed(cb, d.indexCount, 1, d.firstIndex, 0, 0);
            }
        }
    }

    frameDraws = 0;
    frameTriangles = 0;
    for (const auto& entry : chunks) {
        for (const ChunkGpu::Draw& d : entry.second->draws) {
            ++frameDraws;
            frameTriangles += d.indexCount / 3;
        }
    }
    frameDraws += trafficInstanceCount > 0 ? trafficMeshCount : 0;
    vkCmdEndRenderPass(cb);
}

void Renderer::Impl::recordPost(VkCommandBuffer cb, VkFramebuffer target) {
    VkClearValue clear{};
    clear.color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    VkRenderPassBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    bi.renderPass = postPass;
    bi.framebuffer = target;
    bi.renderArea = {{0, 0}, swapExtent};
    bi.clearValueCount = 1;
    bi.pClearValues = &clear;
    vkCmdBeginRenderPass(cb, &bi, VK_SUBPASS_CONTENTS_INLINE);
    VkViewport vp{0.0f, 0.0f, static_cast<f32>(swapExtent.width),
                  static_cast<f32>(swapExtent.height), 0.0f, 1.0f};
    VkRect2D sc{{0, 0}, swapExtent};
    vkCmdSetViewport(cb, 0, 1, &vp);
    vkCmdSetScissor(cb, 0, 1, &sc);
    if (tonemapPipeline != VK_NULL_HANDLE) {
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, tonemapPipeline);
        vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, postLayout, 0, 1,
                                &sets[frame], 0, nullptr);
        vkCmdDraw(cb, 3, 1, 0, 0);
    }
    // The panel goes on last, in the same pass, after the tonemap.  It is drawn
    // in swapchain pixels and is deliberately outside the exposure: text that dims
    // as the sun sets is text that cannot be read exactly when it is needed.
    if (hudPipeline != VK_NULL_HANDLE) {
        uploadHudVertices();
        if (hudVertexCount > 0) {
            HudPush push{};
            push.viewportW = static_cast<f32>(swapExtent.width);
            push.viewportH = static_cast<f32>(swapExtent.height);
            vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, hudPipeline);
            vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, hudLayout, 0, 1, &hudSet, 0,
                                    nullptr);
            vkCmdPushConstants(cb, hudLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(push), &push);
            const VkDeviceSize zero = 0;
            vkCmdBindVertexBuffers(cb, 0, 1, &hudBuffer, &zero);
            vkCmdDraw(cb, hudVertexCount, 1, 0, 0);
            ++frameDraws;
        }
    }
    vkCmdEndRenderPass(cb);
}

// =============================================================================
//  drawFrame
// =============================================================================
bool Renderer::drawFrame(std::string* error) {
    Impl& d = *m_impl;
    if (!d.device) {
        if (error) *error = "renderer is not initialised";
        return false;
    }
    if (d.surface == VK_NULL_HANDLE) {
        if (error) *error = "no window surface (host build)";
        return false;
    }
    if (d.swapchainDirty) {
        std::string err;
        if (!d.buildSwapchain(d.swapExtent.width, d.swapExtent.height, &err)) {
            if (error) *error = err;
            return false;
        }
        if (!d.buildOffscreen(&err)) {
            if (error) *error = err;
            return false;
        }
        d.swapchainDirty = false;
    }

    vkWaitForFences(d.device, 1, &d.inFlight[d.frame], VK_TRUE, UINT64_MAX);

    u32 imageIndex = 0;
    VkResult r = vkAcquireNextImageKHR(d.device, d.swapchain, UINT64_MAX,
                                       d.imageReady[d.frame], VK_NULL_HANDLE, &imageIndex);
    if (r == VK_ERROR_OUT_OF_DATE_KHR) {
        // Not a failure: the surface changed.  Rebuild and report, and the caller
        // simply draws the next frame.
        d.swapchainDirty = true;
        if (error) *error = "swapchain out of date";
        return false;
    }
    if (r != VK_SUCCESS && r != VK_SUBOPTIMAL_KHR) {
        if (error) *error = std::string("vkAcquireNextImageKHR -> ") + std::to_string(r);
        return false;
    }
    vkResetFences(d.device, 1, &d.inFlight[d.frame]);

    // --- uniforms ------------------------------------------------------------
    {
        const f32 aspect = static_cast<f32>(d.renderExtent.width) /
                           static_cast<f32>(std::max(1u, d.renderExtent.height));
        const Mat4 view = m_camera.view();
        const Mat4 proj = m_camera.proj(aspect);
        const Mat4 viewProj = proj * view;
        FrameUbo& u = d.ubo;
        std::memcpy(u.viewProj, viewProj.m, sizeof(u.viewProj));
        u.cameraPos[0] = m_camera.pos.x;
        u.cameraPos[1] = m_camera.pos.y;
        u.cameraPos[2] = m_camera.pos.z;
        u.cameraPos[3] = m_environment.time;
        u.sunDir[0] = m_environment.sun.direction.x;
        u.sunDir[1] = m_environment.sun.direction.y;
        u.sunDir[2] = m_environment.sun.direction.z;
        u.sunDir[3] = m_environment.sun.intensity;
        u.sunColour[0] = m_environment.sun.colour.x;
        u.sunColour[1] = m_environment.sun.colour.y;
        u.sunColour[2] = m_environment.sun.colour.z;
        u.sunColour[3] = m_environment.sun.ambient;
        u.fog[0] = m_environment.fogColour.x;
        u.fog[1] = m_environment.fogColour.y;
        u.fog[2] = m_environment.fogColour.z;
        u.fog[3] = m_environment.fogDensity;
        u.envParams[0] = m_environment.humidity;
        u.envParams[1] = m_environment.pollution;
        u.envParams[2] = m_environment.exposure;
        u.envParams[3] = m_environment.wetness;
        u.skyParams[0] = static_cast<f32>(d.renderExtent.width);
        u.skyParams[1] = static_cast<f32>(d.renderExtent.height);
        u.skyParams[2] = m_environment.time;
        u.skyParams[3] = 0.0f;
        std::memcpy(d.uniformMapped[d.frame], &u, sizeof(u));
    }

    // --- record --------------------------------------------------------------
    vkResetCommandBuffer(d.cmd[d.frame], 0);
    VkCommandBufferBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(d.cmd[d.frame], &bi);
    d.recordPrepass(d.cmd[d.frame]);
    d.recordForward(d.cmd[d.frame]);
    d.recordPost(d.cmd[d.frame], d.swapFramebuffers[imageIndex]);
    vkEndCommandBuffer(d.cmd[d.frame]);

    // --- submit --------------------------------------------------------------
    {
        const VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkSubmitInfo si{};
        si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        si.waitSemaphoreCount = 1;
        si.pWaitSemaphores = &d.imageReady[d.frame];
        si.pWaitDstStageMask = &waitStage;
        si.commandBufferCount = 1;
        si.pCommandBuffers = &d.cmd[d.frame];
        si.signalSemaphoreCount = 1;
        si.pSignalSemaphores = &d.renderDone[d.frame];
        r = vkQueueSubmit(d.queue, 1, &si, d.inFlight[d.frame]);
        if (r != VK_SUCCESS) {
            if (error) *error = "vkQueueSubmit -> " + std::to_string(r);
            return false;
        }
    }
    {
        VkPresentInfoKHR pi{};
        pi.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
        pi.waitSemaphoreCount = 1;
        pi.pWaitSemaphores = &d.renderDone[d.frame];
        pi.swapchainCount = 1;
        pi.pSwapchains = &d.swapchain;
        pi.pImageIndices = &imageIndex;
        r = vkQueuePresentKHR(d.queue, &pi);
        if (r == VK_ERROR_OUT_OF_DATE_KHR || r == VK_SUBOPTIMAL_KHR) {
            d.swapchainDirty = true;
        } else if (r != VK_SUCCESS) {
            if (error) *error = "vkQueuePresentKHR -> " + std::to_string(r);
            return false;
        }
    }

    d.frame = (d.frame + 1) % kFramesInFlight;
    m_stats.drawCalls = d.frameDraws;
    m_stats.triangles = d.frameTriangles;
    m_stats.instances = d.trafficInstanceCount;
    m_stats.renderWidth = d.renderExtent.width;
    m_stats.renderHeight = d.renderExtent.height;
    m_stats.chunksResident = static_cast<u32>(d.chunks.size());
    m_stats.gpuBytes = d.vertexArena.used() + d.indexArena.used();
    return true;
}

}  // namespace mumbai::render
