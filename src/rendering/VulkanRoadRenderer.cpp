#include "VulkanRoadRenderer.h"
#include "core/Logging.h"
#include "vehicle/MultiBodyParams.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <string>

static const std::string SHADER_DIR_STR = SHADER_DIR;

// Sky pipeline: full-screen triangle, no vertex buffer, no depth test/write.
static const PipelineConfig SKY_PIPELINE_CONFIG {
    .hasVertexInput = false,
    .depthTest      = false,
    .depthWrite     = false,
    .cullMode       = VK_CULL_MODE_NONE
};

// Vehicle opaque pipeline: wheels, vehicle vertex layout.
static const PipelineConfig VEHICLE_OPAQUE_CONFIG {
    .hasVertexInput = true,
    .depthTest      = true,
    .depthWrite     = true,
    .blendEnable    = false,
    .cullMode       = VK_CULL_MODE_NONE,   // camera X-fix changes winding; NONE for debug mesh
    .vertexLayout   = VertexLayout::Vehicle
};

// Vehicle translucent pipeline: body box, alpha blend, no depth write.
static const PipelineConfig VEHICLE_TRANSLUCENT_CONFIG {
    .hasVertexInput = true,
    .depthTest      = true,
    .depthWrite     = false,
    .blendEnable    = true,
    .cullMode       = VK_CULL_MODE_NONE,
    .vertexLayout   = VertexLayout::Vehicle
};

// HUD pipeline: screen-space quads, no depth test, alpha blend.
static const PipelineConfig HUD_PIPELINE_CONFIG {
    .hasVertexInput = true,
    .depthTest      = false,
    .depthWrite     = false,
    .blendEnable    = true,
    .cullMode       = VK_CULL_MODE_NONE,
    .vertexLayout   = VertexLayout::Vehicle
};

bool VulkanRoadRenderer::initialize(SDL_Window* window)
{
    window_ = window;
    if (!ctx_.initialize(window)) return false;

    if (!skyPipeline_.create(ctx_,
            SHADER_DIR_STR + "sky.vert.spv",
            SHADER_DIR_STR + "sky.frag.spv",
            SKY_PIPELINE_CONFIG))
        return false;

    // Road pipeline: slight depth bias to prevent Z-fighting between coplanar
    // surfaces (cliff face vs ground, dirt vs tarmac at transitions).
    PipelineConfig roadConfig {};
    roadConfig.depthBiasEnable  = true;
    roadConfig.depthBiasConstant = 1.0f;    // small constant offset
    roadConfig.depthBiasSlope    = 1.0f;    // slope-scaled offset
    if (!roadPipeline_.create(ctx_,
            SHADER_DIR_STR + "road.vert.spv",
            SHADER_DIR_STR + "road.frag.spv",
            roadConfig))
        return false;

    createVehiclePipelines();
    uploadVehicleMeshes();

    LOG_INFO("VulkanRoadRenderer initialized");
    return true;
}

void VulkanRoadRenderer::createVehiclePipelines()
{
    vehicleOpaquePipeline_.create(ctx_,
        SHADER_DIR_STR + "vehicle.vert.spv",
        SHADER_DIR_STR + "vehicle.frag.spv",
        VEHICLE_OPAQUE_CONFIG);

    vehicleTranslucentPipeline_.create(ctx_,
        SHADER_DIR_STR + "vehicle.vert.spv",
        SHADER_DIR_STR + "vehicle.frag.spv",
        VEHICLE_TRANSLUCENT_CONFIG);

    hudPipeline_.create(ctx_,
        SHADER_DIR_STR + "vehicle.vert.spv",
        SHADER_DIR_STR + "vehicle.frag.spv",
        HUD_PIPELINE_CONFIG);
}

void VulkanRoadRenderer::uploadVehicleMeshes()
{
    // BRZ-proportioned dimensions (in body frame, CG at origin)
    static constexpr glm::vec4 bodyColor  { 0.5f, 0.75f, 1.0f, 0.25f };  // translucent light blue
    static constexpr glm::vec4 wheelColor { 0.12f, 0.12f, 0.12f, 1.0f }; // dark rubber

    MultiBodyParams p{};

    VehicleMesh body = vehicle_geom::makeBox(
        p.halfTrackFrontM,          // half-width (~0.77m)
        -(p.cgHeightM - p.wheelRadiusM),  // y_bot below CG
         0.70f,                     // y_top above CG (approximate roof)
        p.lrDistM,                  // half-length rear
        p.lfDistM,                  // half-length front
        bodyColor);

    VehicleMesh wheel = vehicle_geom::makeCylinder(
        p.wheelRadiusM,             // ~0.317m
        0.11f,                      // half tyre width
        16,
        wheelColor);

    // Upload body mesh
    bodyVertBuf_.upload(ctx_,
        body.vertices.data(),
        body.vertices.size() * sizeof(VehicleVertex),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    bodyIdxBuf_.upload(ctx_,
        body.indices.data(),
        body.indices.size() * sizeof(uint32_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    bodyIndexCount_ = static_cast<uint32_t>(body.indices.size());

    // Upload wheel mesh (shared by all 4 wheels, different transforms)
    wheelVertBuf_.upload(ctx_,
        wheel.vertices.data(),
        wheel.vertices.size() * sizeof(VehicleVertex),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    wheelIdxBuf_.upload(ctx_,
        wheel.indices.data(),
        wheel.indices.size() * sizeof(uint32_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    wheelIndexCount_ = static_cast<uint32_t>(wheel.indices.size());

    // --- Suspension components ---
    static constexpr glm::vec4 armColor   { 0.4f, 0.4f, 0.45f, 1.0f };  // dark steel
    static constexpr glm::vec4 strutColor { 0.85f, 0.65f, 0.05f, 1.0f }; // gold/yellow spring

    // Lower control arm: thin beam, unit half-length (scaled at draw time)
    VehicleMesh arm = vehicle_geom::makeBeam(0.5f, 0.015f, 0.008f, armColor);
    armVertBuf_.upload(ctx_,
        arm.vertices.data(), arm.vertices.size() * sizeof(VehicleVertex),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    armIdxBuf_.upload(ctx_,
        arm.indices.data(), arm.indices.size() * sizeof(uint32_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    armIndexCount_ = static_cast<uint32_t>(arm.indices.size());

    // Vertical strut: thin cylinder, unit half-length (scaled at draw time)
    VehicleMesh strut = vehicle_geom::makeStrut(0.018f, 0.5f, 8, strutColor);
    strutVertBuf_.upload(ctx_,
        strut.vertices.data(), strut.vertices.size() * sizeof(VehicleVertex),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    strutIdxBuf_.upload(ctx_,
        strut.indices.data(), strut.indices.size() * sizeof(uint32_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    strutIndexCount_ = static_cast<uint32_t>(strut.indices.size());
}

void VulkanRoadRenderer::recreatePipelines()
{
    skyPipeline_.destroy(ctx_.device);
    skyPipeline_.create(ctx_,
        SHADER_DIR_STR + "sky.vert.spv",
        SHADER_DIR_STR + "sky.frag.spv",
        SKY_PIPELINE_CONFIG);

    roadPipeline_.destroy(ctx_.device);
    roadPipeline_.create(ctx_,
        SHADER_DIR_STR + "road.vert.spv",
        SHADER_DIR_STR + "road.frag.spv");

    vehicleOpaquePipeline_.destroy(ctx_.device);
    vehicleTranslucentPipeline_.destroy(ctx_.device);
    hudPipeline_.destroy(ctx_.device);
    createVehiclePipelines();
}

void VulkanRoadRenderer::uploadMesh(const RoadMesh& mesh)
{
    if (mesh.vertices.empty() || mesh.indices.empty()) return;

    vertexBuffer_.upload(ctx_,
        mesh.vertices.data(),
        mesh.vertices.size() * sizeof(RoadVertex),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);

    indexBuffer_.upload(ctx_,
        mesh.indices.data(),
        mesh.indices.size() * sizeof(uint32_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT);

    indexCount_ = static_cast<uint32_t>(mesh.indices.size());

    // Upload tree geometry (batched into one VehicleMesh)
    if (!mesh.trees.empty()) {
        std::vector<glm::vec3> treePos;
        std::vector<float>     treeH, treeR;
        treePos.reserve(mesh.trees.size());
        treeH.reserve(mesh.trees.size());
        treeR.reserve(mesh.trees.size());
        for (const auto& t : mesh.trees) {
            treePos.push_back(t.position);
            treeH.push_back(t.height);
            treeR.push_back(t.trunkRadius);
        }
        VehicleMesh treeMesh = vehicle_geom::makeTreeBatch(treePos, treeH, treeR);
        treeVertBuf_.upload(ctx_,
            treeMesh.vertices.data(),
            treeMesh.vertices.size() * sizeof(VehicleVertex),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        treeIdxBuf_.upload(ctx_,
            treeMesh.indices.data(),
            treeMesh.indices.size() * sizeof(uint32_t),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
        treeIndexCount_ = static_cast<uint32_t>(treeMesh.indices.size());
        LOG_INFO("Trees uploaded: %zu instances, %u triangles",
                 mesh.trees.size(), treeIndexCount_ / 3);
    }

    LOG_INFO("Road mesh uploaded: %zu verts, %zu indices",
             mesh.vertices.size(), mesh.indices.size());
}

void VulkanRoadRenderer::setViewProjection(const glm::mat4& view, const glm::mat4& proj)
{
    view_ = view;
    proj_ = proj;
}

void VulkanRoadRenderer::setCarTransform(const glm::mat4& model)
{
    carModel_ = model;
}

void VulkanRoadRenderer::setVehicleState(const VehicleState& state)
{
    vehicleState_    = state;
    vehicleStateSet_ = true;
}

void VulkanRoadRenderer::drawFrame()
{
    if (indexCount_ == 0) return;

    vkWaitForFences(ctx_.device, 1, &ctx_.inFlight[currentFrame_], VK_TRUE, UINT64_MAX);

    uint32_t imageIndex;
    VkResult result = vkAcquireNextImageKHR(ctx_.device, ctx_.swapchain, UINT64_MAX,
        ctx_.imageAvailable[currentFrame_], VK_NULL_HANDLE, &imageIndex);

    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        ctx_.recreateSwapchain(window_);
        recreatePipelines();
        return;
    }

    vkResetFences(ctx_.device, 1, &ctx_.inFlight[currentFrame_]);

    VkCommandBuffer cmd = ctx_.commandBuffers[currentFrame_];
    vkResetCommandBuffer(cmd, 0);

    VkCommandBufferBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    vkBeginCommandBuffer(cmd, &bi);

    // Clear to black; the sky shader fills the colour buffer before road draws.
    VkClearValue clearValues[2];
    clearValues[0].color        = {{ 0.f, 0.f, 0.f, 1.f }};
    clearValues[1].depthStencil = { 1.f, 0 };

    VkRenderPassBeginInfo rpBI{};
    rpBI.sType             = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpBI.renderPass        = ctx_.renderPass;
    rpBI.framebuffer       = ctx_.framebuffers[imageIndex];
    rpBI.renderArea.offset = { 0, 0 };
    rpBI.renderArea.extent = ctx_.swapExtent;
    rpBI.clearValueCount   = 2;
    rpBI.pClearValues      = clearValues;

    vkCmdBeginRenderPass(cmd, &rpBI, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport{ 0, 0,
        (float)ctx_.swapExtent.width, (float)ctx_.swapExtent.height, 0.f, 1.f };
    VkRect2D scissor{ { 0, 0 }, ctx_.swapExtent };
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    glm::mat4 viewProj = proj_ * view_;

    // --- Sky pass: full-screen gradient triangle, no vertex buffer, no depth ---
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipeline_.pipeline);
    vkCmdDraw(cmd, 3, 1, 0, 0);

    // --- Road pass ---
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, roadPipeline_.pipeline);
    {
        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer_.buffer, &offset);
        vkCmdBindIndexBuffer(cmd, indexBuffer_.buffer, 0, VK_INDEX_TYPE_UINT32);

        PushConstants pc;
        pc.viewProj = viewProj;
        pc.model    = glm::mat4(1.f);
        vkCmdPushConstants(cmd, roadPipeline_.pipelineLayout,
            VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);
        vkCmdDrawIndexed(cmd, indexCount_, 1, 0, 0, 0);
    }

    // --- Trees pass (opaque, uses vehicle pipeline) ---
    if (treeIndexCount_ > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                          vehicleOpaquePipeline_.pipeline);
        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &treeVertBuf_.buffer, &offset);
        vkCmdBindIndexBuffer(cmd, treeIdxBuf_.buffer, 0, VK_INDEX_TYPE_UINT32);

        PushConstants pc;
        pc.viewProj = viewProj;
        pc.model    = glm::mat4(1.f);  // trees are already in world space
        vkCmdPushConstants(cmd, vehicleOpaquePipeline_.pipelineLayout,
            VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);
        vkCmdDrawIndexed(cmd, treeIndexCount_, 1, 0, 0, 0);
    }

    // --- Vehicle pass (only when state has been set) ---
    if (vehicleStateSet_ && wheelIndexCount_ > 0 && bodyIndexCount_ > 0) {
        const auto& vs = vehicleState_;
        MultiBodyParams p{};

        // --- Wheels (opaque) ---
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                          vehicleOpaquePipeline_.pipeline);
        {
            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &wheelVertBuf_.buffer, &offset);
            vkCmdBindIndexBuffer(cmd, wheelIdxBuf_.buffer, 0, VK_INDEX_TYPE_UINT32);

            for (int c = 0; c < 4; ++c) {
                float steer = (c < 2) ? vs.steer * p.maxSteerAngleRad : 0.f;

                glm::mat4 m = glm::translate(glm::mat4(1.f), vs.wheelPos[c]);
                m = glm::rotate(m, vs.headingRad + steer, {0.f, 1.f, 0.f});

                PushConstants pc;
                pc.viewProj = viewProj;
                pc.model    = m;
                vkCmdPushConstants(cmd, vehicleOpaquePipeline_.pipelineLayout,
                    VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);
                vkCmdDrawIndexed(cmd, wheelIndexCount_, 1, 0, 0, 0);
            }
        }

        // --- Suspension components (opaque, drawn between wheels and body) ---
        if (armIndexCount_ > 0 && strutIndexCount_ > 0) {
            // Inboard offset: move wheel-side attachment inward so components
            // are visible between wheel and body, not hidden inside the tyre.
            float inboardOffset = 0.13f;  // ~half tyre width

            // Lower control arms — beam from wheel inner face to body mount (horizontal)
            {
                VkDeviceSize offset = 0;
                vkCmdBindVertexBuffers(cmd, 0, 1, &armVertBuf_.buffer, &offset);
                vkCmdBindIndexBuffer(cmd, armIdxBuf_.buffer, 0, VK_INDEX_TYPE_UINT32);

                for (int c = 0; c < 4; ++c) {
                    // Offset wheel-side attachment inboard (toward body center)
                    glm::vec3 hubPos  = vs.wheelPos[c];
                    bool left = (c == 0 || c == 2);  // FL or RL
                    glm::vec3 inward { std::cos(vs.headingRad), 0.f, -std::sin(vs.headingRad) };
                    if (left) inward = -inward;  // left wheels: inboard is -X (right)
                    hubPos += inward * inboardOffset;

                    glm::vec3 bodyMnt = vs.suspTopPos[c];
                    glm::vec3 armEnd  = bodyMnt; armEnd.y = hubPos.y; // horizontal

                    glm::vec3 mid  = (hubPos + armEnd) * 0.5f;
                    glm::vec3 diff = armEnd - hubPos;
                    float len = glm::length(diff);
                    if (len < 0.001f) continue;

                    // Build transform: translate to midpoint, rotate to align Y with arm direction
                    glm::vec3 dir = diff / len;
                    // Rotation from +Y to dir: angle = acos(dir.y), axis = cross(+Y, dir)
                    glm::vec3 up {0.f, 1.f, 0.f};
                    float dot = glm::dot(up, dir);
                    glm::mat4 m = glm::translate(glm::mat4(1.f), mid);
                    if (std::abs(dot) < 0.999f) {
                        glm::vec3 axis = glm::normalize(glm::cross(up, dir));
                        float angle = std::acos(std::clamp(dot, -1.f, 1.f));
                        m = glm::rotate(m, angle, axis);
                    }
                    m = glm::scale(m, glm::vec3(1.f, len, 1.f)); // scale Y to arm length

                    PushConstants pc;
                    pc.viewProj = viewProj;
                    pc.model    = m;
                    vkCmdPushConstants(cmd, vehicleOpaquePipeline_.pipelineLayout,
                        VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);
                    vkCmdDrawIndexed(cmd, armIndexCount_, 1, 0, 0, 0);
                }
            }

            // Vertical struts — cylinder from wheel inner-top to body suspension mount
            {
                VkDeviceSize offset = 0;
                vkCmdBindVertexBuffers(cmd, 0, 1, &strutVertBuf_.buffer, &offset);
                vkCmdBindIndexBuffer(cmd, strutIdxBuf_.buffer, 0, VK_INDEX_TYPE_UINT32);

                for (int c = 0; c < 4; ++c) {
                    glm::vec3 hubTop  = vs.wheelPos[c];
                    // Offset inboard
                    bool left = (c == 0 || c == 2);
                    glm::vec3 inward { std::cos(vs.headingRad), 0.f, -std::sin(vs.headingRad) };
                    if (left) inward = -inward;
                    hubTop += inward * inboardOffset;
                    hubTop.y += p.wheelRadiusM;  // top of wheel
                    glm::vec3 bodyMnt = vs.suspTopPos[c];

                    glm::vec3 mid  = (hubTop + bodyMnt) * 0.5f;
                    glm::vec3 diff = bodyMnt - hubTop;
                    float len = glm::length(diff);
                    if (len < 0.001f) continue;

                    glm::vec3 dir = diff / len;
                    glm::vec3 up {0.f, 1.f, 0.f};
                    float dot = glm::dot(up, dir);
                    glm::mat4 m = glm::translate(glm::mat4(1.f), mid);
                    if (std::abs(dot) < 0.999f) {
                        glm::vec3 axis = glm::normalize(glm::cross(up, dir));
                        float angle = std::acos(std::clamp(dot, -1.f, 1.f));
                        m = glm::rotate(m, angle, axis);
                    }
                    m = glm::scale(m, glm::vec3(1.f, len, 1.f)); // scale Y to strut length

                    PushConstants pc;
                    pc.viewProj = viewProj;
                    pc.model    = m;
                    vkCmdPushConstants(cmd, vehicleOpaquePipeline_.pipelineLayout,
                        VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);
                    vkCmdDrawIndexed(cmd, strutIndexCount_, 1, 0, 0, 0);
                }
            }
        }

        // --- Body box (translucent, drawn after opaque geometry) ---
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                          vehicleTranslucentPipeline_.pipeline);
        {
            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &bodyVertBuf_.buffer, &offset);
            vkCmdBindIndexBuffer(cmd, bodyIdxBuf_.buffer, 0, VK_INDEX_TYPE_UINT32);

            glm::mat4 m = glm::translate(glm::mat4(1.f), vs.position);
            m = glm::rotate(m, vs.headingRad, {0.f, 1.f, 0.f});
            m = glm::rotate(m, vs.rollRad,    {0.f, 0.f, 1.f});
            m = glm::rotate(m, vs.pitchRad,   {1.f, 0.f, 0.f});

            PushConstants pc;
            pc.viewProj = viewProj;
            pc.model    = m;
            vkCmdPushConstants(cmd, vehicleTranslucentPipeline_.pipelineLayout,
                VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);
            vkCmdDrawIndexed(cmd, bodyIndexCount_, 1, 0, 0, 0);
        }
    }

    // --- HUD overlay (screen-space, drawn last, on top of everything) ---
    if (vehicleStateSet_) {
        float winW = static_cast<float>(ctx_.swapExtent.width);
        float winH = static_cast<float>(ctx_.swapExtent.height);
        hud_.buildFrame(vehicleState_, winW, winH);

        if (!hud_.empty()) {
            // Upload HUD geometry (small, rebuilt each frame)
            hudVertBuf_.upload(ctx_,
                hud_.vertices().data(),
                hud_.vertices().size() * sizeof(VehicleVertex),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
            hudIdxBuf_.upload(ctx_,
                hud_.indices().data(),
                hud_.indices().size() * sizeof(uint32_t),
                VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
            hudIndexCount_ = static_cast<uint32_t>(hud_.indices().size());

            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, hudPipeline_.pipeline);
            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &hudVertBuf_.buffer, &offset);
            vkCmdBindIndexBuffer(cmd, hudIdxBuf_.buffer, 0, VK_INDEX_TYPE_UINT32);

            // Orthographic projection: (0,0) = top-left, (winW, winH) = bottom-right
            PushConstants pc;
            pc.viewProj = glm::ortho(0.f, winW, 0.f, winH, -1.f, 1.f);
            pc.model    = glm::mat4(1.f);
            vkCmdPushConstants(cmd, hudPipeline_.pipelineLayout,
                VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);
            vkCmdDrawIndexed(cmd, hudIndexCount_, 1, 0, 0, 0);
        }
    }

    vkCmdEndRenderPass(cmd);
    vkEndCommandBuffer(cmd);

    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo si{};
    si.sType                = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.waitSemaphoreCount   = 1;
    si.pWaitSemaphores      = &ctx_.imageAvailable[currentFrame_];
    si.pWaitDstStageMask    = &waitStage;
    si.commandBufferCount   = 1;
    si.pCommandBuffers      = &cmd;
    si.signalSemaphoreCount = 1;
    si.pSignalSemaphores    = &ctx_.renderFinished[currentFrame_];
    vkQueueSubmit(ctx_.graphicsQueue, 1, &si, ctx_.inFlight[currentFrame_]);

    VkPresentInfoKHR pi{};
    pi.sType              = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    pi.waitSemaphoreCount = 1;
    pi.pWaitSemaphores    = &ctx_.renderFinished[currentFrame_];
    pi.swapchainCount     = 1;
    pi.pSwapchains        = &ctx_.swapchain;
    pi.pImageIndices      = &imageIndex;
    result = vkQueuePresentKHR(ctx_.presentQueue, &pi);

    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized_) {
        framebufferResized_ = false;
        ctx_.recreateSwapchain(window_);
        recreatePipelines();
    }

    currentFrame_ = (currentFrame_ + 1) % VulkanContext::MAX_FRAMES_IN_FLIGHT;
}

void VulkanRoadRenderer::onResize(uint32_t /*w*/, uint32_t /*h*/)
{
    framebufferResized_ = true;
}

void VulkanRoadRenderer::shutdown()
{
    vkDeviceWaitIdle(ctx_.device);
    vertexBuffer_.destroy(ctx_.device);
    indexBuffer_.destroy(ctx_.device);
    wheelVertBuf_.destroy(ctx_.device);
    wheelIdxBuf_.destroy(ctx_.device);
    bodyVertBuf_.destroy(ctx_.device);
    bodyIdxBuf_.destroy(ctx_.device);
    armVertBuf_.destroy(ctx_.device);
    armIdxBuf_.destroy(ctx_.device);
    strutVertBuf_.destroy(ctx_.device);
    strutIdxBuf_.destroy(ctx_.device);
    treeVertBuf_.destroy(ctx_.device);
    treeIdxBuf_.destroy(ctx_.device);
    hudVertBuf_.destroy(ctx_.device);
    hudIdxBuf_.destroy(ctx_.device);
    roadPipeline_.destroy(ctx_.device);
    skyPipeline_.destroy(ctx_.device);
    vehicleOpaquePipeline_.destroy(ctx_.device);
    vehicleTranslucentPipeline_.destroy(ctx_.device);
    hudPipeline_.destroy(ctx_.device);
    ctx_.shutdown();
}
