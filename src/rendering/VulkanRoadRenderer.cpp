#include "VulkanRoadRenderer.h"
#include "core/Logging.h"
#include <glm/gtc/matrix_transform.hpp>
#include <string>

static const std::string SHADER_DIR_STR = SHADER_DIR;

bool VulkanRoadRenderer::initialize(SDL_Window* window)
{
    window_ = window;
    if (!ctx_.initialize(window)) return false;

    if (!pipeline_.create(ctx_,
            SHADER_DIR_STR + "road.vert.spv",
            SHADER_DIR_STR + "road.frag.spv"))
        return false;

    LOG_INFO("VulkanRoadRenderer initialized");
    return true;
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

void VulkanRoadRenderer::drawFrame()
{
    if (indexCount_ == 0) return;

    vkWaitForFences(ctx_.device, 1, &ctx_.inFlight[currentFrame_], VK_TRUE, UINT64_MAX);

    uint32_t imageIndex;
    VkResult result = vkAcquireNextImageKHR(ctx_.device, ctx_.swapchain, UINT64_MAX,
        ctx_.imageAvailable[currentFrame_], VK_NULL_HANDLE, &imageIndex);

    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        ctx_.recreateSwapchain(window_);
        pipeline_.destroy(ctx_.device);
        pipeline_.create(ctx_, SHADER_DIR_STR + "road.vert.spv", SHADER_DIR_STR + "road.frag.spv");
        return;
    }

    vkResetFences(ctx_.device, 1, &ctx_.inFlight[currentFrame_]);

    VkCommandBuffer cmd = ctx_.commandBuffers[currentFrame_];
    vkResetCommandBuffer(cmd, 0);

    VkCommandBufferBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    vkBeginCommandBuffer(cmd, &bi);

    VkClearValue clearValues[2];
    clearValues[0].color        = {{ 0.1f, 0.12f, 0.1f, 1.f }};  // dark green-grey sky
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
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.pipeline);

    // Dynamic viewport/scissor
    VkViewport viewport{ 0, 0,
        (float)ctx_.swapExtent.width, (float)ctx_.swapExtent.height, 0.f, 1.f };
    VkRect2D scissor{ { 0, 0 }, ctx_.swapExtent };
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    // Bind vertex + index buffers
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer_.buffer, &offset);
    vkCmdBindIndexBuffer(cmd, indexBuffer_.buffer, 0, VK_INDEX_TYPE_UINT32);

    // Draw road (model = identity)
    PushConstants pc;
    pc.viewProj = proj_ * view_;
    pc.model    = glm::mat4(1.f);
    vkCmdPushConstants(cmd, pipeline_.pipelineLayout,
        VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);
    vkCmdDrawIndexed(cmd, indexCount_, 1, 0, 0, 0);

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
        pipeline_.destroy(ctx_.device);
        pipeline_.create(ctx_, SHADER_DIR_STR + "road.vert.spv", SHADER_DIR_STR + "road.frag.spv");
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
    pipeline_.destroy(ctx_.device);
    ctx_.shutdown();
}
