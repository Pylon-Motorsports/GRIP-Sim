#pragma once
#include "IRoadRenderer.h"
#include "VulkanContext.h"
#include "VulkanPipeline.h"
#include "VulkanBuffer.h"
#include <glm/glm.hpp>

/// Concrete IRoadRenderer implementation using Vulkan.
/// Manages vertex/index buffers for the road mesh and issues draw calls each frame.
class VulkanRoadRenderer : public IRoadRenderer {
public:
    bool initialize(SDL_Window* window) override;
    void uploadMesh(const RoadMesh& mesh) override;
    void setViewProjection(const glm::mat4& view, const glm::mat4& proj) override;
    void setCarTransform(const glm::mat4& model) override;
    void drawFrame() override;
    void onResize(uint32_t w, uint32_t h) override;
    void shutdown() override;

private:
    VulkanContext  ctx_;
    VulkanPipeline pipeline_;
    VulkanBuffer   vertexBuffer_;
    VulkanBuffer   indexBuffer_;

    glm::mat4 view_     { 1.f };
    glm::mat4 proj_     { 1.f };
    glm::mat4 carModel_ { 1.f };

    uint32_t indexCount_    { 0 };
    int      currentFrame_  { 0 };
    bool     framebufferResized_ { false };
    SDL_Window* window_ { nullptr };
};
