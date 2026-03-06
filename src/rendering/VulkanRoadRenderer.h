#pragma once
#include "IRoadRenderer.h"
#include "VulkanContext.h"
#include "VulkanPipeline.h"
#include "VulkanBuffer.h"
#include "VehicleMesh.h"
#include "HudOverlay.h"
#include <glm/glm.hpp>

/// Concrete IRoadRenderer implementation using Vulkan.
/// Manages vertex/index buffers for the road mesh and issues draw calls each frame.
/// Draws a procedural sky gradient before the road geometry each frame.
/// Also draws a debug vehicle body (translucent box) and 4 wheels (opaque cylinders).
class VulkanRoadRenderer : public IRoadRenderer {
public:
    bool initialize(SDL_Window* window) override;
    void uploadMesh(const RoadMesh& mesh) override;
    void setViewProjection(const glm::mat4& view, const glm::mat4& proj) override;
    void setCarTransform(const glm::mat4& model) override;
    void setVehicleState(const VehicleState& state) override;
    void drawFrame() override;
    void onResize(uint32_t w, uint32_t h) override;
    void shutdown() override;

private:
    VulkanContext  ctx_;
    VulkanPipeline roadPipeline_;
    VulkanPipeline skyPipeline_;
    VulkanPipeline vehicleOpaquePipeline_;       ///< Wheels (opaque)
    VulkanPipeline vehicleTranslucentPipeline_;  ///< Body box (alpha blend, no depth write)
    VulkanBuffer   vertexBuffer_;
    VulkanBuffer   indexBuffer_;
    VulkanBuffer   wheelVertBuf_;
    VulkanBuffer   wheelIdxBuf_;
    VulkanBuffer   bodyVertBuf_;
    VulkanBuffer   bodyIdxBuf_;
    VulkanBuffer   armVertBuf_;       ///< Lower control arm mesh
    VulkanBuffer   armIdxBuf_;
    VulkanBuffer   strutVertBuf_;     ///< Strut (spring/damper) mesh
    VulkanBuffer   strutIdxBuf_;
    VulkanBuffer   treeVertBuf_;      ///< Batched tree geometry (trunks + canopies)
    VulkanBuffer   treeIdxBuf_;
    VulkanBuffer   hudVertBuf_;       ///< HUD overlay geometry (rebuilt each frame)
    VulkanBuffer   hudIdxBuf_;
    VulkanPipeline hudPipeline_;      ///< No depth test, alpha blend, vehicle vertex layout
    HudOverlay     hud_;

    glm::mat4 view_     { 1.f };
    glm::mat4 proj_     { 1.f };
    glm::mat4 carModel_ { 1.f };

    VehicleState vehicleState_ {};
    bool         vehicleStateSet_ { false };

    uint32_t indexCount_         { 0 };
    uint32_t wheelIndexCount_    { 0 };
    uint32_t bodyIndexCount_     { 0 };
    uint32_t armIndexCount_      { 0 };
    uint32_t strutIndexCount_    { 0 };
    uint32_t treeIndexCount_     { 0 };
    uint32_t hudIndexCount_      { 0 };
    int      currentFrame_       { 0 };
    bool     framebufferResized_ { false };
    SDL_Window* window_          { nullptr };

    void createVehiclePipelines();
    void uploadVehicleMeshes();
    /// Recreate all pipelines after a swapchain resize.
    void recreatePipelines();
};
