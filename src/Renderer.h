#pragma once
#include "VulkanContext.h"
#include "Vehicle.h"
#include "Scenario.h"
#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

struct Vertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec4 color;
};

struct HudData {
    float speedKmh       = 0.f;
    float rpm            = 0.f;
    float rpmLimit       = 6500.f;
    int   gear           = 1;
};

struct TrailGeometry {
    const Vertex* verts = nullptr;
    uint32_t vertCount  = 0;
    const uint32_t* indices = nullptr;
    uint32_t idxCount   = 0;
};

struct Renderer {
    bool init(const VulkanContext& ctx);
    void draw(VkCommandBuffer cmd, uint32_t W, uint32_t H,
              const Vehicle& veh, const HudData& hud,
              const Playground& playground,
              const TrailGeometry* trails = nullptr);
    void shutdown(VkDevice dev);

private:
    struct PushConst { glm::mat4 viewProj; glm::mat4 model; };
    struct Slice     { uint32_t firstIdx; uint32_t idxCount; };

    VkDevice         dev_              = VK_NULL_HANDLE;
    VkPipelineLayout layout_           = VK_NULL_HANDLE;
    VkPipeline       pipeline_         = VK_NULL_HANDLE;  // depth write ON
    VkPipeline       translucentPipe_  = VK_NULL_HANDLE;  // depth write OFF (for see-through body)
    VkPipelineLayout skyLayout_        = VK_NULL_HANDLE;
    VkPipeline       skyPipeline_      = VK_NULL_HANDLE;

    VkBuffer       vbuf_ = VK_NULL_HANDLE;  VkDeviceMemory vmem_ = VK_NULL_HANDLE;
    VkBuffer       ibuf_ = VK_NULL_HANDLE;  VkDeviceMemory imem_ = VK_NULL_HANDLE;

    // Dynamic HUD buffer (rebuilt each frame)
    VkBuffer       hudVbuf_ = VK_NULL_HANDLE;  VkDeviceMemory hudVmem_ = VK_NULL_HANDLE;
    VkBuffer       hudIbuf_ = VK_NULL_HANDLE;  VkDeviceMemory hudImem_ = VK_NULL_HANDLE;
    static constexpr uint32_t HUD_MAX_VERTS = 4096;
    static constexpr uint32_t HUD_MAX_IDX   = 8192;

    // Dynamic tire trail buffer (rebuilt each frame)
    VkBuffer       trailVbuf_ = VK_NULL_HANDLE;  VkDeviceMemory trailVmem_ = VK_NULL_HANDLE;
    VkBuffer       trailIbuf_ = VK_NULL_HANDLE;  VkDeviceMemory trailImem_ = VK_NULL_HANDLE;
    static constexpr uint32_t TRAIL_MAX_VERTS = 16384;  // 4 wheels x 2000 pts x 2 verts
    static constexpr uint32_t TRAIL_MAX_IDX   = 49152;  // 4 wheels x 1999 segs x 6 indices
    uint32_t trailIdxCount_ = 0;

    // Dynamic ground label buffer (rebuilt each frame)
    VkBuffer       labelVbuf_ = VK_NULL_HANDLE;  VkDeviceMemory labelVmem_ = VK_NULL_HANDLE;
    VkBuffer       labelIbuf_ = VK_NULL_HANDLE;  VkDeviceMemory labelImem_ = VK_NULL_HANDLE;
    static constexpr uint32_t LABEL_MAX_VERTS = 8192;
    static constexpr uint32_t LABEL_MAX_IDX   = 16384;
    uint32_t labelIdxCount_ = 0;

    Slice ground_, body_, rim_, tire_, axle_, unitBump_, subframe_;

    void drawScene(VkCommandBuffer cmd, const glm::mat4& vp, const Vehicle& veh,
                   const Playground& playground, bool hasTrails);
    void drawHud(VkCommandBuffer cmd, uint32_t W, uint32_t H, const HudData& hud);
};
