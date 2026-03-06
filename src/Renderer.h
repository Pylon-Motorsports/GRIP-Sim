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
    int   activeScenario = 0;
    int   numScenarios   = 0;
    const char* const* scenarioNames = nullptr;
    float speedKmh       = 0.f;
    float rpm            = 0.f;
    float rpmLimit       = 6500.f;
};

struct Renderer {
    bool init(const VulkanContext& ctx);
    void draw(VkCommandBuffer cmd, uint32_t W, uint32_t H,
              const Vehicle& veh, const HudData& hud,
              const std::vector<Bump>& bumps);
    void shutdown(VkDevice dev);

    // Returns index of clicked button, or -1 if none
    int hitTestButton(int mouseX, int mouseY, uint32_t W, uint32_t H, int numScenarios) const;

private:
    struct PushConst { glm::mat4 viewProj; glm::mat4 model; };
    struct Slice     { uint32_t firstIdx; uint32_t idxCount; };

    VkDevice         dev_         = VK_NULL_HANDLE;
    VkPipelineLayout layout_      = VK_NULL_HANDLE;
    VkPipeline       pipeline_    = VK_NULL_HANDLE;
    VkPipelineLayout skyLayout_   = VK_NULL_HANDLE;
    VkPipeline       skyPipeline_ = VK_NULL_HANDLE;

    VkBuffer       vbuf_ = VK_NULL_HANDLE;  VkDeviceMemory vmem_ = VK_NULL_HANDLE;
    VkBuffer       ibuf_ = VK_NULL_HANDLE;  VkDeviceMemory imem_ = VK_NULL_HANDLE;

    // Dynamic HUD buffer (rebuilt each frame)
    VkBuffer       hudVbuf_ = VK_NULL_HANDLE;  VkDeviceMemory hudVmem_ = VK_NULL_HANDLE;
    VkBuffer       hudIbuf_ = VK_NULL_HANDLE;  VkDeviceMemory hudImem_ = VK_NULL_HANDLE;
    static constexpr uint32_t HUD_MAX_VERTS = 4096;
    static constexpr uint32_t HUD_MAX_IDX   = 8192;

    Slice ground_, body_, rim_, tire_, axle_, bumpWide_, bumpNarrow_;

    void drawScene(VkCommandBuffer cmd, const glm::mat4& vp, const Vehicle& veh,
                   const std::vector<Bump>& bumps);
    void drawHud(VkCommandBuffer cmd, uint32_t W, uint32_t H, const HudData& hud);

    // Button layout constants
    static constexpr float BTN_H  = 28.f;
    static constexpr float BTN_W  = 130.f;
    static constexpr float BTN_GAP = 6.f;
    static constexpr float BTN_Y  = 6.f;
};
