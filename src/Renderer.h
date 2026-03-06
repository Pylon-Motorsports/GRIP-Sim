#pragma once
#include "VulkanContext.h"
#include "Vehicle.h"
#include <glm/glm.hpp>
#include <cstdint>

struct Vertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec4 color;
};

struct Renderer {
    bool init(const VulkanContext& ctx);
    void draw(VkCommandBuffer cmd, uint32_t W, uint32_t H, const Vehicle& veh);
    void shutdown(VkDevice dev);

private:
    struct PushConst { glm::mat4 viewProj; glm::mat4 model; };
    struct Slice     { uint32_t firstIdx; uint32_t idxCount; };

    VkPipelineLayout layout_      = VK_NULL_HANDLE;
    VkPipeline       pipeline_    = VK_NULL_HANDLE;
    VkPipelineLayout skyLayout_   = VK_NULL_HANDLE;
    VkPipeline       skyPipeline_ = VK_NULL_HANDLE;

    VkBuffer       vbuf_ = VK_NULL_HANDLE;  VkDeviceMemory vmem_ = VK_NULL_HANDLE;
    VkBuffer       ibuf_ = VK_NULL_HANDLE;  VkDeviceMemory imem_ = VK_NULL_HANDLE;

    Slice ground_, body_, wheel_;

    void drawScene(VkCommandBuffer cmd, const glm::mat4& vp, const Vehicle& veh);
};
