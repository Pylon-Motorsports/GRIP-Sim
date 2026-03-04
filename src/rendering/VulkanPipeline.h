#pragma once
#include "VulkanContext.h"
#include <string>

/// Push constants layout shared between CPU and shaders.
/// Fits within the Vulkan minimum-spec 128-byte push constant range.
struct PushConstants {
    glm::mat4 viewProj { 1.f };  // 64 bytes
    glm::mat4 model    { 1.f };  // 64 bytes
};
static_assert(sizeof(PushConstants) <= 128, "PushConstants exceed min-spec limit");

/// Owns the graphics pipeline and its layout for the road and car meshes.
struct VulkanPipeline {
    VkPipeline       pipeline       { VK_NULL_HANDLE };
    VkPipelineLayout pipelineLayout { VK_NULL_HANDLE };

    /// Create pipeline for road/car rendering.
    bool create(const VulkanContext& ctx,
                const std::string& vertSpvPath,
                const std::string& fragSpvPath);

    void destroy(VkDevice device);

private:
    static std::vector<char> readSpv(const std::string& path);
    static VkShaderModule    createShaderModule(VkDevice device, const std::vector<char>& code);
};
