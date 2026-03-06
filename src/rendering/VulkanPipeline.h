#pragma once
#include "VulkanContext.h"
#include <glm/glm.hpp>
#include <string>
#include <vulkan/vulkan.h>

/// Push constants layout shared between CPU and shaders.
/// Fits within the Vulkan minimum-spec 128-byte push constant range.
struct PushConstants {
    glm::mat4 viewProj { 1.f };  // 64 bytes
    glm::mat4 model    { 1.f };  // 64 bytes
};
static_assert(sizeof(PushConstants) <= 128, "PushConstants exceed min-spec limit");

/// Selects which vertex struct the pipeline reads from the vertex buffer.
enum class VertexLayout { Road, Vehicle };

/// Configures optional pipeline features — defaults produce the road/mesh pipeline.
struct PipelineConfig {
    bool            hasVertexInput { true  };   ///< false = procedural vertex (e.g. sky triangle)
    bool            depthTest      { true  };   ///< false = draw on top regardless of depth
    bool            depthWrite     { true  };   ///< false = don't write depth (e.g. sky pass)
    bool            blendEnable    { false };   ///< true  = alpha blending (for translucent geometry)
    VkCullModeFlags cullMode       { VK_CULL_MODE_NONE };
    VertexLayout    vertexLayout   { VertexLayout::Road };
    bool            depthBiasEnable { false };  ///< Polygon offset to reduce Z-fighting
    float           depthBiasConstant { 0.f };  ///< Constant depth bias (in depth buffer units)
    float           depthBiasSlope    { 0.f };  ///< Slope-scaled depth bias
};

/// Owns a Vulkan graphics pipeline and its layout.
struct VulkanPipeline {
    VkPipeline       pipeline       { VK_NULL_HANDLE };
    VkPipelineLayout pipelineLayout { VK_NULL_HANDLE };

    /// Create pipeline. Uses default PipelineConfig for road/mesh geometry.
    bool create(const VulkanContext& ctx,
                const std::string&  vertSpvPath,
                const std::string&  fragSpvPath,
                const PipelineConfig& config = {});

    void destroy(VkDevice device);

private:
    static std::vector<char> readSpv(const std::string& path);
    static VkShaderModule    createShaderModule(VkDevice device, const std::vector<char>& code);
};
