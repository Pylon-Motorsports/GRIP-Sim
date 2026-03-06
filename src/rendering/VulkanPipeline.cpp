#include "VulkanPipeline.h"
#include "RoadMesh.h"
#include "VehicleMesh.h"
#include "core/Logging.h"
#include <fstream>
#include <array>
#include <glm/glm.hpp>

std::vector<char> VulkanPipeline::readSpv(const std::string& path)
{
    std::ifstream file(path, std::ios::ate | std::ios::binary);
    if (!file.is_open()) {
        LOG_ERROR("Cannot open shader: %s", path.c_str());
        return {};
    }
    size_t size = (size_t)file.tellg();
    std::vector<char> buf(size);
    file.seekg(0);
    file.read(buf.data(), (std::streamsize)size);
    return buf;
}

VkShaderModule VulkanPipeline::createShaderModule(VkDevice device, const std::vector<char>& code)
{
    VkShaderModuleCreateInfo ci{};
    ci.sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    ci.codeSize = code.size();
    ci.pCode    = reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule mod;
    vkCreateShaderModule(device, &ci, nullptr, &mod);
    return mod;
}

bool VulkanPipeline::create(const VulkanContext& ctx,
                             const std::string&   vertSpvPath,
                             const std::string&   fragSpvPath,
                             const PipelineConfig& config)
{
    auto vertCode = readSpv(vertSpvPath);
    auto fragCode = readSpv(fragSpvPath);
    if (vertCode.empty() || fragCode.empty()) return false;

    VkShaderModule vertMod = createShaderModule(ctx.device, vertCode);
    VkShaderModule fragMod = createShaderModule(ctx.device, fragCode);

    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vertMod;
    stages[0].pName  = "main";
    stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = fragMod;
    stages[1].pName  = "main";

    // Vertex input — omit binding/attributes when pipeline generates its own vertices (e.g. sky)
    VkVertexInputBindingDescription binding{};
    binding.binding   = 0;
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> attrs;
    if (config.vertexLayout == VertexLayout::Vehicle) {
        binding.stride = sizeof(VehicleVertex);
        attrs = {
            { 0, 0, VK_FORMAT_R32G32B32_SFLOAT,    offsetof(VehicleVertex, pos)    },
            { 1, 0, VK_FORMAT_R32G32B32_SFLOAT,    offsetof(VehicleVertex, normal) },
            { 2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(VehicleVertex, color)  },
        };
    } else {
        binding.stride = sizeof(RoadVertex);
        attrs = {
            { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(RoadVertex, position)  },
            { 1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(RoadVertex, normal)    },
            { 2, 0, VK_FORMAT_R32G32_SFLOAT,    offsetof(RoadVertex, uv)        },
            { 3, 0, VK_FORMAT_R32_SFLOAT,       offsetof(RoadVertex, surfaceId) },
        };
    }

    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    if (config.hasVertexInput) {
        vertexInput.vertexBindingDescriptionCount   = 1;
        vertexInput.pVertexBindingDescriptions      = &binding;
        vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attrs.size());
        vertexInput.pVertexAttributeDescriptions    = attrs.data();
    }

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType    = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkViewport viewport{ 0, 0, (float)ctx.swapExtent.width, (float)ctx.swapExtent.height, 0.f, 1.f };
    VkRect2D   scissor { { 0, 0 }, ctx.swapExtent };

    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType         = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.pViewports    = &viewport;
    viewportState.scissorCount  = 1;
    viewportState.pScissors     = &scissor;

    VkPipelineRasterizationStateCreateInfo rast{};
    rast.sType       = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rast.polygonMode = VK_POLYGON_MODE_FILL;
    rast.cullMode    = config.cullMode;
    rast.frontFace   = VK_FRONT_FACE_CLOCKWISE;
    rast.lineWidth   = 1.f;
    rast.depthBiasEnable         = config.depthBiasEnable ? VK_TRUE : VK_FALSE;
    rast.depthBiasConstantFactor = config.depthBiasConstant;
    rast.depthBiasSlopeFactor    = config.depthBiasSlope;

    VkPipelineMultisampleStateCreateInfo msaa{};
    msaa.sType                = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    msaa.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo depth{};
    depth.sType            = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depth.depthTestEnable  = config.depthTest  ? VK_TRUE : VK_FALSE;
    depth.depthWriteEnable = config.depthWrite ? VK_TRUE : VK_FALSE;
    depth.depthCompareOp   = VK_COMPARE_OP_LESS;

    VkPipelineColorBlendAttachmentState blend{};
    blend.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
                         | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    if (config.blendEnable) {
        blend.blendEnable         = VK_TRUE;
        blend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blend.colorBlendOp        = VK_BLEND_OP_ADD;
        blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
        blend.alphaBlendOp        = VK_BLEND_OP_ADD;
    }
    VkPipelineColorBlendStateCreateInfo blendState{};
    blendState.sType           = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    blendState.attachmentCount = 1;
    blendState.pAttachments    = &blend;

    // Dynamic state for viewport/scissor resize
    std::array<VkDynamicState, 2> dynamicStates = {
        VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR
    };
    VkPipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.sType             = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates    = dynamicStates.data();

    // Pipeline layout with push constants (all pipelines share the same range)
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushRange.offset     = 0;
    pushRange.size       = sizeof(PushConstants);

    VkPipelineLayoutCreateInfo layoutCI{};
    layoutCI.sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layoutCI.pushConstantRangeCount = 1;
    layoutCI.pPushConstantRanges    = &pushRange;

    if (vkCreatePipelineLayout(ctx.device, &layoutCI, nullptr, &pipelineLayout) != VK_SUCCESS) {
        LOG_ERROR("Failed to create pipeline layout");
        return false;
    }

    VkGraphicsPipelineCreateInfo pipelineCI{};
    pipelineCI.sType               = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipelineCI.stageCount          = 2;
    pipelineCI.pStages             = stages;
    pipelineCI.pVertexInputState   = &vertexInput;
    pipelineCI.pInputAssemblyState = &inputAssembly;
    pipelineCI.pViewportState      = &viewportState;
    pipelineCI.pRasterizationState = &rast;
    pipelineCI.pMultisampleState   = &msaa;
    pipelineCI.pDepthStencilState  = &depth;
    pipelineCI.pColorBlendState    = &blendState;
    pipelineCI.pDynamicState       = &dynamicState;
    pipelineCI.layout              = pipelineLayout;
    pipelineCI.renderPass          = ctx.renderPass;
    pipelineCI.subpass             = 0;

    VkResult res = vkCreateGraphicsPipelines(ctx.device, VK_NULL_HANDLE, 1, &pipelineCI, nullptr, &pipeline);
    vkDestroyShaderModule(ctx.device, vertMod, nullptr);
    vkDestroyShaderModule(ctx.device, fragMod, nullptr);

    if (res != VK_SUCCESS) {
        LOG_ERROR("Failed to create graphics pipeline");
        return false;
    }
    return true;
}

void VulkanPipeline::destroy(VkDevice device)
{
    vkDestroyPipeline(device, pipeline, nullptr);
    vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
    pipeline       = VK_NULL_HANDLE;
    pipelineLayout = VK_NULL_HANDLE;
}
