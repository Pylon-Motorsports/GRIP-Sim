#include "Renderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <fstream>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <cstring>

// ---- helpers ---------------------------------------------------------------

static std::vector<char> readFile(const char* path)
{
    std::ifstream f(path, std::ios::ate | std::ios::binary);
    if (!f.is_open()) { std::fprintf(stderr, "Cannot open %s\n", path); return {}; }
    size_t sz = (size_t)f.tellg();
    std::vector<char> buf(sz);
    f.seekg(0);
    f.read(buf.data(), sz);
    return buf;
}

static VkShaderModule makeModule(VkDevice dev, const std::vector<char>& code)
{
    VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    ci.codeSize = code.size();
    ci.pCode    = reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule m;
    vkCreateShaderModule(dev, &ci, nullptr, &m);
    return m;
}

// ---- mesh generation -------------------------------------------------------

using VList = std::vector<Vertex>;
using IList = std::vector<uint32_t>;

static void makeBox(float hx, float hy, float hz, glm::vec4 col,
                    VList& verts, IList& idx)
{
    auto face = [&](glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 n) {
        uint32_t b = (uint32_t)verts.size();
        verts.push_back({ p0, n, col });
        verts.push_back({ p1, n, col });
        verts.push_back({ p2, n, col });
        verts.push_back({ p3, n, col });
        idx.insert(idx.end(), { b, b+1, b+2, b, b+2, b+3 });
    };
    face({-hx,-hy, hz}, { hx,-hy, hz}, { hx, hy, hz}, {-hx, hy, hz}, { 0, 0, 1});
    face({ hx,-hy,-hz}, {-hx,-hy,-hz}, {-hx, hy,-hz}, { hx, hy,-hz}, { 0, 0,-1});
    face({ hx,-hy, hz}, { hx,-hy,-hz}, { hx, hy,-hz}, { hx, hy, hz}, { 1, 0, 0});
    face({-hx,-hy,-hz}, {-hx,-hy, hz}, {-hx, hy, hz}, {-hx, hy,-hz}, {-1, 0, 0});
    face({-hx, hy, hz}, { hx, hy, hz}, { hx, hy,-hz}, {-hx, hy,-hz}, { 0, 1, 0});
    face({-hx,-hy,-hz}, { hx,-hy,-hz}, { hx,-hy, hz}, {-hx,-hy, hz}, { 0,-1, 0});
}

static void makeCylinder(float radius, float halfW, int segs, glm::vec4 col,
                         VList& verts, IList& idx)
{
    const float TAU = 6.283185307f;

    uint32_t cL = (uint32_t)verts.size();
    verts.push_back({{ -halfW, 0, 0 }, { -1, 0, 0 }, col });
    uint32_t cR = (uint32_t)verts.size();
    verts.push_back({{  halfW, 0, 0 }, {  1, 0, 0 }, col });

    struct Ring { uint32_t capL, capR, sideL, sideR; };
    std::vector<Ring> rings(segs);

    for (int i = 0; i < segs; ++i) {
        float a  = i * TAU / segs;
        float cy = radius * std::cos(a);
        float cz = radius * std::sin(a);
        glm::vec3 outward = glm::normalize(glm::vec3{0, cy, cz});

        rings[i].capL  = (uint32_t)verts.size();
        verts.push_back({{ -halfW, cy, cz }, { -1, 0, 0 }, col });
        rings[i].capR  = (uint32_t)verts.size();
        verts.push_back({{  halfW, cy, cz }, {  1, 0, 0 }, col });
        rings[i].sideL = (uint32_t)verts.size();
        verts.push_back({{ -halfW, cy, cz }, outward, col });
        rings[i].sideR = (uint32_t)verts.size();
        verts.push_back({{  halfW, cy, cz }, outward, col });
    }

    for (int i = 0; i < segs; ++i) {
        int j = (i + 1) % segs;
        idx.insert(idx.end(), { cL, rings[j].capL, rings[i].capL });
        idx.insert(idx.end(), { cR, rings[i].capR, rings[j].capR });
        idx.insert(idx.end(), {
            rings[i].sideL, rings[j].sideL, rings[j].sideR,
            rings[i].sideL, rings[j].sideR, rings[i].sideR });
    }
}

static void makeGround(float halfSize, VList& verts, IList& idx)
{
    glm::vec4 col{ 0.4f, 0.45f, 0.3f, 0.f };
    glm::vec3 n{ 0, 1, 0 };
    float s = halfSize;
    uint32_t b = (uint32_t)verts.size();
    verts.push_back({{ -s, 0,  s }, n, col });
    verts.push_back({{  s, 0,  s }, n, col });
    verts.push_back({{  s, 0, -s }, n, col });
    verts.push_back({{ -s, 0, -s }, n, col });
    idx.insert(idx.end(), { b, b+1, b+2, b, b+2, b+3 });
}

// ---- 5x7 bitmap font -------------------------------------------------------

// Each glyph: 7 rows, 5 bits per row (MSB = leftmost pixel).
// Covers ASCII 32..90 (space through Z) plus a few extras.
static const uint8_t FONT_5x7[][7] = {
    // 32: ' '
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // 33: '!'
    {0x04,0x04,0x04,0x04,0x04,0x00,0x04},
    // 34: '"'
    {0x0A,0x0A,0x00,0x00,0x00,0x00,0x00},
    // 35: '#'
    {0x0A,0x1F,0x0A,0x0A,0x1F,0x0A,0x00},
    // 36: '$'
    {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04},
    // 37: '%'
    {0x18,0x19,0x02,0x04,0x08,0x13,0x03},
    // 38: '&'
    {0x08,0x14,0x14,0x08,0x15,0x12,0x0D},
    // 39: '''
    {0x04,0x04,0x00,0x00,0x00,0x00,0x00},
    // 40: '('
    {0x02,0x04,0x08,0x08,0x08,0x04,0x02},
    // 41: ')'
    {0x08,0x04,0x02,0x02,0x02,0x04,0x08},
    // 42: '*'
    {0x00,0x04,0x15,0x0E,0x15,0x04,0x00},
    // 43: '+'
    {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
    // 44: ','
    {0x00,0x00,0x00,0x00,0x00,0x04,0x08},
    // 45: '-'
    {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
    // 46: '.'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x04},
    // 47: '/'
    {0x01,0x01,0x02,0x04,0x08,0x10,0x10},
    // 48: '0'
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    // 49: '1'
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    // 50: '2'
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
    // 51: '3'
    {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    // 52: '4'
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    // 53: '5'
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    // 54: '6'
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
    // 55: '7'
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    // 56: '8'
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    // 57: '9'
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
    // 58: ':'
    {0x00,0x00,0x04,0x00,0x00,0x04,0x00},
    // 59: ';'
    {0x00,0x00,0x04,0x00,0x00,0x04,0x08},
    // 60: '<'
    {0x02,0x04,0x08,0x10,0x08,0x04,0x02},
    // 61: '='
    {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
    // 62: '>'
    {0x08,0x04,0x02,0x01,0x02,0x04,0x08},
    // 63: '?'
    {0x0E,0x11,0x01,0x02,0x04,0x00,0x04},
    // 64: '@'
    {0x0E,0x11,0x17,0x15,0x17,0x10,0x0E},
    // 65: 'A'
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
    // 66: 'B'
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    // 67: 'C'
    {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    // 68: 'D'
    {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
    // 69: 'E'
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
    // 70: 'F'
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    // 71: 'G'
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E},
    // 72: 'H'
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    // 73: 'I'
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
    // 74: 'J'
    {0x07,0x02,0x02,0x02,0x02,0x12,0x0C},
    // 75: 'K'
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
    // 76: 'L'
    {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    // 77: 'M'
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
    // 78: 'N'
    {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    // 79: 'O'
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    // 80: 'P'
    {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    // 81: 'Q'
    {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
    // 82: 'R'
    {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    // 83: 'S'
    {0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E},
    // 84: 'T'
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    // 85: 'U'
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    // 86: 'V'
    {0x11,0x11,0x11,0x11,0x0A,0x0A,0x04},
    // 87: 'W'
    {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},
    // 88: 'X'
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
    // 89: 'Y'
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
    // 90: 'Z'
    {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
};

// HUD Z: near the near plane so HUD always passes depth test over scene geometry.
// Ortho maps Z=[-1,1] to depth [0,1], so Z=-0.99 maps to depth ~0.005.
static constexpr float HUD_Z = -0.99f;

// Build quads for a text string at (x,y) with given pixel scale and color.
// normal=(0,0,0) for unlit HUD rendering.
static void buildText(const char* str, float x, float y, float scale,
                      glm::vec4 col, VList& verts, IList& idx)
{
    constexpr glm::vec3 N{0,0,0};  // zero normal = unlit
    float cx = x;
    for (const char* p = str; *p; ++p) {
        int ch = (int)(unsigned char)*p;
        if (ch < 32 || ch > 90) {
            // Lowercase -> uppercase for simplicity
            if (ch >= 'a' && ch <= 'z') ch = ch - 'a' + 'A';
            else { cx += 6.f * scale; continue; }
        }
        int gi = ch - 32;
        for (int row = 0; row < 7; ++row) {
            uint8_t bits = FONT_5x7[gi][row];
            for (int col_i = 0; col_i < 5; ++col_i) {
                if (bits & (0x10 >> col_i)) {
                    float px = cx + col_i * scale;
                    float py = y + row * scale;
                    uint32_t b = (uint32_t)verts.size();
                    verts.push_back({{px,        py,        HUD_Z}, N, col});
                    verts.push_back({{px+scale,  py,        HUD_Z}, N, col});
                    verts.push_back({{px+scale,  py+scale,  HUD_Z}, N, col});
                    verts.push_back({{px,        py+scale,  HUD_Z}, N, col});
                    idx.insert(idx.end(), {b, b+1, b+2, b, b+2, b+3});
                }
            }
        }
        cx += 6.f * scale;
    }
}

// Build a filled rectangle
static void buildRect(float x, float y, float w, float h, glm::vec4 col,
                      VList& verts, IList& idx)
{
    constexpr glm::vec3 N{0,0,0};
    uint32_t b = (uint32_t)verts.size();
    verts.push_back({{x,   y,   HUD_Z}, N, col});
    verts.push_back({{x+w, y,   HUD_Z}, N, col});
    verts.push_back({{x+w, y+h, HUD_Z}, N, col});
    verts.push_back({{x,   y+h, HUD_Z}, N, col});
    idx.insert(idx.end(), {b, b+1, b+2, b, b+2, b+3});
}

// ---- init ------------------------------------------------------------------

bool Renderer::init(const VulkanContext& ctx)
{
    VkDevice dev = ctx.device;
    dev_ = dev;

    // ---- Pipeline layout ----
    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pcr.size       = sizeof(PushConst);

    VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pli.pushConstantRangeCount = 1;
    pli.pPushConstantRanges    = &pcr;
    vkCreatePipelineLayout(dev, &pli, nullptr, &layout_);

    // ---- Shaders ----
    auto vertCode = readFile("shaders/scene.vert.spv");
    auto fragCode = readFile("shaders/scene.frag.spv");
    if (vertCode.empty() || fragCode.empty()) return false;

    VkShaderModule vertMod = makeModule(dev, vertCode);
    VkShaderModule fragMod = makeModule(dev, fragCode);

    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vertMod;
    stages[0].pName  = "main";
    stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = fragMod;
    stages[1].pName  = "main";

    // ---- Vertex input ----
    VkVertexInputBindingDescription bind{};
    bind.stride    = sizeof(Vertex);
    bind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription attrs[3]{};
    attrs[0] = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT,    offsetof(Vertex, pos)    };
    attrs[1] = { 1, 0, VK_FORMAT_R32G32B32_SFLOAT,    offsetof(Vertex, normal) };
    attrs[2] = { 2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(Vertex, color)  };

    VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vi.vertexBindingDescriptionCount   = 1;  vi.pVertexBindingDescriptions   = &bind;
    vi.vertexAttributeDescriptionCount = 3;  vi.pVertexAttributeDescriptions = attrs;

    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vps{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vps.viewportCount = 1;
    vps.scissorCount  = 1;

    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode    = VK_CULL_MODE_NONE;
    rs.lineWidth   = 1.f;

    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    ds.depthTestEnable  = VK_TRUE;
    ds.depthWriteEnable = VK_TRUE;
    ds.depthCompareOp   = VK_COMPARE_OP_LESS_OR_EQUAL;

    VkPipelineColorBlendAttachmentState cba{};
    cba.colorWriteMask = 0xF;

    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    cb.attachmentCount = 1;
    cb.pAttachments    = &cba;

    VkDynamicState dynStates[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dyn{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dyn.dynamicStateCount = 2;
    dyn.pDynamicStates    = dynStates;

    VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    gp.stageCount          = 2;
    gp.pStages             = stages;
    gp.pVertexInputState   = &vi;
    gp.pInputAssemblyState = &ia;
    gp.pViewportState      = &vps;
    gp.pRasterizationState = &rs;
    gp.pMultisampleState   = &ms;
    gp.pDepthStencilState  = &ds;
    gp.pColorBlendState    = &cb;
    gp.pDynamicState       = &dyn;
    gp.layout              = layout_;
    gp.renderPass          = ctx.renderPass;

    vkCreateGraphicsPipelines(dev, VK_NULL_HANDLE, 1, &gp, nullptr, &pipeline_);

    vkDestroyShaderModule(dev, fragMod, nullptr);
    vkDestroyShaderModule(dev, vertMod, nullptr);

    // ---- Sky pipeline ----
    {
        auto skyVert = readFile("shaders/sky.vert.spv");
        auto skyFrag = readFile("shaders/sky.frag.spv");
        if (skyVert.empty() || skyFrag.empty()) return false;

        VkShaderModule sv = makeModule(dev, skyVert);
        VkShaderModule sf = makeModule(dev, skyFrag);

        VkPipelineShaderStageCreateInfo skyStages[2]{};
        skyStages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        skyStages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
        skyStages[0].module = sv;
        skyStages[0].pName  = "main";
        skyStages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        skyStages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
        skyStages[1].module = sf;
        skyStages[1].pName  = "main";

        VkPipelineLayoutCreateInfo skyPli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        vkCreatePipelineLayout(dev, &skyPli, nullptr, &skyLayout_);

        VkPipelineVertexInputStateCreateInfo skyVi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

        VkPipelineRasterizationStateCreateInfo skyRs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        skyRs.polygonMode = VK_POLYGON_MODE_FILL;
        skyRs.cullMode    = VK_CULL_MODE_NONE;
        skyRs.lineWidth   = 1.f;

        VkPipelineDepthStencilStateCreateInfo skyDs{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
        skyDs.depthTestEnable  = VK_TRUE;
        skyDs.depthWriteEnable = VK_FALSE;
        skyDs.depthCompareOp   = VK_COMPARE_OP_LESS_OR_EQUAL;

        VkGraphicsPipelineCreateInfo skyGp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        skyGp.stageCount          = 2;
        skyGp.pStages             = skyStages;
        skyGp.pVertexInputState   = &skyVi;
        skyGp.pInputAssemblyState = &ia;
        skyGp.pViewportState      = &vps;
        skyGp.pRasterizationState = &skyRs;
        skyGp.pMultisampleState   = &ms;
        skyGp.pDepthStencilState  = &skyDs;
        skyGp.pColorBlendState    = &cb;
        skyGp.pDynamicState       = &dyn;
        skyGp.layout              = skyLayout_;
        skyGp.renderPass          = ctx.renderPass;

        vkCreateGraphicsPipelines(dev, VK_NULL_HANDLE, 1, &skyGp, nullptr, &skyPipeline_);

        vkDestroyShaderModule(dev, sf, nullptr);
        vkDestroyShaderModule(dev, sv, nullptr);
    }

    // ---- Generate meshes ----
    VList allV;
    IList allI;

    auto record = [&](auto fn) -> Slice {
        uint32_t iOff = (uint32_t)allI.size();
        fn(allV, allI);
        return { iOff, (uint32_t)allI.size() - iOff };
    };

    ground_ = record([](VList& v, IList& i){ makeGround(100.f, v, i); });

    glm::vec4 bodyCol { 0.25f, 0.55f, 0.30f, 1.f };  // green
    glm::vec4 rimCol  { 0.90f, 0.92f, 0.95f, 1.f };  // white
    glm::vec4 tireCol { 0.10f, 0.10f, 0.10f, 1.f };  // black rubber

    body_  = record([&](VList& v, IList& i){
        makeBox(Vehicle::BODY_HALF_W, Vehicle::BODY_HALF_H, Vehicle::BODY_HALF_L, bodyCol, v, i);
    });
    rim_ = record([&](VList& v, IList& i){
        makeCylinder(Vehicle::RIM_RADIUS, Vehicle::RIM_HALF_W, 16, rimCol, v, i);
    });
    tire_ = record([&](VList& v, IList& i){
        makeCylinder(Vehicle::TIRE_RADIUS, Vehicle::TIRE_HALF_W, 16, tireCol, v, i);
    });

    // Axle template: thin cylinder along X, unit length (will be scaled per-axle)
    glm::vec4 axleCol { 0.4f, 0.4f, 0.45f, 1.f };  // dark steel
    axle_ = record([&](VList& v, IList& i){
        makeCylinder(0.015f, 0.5f, 8, axleCol, v, i);  // radius=1.5cm, half-length=0.5 (scaled)
    });

    // Bump templates: yellow boxes
    glm::vec4 bumpCol { 0.85f, 0.75f, 0.15f, 1.f };
    bumpWide_ = record([&](VList& v, IList& i){
        makeBox(1.5f, 0.04f, 0.25f, bumpCol, v, i);   // 3m wide x 8cm tall x 50cm long
    });
    bumpNarrow_ = record([&](VList& v, IList& i){
        makeBox(0.75f, 0.04f, 0.25f, bumpCol, v, i);   // 1.5m wide
    });

    // ---- Upload scene geometry ----
    {
        VkDeviceSize vSz = allV.size() * sizeof(Vertex);
        vbuf_ = ctx.allocBuffer(vSz,
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            vmem_);
        void* p; vkMapMemory(dev, vmem_, 0, vSz, 0, &p);
        std::memcpy(p, allV.data(), vSz);
        vkUnmapMemory(dev, vmem_);
    }
    {
        VkDeviceSize iSz = allI.size() * sizeof(uint32_t);
        ibuf_ = ctx.allocBuffer(iSz,
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            imem_);
        void* p; vkMapMemory(dev, imem_, 0, iSz, 0, &p);
        std::memcpy(p, allI.data(), iSz);
        vkUnmapMemory(dev, imem_);
    }

    // ---- HUD buffers (dynamic, updated each frame) ----
    hudVbuf_ = ctx.allocBuffer(HUD_MAX_VERTS * sizeof(Vertex),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        hudVmem_);
    hudIbuf_ = ctx.allocBuffer(HUD_MAX_IDX * sizeof(uint32_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        hudImem_);

    std::printf("Meshes: %zu verts, %zu indices\n", allV.size(), allI.size());
    return true;
}

// ---- draw ------------------------------------------------------------------

void Renderer::drawScene(VkCommandBuffer cmd, const glm::mat4& vp, const Vehicle& veh,
                         const std::vector<Bump>& bumps)
{
    auto push = [&](const glm::mat4& model) {
        PushConst pc{ vp, model };
        vkCmdPushConstants(cmd, layout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
    };
    auto drawSlice = [&](const Slice& s) {
        vkCmdDrawIndexed(cmd, s.idxCount, 1, s.firstIdx, 0, 0);
    };

    // Ground
    push(glm::mat4(1.f));
    drawSlice(ground_);

    // Body (with heading, pitch, roll)
    glm::mat4 bodyM = glm::translate(glm::mat4(1.f), veh.position)
        * glm::rotate(glm::mat4(1.f), veh.heading, glm::vec3{0,1,0})
        * glm::rotate(glm::mat4(1.f), veh.pitch,   glm::vec3{1,0,0})
        * glm::rotate(glm::mat4(1.f), veh.roll,    glm::vec3{0,0,1});
    push(bodyM);
    drawSlice(body_);

    // 4 tire + rim combos + axles from mount to wheel
    for (int i = 0; i < 4; ++i) {
        glm::mat4 wheelT = glm::translate(glm::mat4(1.f), veh.wheelPos[i]);
        push(wheelT);
        drawSlice(tire_);
        push(wheelT);
        drawSlice(rim_);

        // Axle: thin cylinder from mountPos to wheelPos
        glm::vec3 mp = veh.mountPos[i];
        glm::vec3 wp = veh.wheelPos[i];
        glm::vec3 diff = wp - mp;
        float len = glm::length(diff);
        if (len > 0.001f) {
            glm::vec3 mid = (mp + wp) * 0.5f;
            glm::vec3 dir = diff / len;
            // Axle template is along X axis, half-length=0.5 => total=1.0
            // We need to rotate X-axis to point along 'dir' and scale to 'len'
            // Build a rotation that takes (1,0,0) to 'dir'
            glm::vec3 up{0,1,0};
            glm::vec3 right = glm::normalize(glm::cross(up, dir));
            glm::vec3 realUp = glm::cross(dir, right);
            glm::mat4 axleM = glm::translate(glm::mat4(1.f), mid);
            // Column-major rotation: col0=dir, col1=realUp, col2=right scaled by len
            axleM[0] = glm::vec4(dir * len, 0.f);
            axleM[1] = glm::vec4(realUp, 0.f);
            axleM[2] = glm::vec4(right, 0.f);
            push(axleM);
            drawSlice(axle_);
        }
    }

    // Bumps
    for (auto& b : bumps) {
        float halfX = (b.xMax - b.xMin) * 0.5f;
        float centerX = (b.xMax + b.xMin) * 0.5f;
        bool wide = halfX > 1.f;
        glm::mat4 bumpT = glm::translate(glm::mat4(1.f),
            glm::vec3{centerX, b.height * 0.5f, b.zCenter});
        push(bumpT);
        drawSlice(wide ? bumpWide_ : bumpNarrow_);
    }
}

void Renderer::drawHud(VkCommandBuffer cmd, uint32_t W, uint32_t H, const HudData& hud)
{
    // Build HUD geometry in screen-space pixels
    VList hudV;
    IList hudI;

    float fw = (float)W;
    float fh = (float)H;

    // --- Scenario buttons along the top ---
    float totalBtnW = hud.numScenarios * BTN_W + (hud.numScenarios - 1) * BTN_GAP;
    float btnX0 = (fw - totalBtnW) * 0.5f;

    for (int i = 0; i < hud.numScenarios; ++i) {
        float bx = btnX0 + i * (BTN_W + BTN_GAP);
        bool active = (i == hud.activeScenario);

        // Button background
        glm::vec4 bgCol = active ? glm::vec4{0.3f, 0.55f, 0.8f, 1.f}
                                 : glm::vec4{0.25f, 0.25f, 0.30f, 1.f};
        buildRect(bx, BTN_Y, BTN_W, BTN_H, bgCol, hudV, hudI);

        // Button text
        glm::vec4 txtCol = active ? glm::vec4{1.f, 1.f, 1.f, 1.f}
                                  : glm::vec4{0.7f, 0.7f, 0.7f, 1.f};
        if (hud.scenarioNames && hud.scenarioNames[i]) {
            // Center text in button
            const char* name = hud.scenarioNames[i];
            int len = 0; for (const char* p = name; *p; ++p) ++len;
            float textW = len * 6.f * 2.f;
            float tx = bx + (BTN_W - textW) * 0.5f;
            float ty = BTN_Y + (BTN_H - 7.f * 2.f) * 0.5f;
            buildText(name, tx, ty, 2.f, txtCol, hudV, hudI);
        }
    }

    // --- Speedometer (bottom-right area) ---
    {
        float panelX = fw - 200.f;
        float panelY = fh - 70.f;

        // "KM/H" label
        buildText("KM/H", panelX + 110.f, panelY + 4.f, 2.f,
                  {0.8f, 0.8f, 0.8f, 1.f}, hudV, hudI);

        // Speed number (large)
        char speedStr[16];
        int spd = (int)(std::abs(hud.speedKmh) + 0.5f);
        std::snprintf(speedStr, sizeof(speedStr), "%3d", spd);
        buildText(speedStr, panelX, panelY - 4.f, 4.f,
                  {1.f, 1.f, 1.f, 1.f}, hudV, hudI);
    }

    // --- RPM bar ---
    {
        float barX = fw - 200.f;
        float barY = fh - 30.f;
        float barW = 190.f;
        float barH = 14.f;

        // Background
        buildRect(barX, barY, barW, barH, {0.15f, 0.15f, 0.15f, 1.f}, hudV, hudI);

        // Fill
        float frac = std::clamp(hud.rpm / hud.rpmLimit, 0.f, 1.f);
        glm::vec4 barCol = (frac < 0.7f) ? glm::vec4{0.2f, 0.7f, 0.3f, 1.f}
                         : (frac < 0.9f) ? glm::vec4{0.9f, 0.8f, 0.1f, 1.f}
                                         : glm::vec4{0.9f, 0.2f, 0.1f, 1.f};
        if (frac > 0.01f)
            buildRect(barX + 2, barY + 2, (barW - 4) * frac, barH - 4, barCol, hudV, hudI);

        // "RPM" label
        buildText("RPM", barX + barW + 4.f, barY + 1.f, 2.f,
                  {0.8f, 0.8f, 0.8f, 1.f}, hudV, hudI);
    }

    if (hudV.empty()) return;

    // Upload HUD geometry
    {
        VkDeviceSize vSz = hudV.size() * sizeof(Vertex);
        void* p; vkMapMemory(dev_, hudVmem_, 0, vSz, 0, &p);
        std::memcpy(p, hudV.data(), vSz);
        vkUnmapMemory(dev_, hudVmem_);
    }
    {
        VkDeviceSize iSz = hudI.size() * sizeof(uint32_t);
        void* p; vkMapMemory(dev_, hudImem_, 0, iSz, 0, &p);
        std::memcpy(p, hudI.data(), iSz);
        vkUnmapMemory(dev_, hudImem_);
    }

    // Set full-screen viewport
    VkViewport vp{};
    vp.width    = fw;
    vp.height   = fh;
    vp.minDepth = 0.f;
    vp.maxDepth = 1.f;
    vkCmdSetViewport(cmd, 0, 1, &vp);

    VkRect2D sc{};
    sc.extent = { W, H };
    vkCmdSetScissor(cmd, 0, 1, &sc);

    // Ortho projection: (0,0) top-left, (W,H) bottom-right
    // In Vulkan clip space, Y=-1 is top and Y=+1 is bottom.
    // glm::ortho(0,W,0,H) maps Y=0 to -1 (top) and Y=H to +1 (bottom).
    // No Y-flip needed — this IS screen coordinates for Vulkan.
    glm::mat4 ortho = glm::ortho(0.f, fw, 0.f, fh, -1.f, 1.f);
    // Remap depth [-1,1] to [0,1]
    for (int col = 0; col < 4; ++col)
        ortho[col][2] = 0.5f * ortho[col][2] + 0.5f * ortho[col][3];

    PushConst pc{ ortho, glm::mat4(1.f) };
    vkCmdPushConstants(cmd, layout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);

    VkDeviceSize off = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &hudVbuf_, &off);
    vkCmdBindIndexBuffer(cmd, hudIbuf_, 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(cmd, (uint32_t)hudI.size(), 1, 0, 0, 0);
}

void Renderer::draw(VkCommandBuffer cmd, uint32_t W, uint32_t H,
                    const Vehicle& veh, const HudData& hud,
                    const std::vector<Bump>& bumps)
{
    VkDeviceSize off = 0;

    // Reserve a strip at the top for HUD buttons
    constexpr uint32_t HUD_STRIP = 40;
    uint32_t hw = W / 2;
    uint32_t viewH = H - HUD_STRIP;
    uint32_t hh = viewH / 2;
    constexpr uint32_t GAP = 2;

    float h     = veh.heading;
    glm::vec3 pos = veh.position;
    glm::vec3 fwd   { std::sin(h), 0.f, std::cos(h) };
    glm::vec3 right { std::cos(h), 0.f, -std::sin(h) };

    auto fixDepth = [](glm::mat4& p) {
        for (int col = 0; col < 4; ++col)
            p[col][2] = 0.5f * p[col][2] + 0.5f * p[col][3];
    };
    auto vkOrtho = [&](float l, float r, float b, float t, float n, float f) {
        glm::mat4 p = glm::ortho(l, r, b, t, n, f);
        p[1][1] *= -1.f;
        fixDepth(p);
        return p;
    };
    auto vkPersp = [&](float fovRad, float aspect, float n, float f) {
        glm::mat4 p = glm::perspective(fovRad, aspect, n, f);
        p[1][1] *= -1.f;
        fixDepth(p);
        return p;
    };

    float range = 3.5f;
    float aspQ  = (float)(hw - GAP) / (float)(hh - GAP);

    struct Quad { uint32_t x, y, w, h; glm::mat4 vp; };
    Quad quads[4];

    glm::vec3 lookTarget = pos + glm::vec3{0, 0.3f, 0};

    // Top-left: BEHIND view (below HUD strip)
    quads[0] = { 0, HUD_STRIP, hw - GAP, hh - GAP,
        vkOrtho(-range*aspQ, range*aspQ, -range*0.6f, range*1.0f, 0.1f, 100.f) *
        glm::lookAt(pos - fwd * 10.f + glm::vec3{0, 3.f, 0},
                     lookTarget, glm::vec3{0, 1, 0})
    };

    // Top-right: SIDE view
    quads[1] = { hw + GAP, HUD_STRIP, hw - GAP, hh - GAP,
        vkOrtho(-range*aspQ, range*aspQ, -range*0.6f, range*1.0f, 0.1f, 100.f) *
        glm::lookAt(pos + right * 10.f + glm::vec3{0, 3.f, 0},
                     lookTarget, glm::vec3{0, 1, 0})
    };

    // Bottom-left: TOP view
    quads[2] = { 0, HUD_STRIP + hh + GAP, hw - GAP, hh - GAP,
        vkOrtho(-range*aspQ, range*aspQ, -range, range, 0.1f, 100.f) *
        glm::lookAt(pos + glm::vec3{0, 50.f, 0}, pos, fwd)
    };

    // Bottom-right: PERSPECTIVE
    {
        glm::vec3 camOff = glm::normalize(-fwd + right + glm::vec3{0,1,0}) * 6.f;
        quads[3] = { hw + GAP, HUD_STRIP + hh + GAP, hw - GAP, hh - GAP,
            vkPersp(glm::radians(45.f), aspQ, 0.1f, 300.f) *
            glm::lookAt(pos + camOff, lookTarget, glm::vec3{0, 1, 0})
        };
    }

    for (auto& q : quads) {
        VkViewport vp{};
        vp.x        = (float)q.x;
        vp.y        = (float)q.y;
        vp.width    = (float)q.w;
        vp.height   = (float)q.h;
        vp.minDepth = 0.f;
        vp.maxDepth = 1.f;
        vkCmdSetViewport(cmd, 0, 1, &vp);

        VkRect2D sc{};
        sc.offset = { (int32_t)q.x, (int32_t)q.y };
        sc.extent = { q.w, q.h };
        vkCmdSetScissor(cmd, 0, 1, &sc);

        // Sky background
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipeline_);
        vkCmdDraw(cmd, 3, 1, 0, 0);

        // Scene on top
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbuf_, &off);
        vkCmdBindIndexBuffer(cmd, ibuf_, 0, VK_INDEX_TYPE_UINT32);
        drawScene(cmd, q.vp, veh, bumps);
    }

    // HUD overlay (full-screen, on top of everything)
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
    drawHud(cmd, W, H, hud);
}

int Renderer::hitTestButton(int mouseX, int mouseY, uint32_t W, uint32_t /*H*/,
                            int numScenarios) const
{
    float fw = (float)W;
    float totalBtnW = numScenarios * BTN_W + (numScenarios - 1) * BTN_GAP;
    float btnX0 = (fw - totalBtnW) * 0.5f;

    for (int i = 0; i < numScenarios; ++i) {
        float bx = btnX0 + i * (BTN_W + BTN_GAP);
        if (mouseX >= bx && mouseX <= bx + BTN_W &&
            mouseY >= BTN_Y && mouseY <= BTN_Y + BTN_H) {
            return i;
        }
    }
    return -1;
}

// ---- shutdown --------------------------------------------------------------

void Renderer::shutdown(VkDevice dev)
{
    vkDestroyBuffer(dev, hudIbuf_, nullptr);
    vkFreeMemory   (dev, hudImem_, nullptr);
    vkDestroyBuffer(dev, hudVbuf_, nullptr);
    vkFreeMemory   (dev, hudVmem_, nullptr);
    vkDestroyBuffer(dev, ibuf_, nullptr);
    vkFreeMemory   (dev, imem_, nullptr);
    vkDestroyBuffer(dev, vbuf_, nullptr);
    vkFreeMemory   (dev, vmem_, nullptr);
    vkDestroyPipeline      (dev, skyPipeline_, nullptr);
    vkDestroyPipelineLayout(dev, skyLayout_,   nullptr);
    vkDestroyPipeline      (dev, pipeline_, nullptr);
    vkDestroyPipelineLayout(dev, layout_,   nullptr);
}
