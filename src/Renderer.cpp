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

// Cosine-profiled bump mesh: unit template [-1,1] in X and Z, [0,1] in Y.
// Y follows 0.5*(1+cos(pi*z)) so it peaks at center (z=0) and fades to 0 at edges.
// Scaled per-instance to actual bump dimensions via model matrix.
static void makeBumpMesh(int zSegs, glm::vec4 col, VList& verts, IList& idx)
{
    const float PI = 3.14159265f;
    uint32_t base = (uint32_t)verts.size();

    // Top surface: 2 columns (x=-1, x=1), zSegs+1 rows
    for (int j = 0; j <= zSegs; ++j) {
        float t = (float)j / zSegs;
        float z = -1.f + 2.f * t;
        float y = 0.5f * (1.f + std::cos(PI * z));
        float dydz = -0.5f * PI * std::sin(PI * z);
        glm::vec3 normal = glm::normalize(glm::vec3{0.f, 1.f, -dydz});
        verts.push_back({{-1.f, y, z}, normal, col});
        verts.push_back({{ 1.f, y, z}, normal, col});
    }
    for (int j = 0; j < zSegs; ++j) {
        uint32_t bl = base + j * 2;
        uint32_t br = bl + 1, tl = bl + 2, tr = bl + 3;
        idx.insert(idx.end(), {bl, br, tr, bl, tr, tl});
    }

    // Left side wall (x=-1, facing -X)
    uint32_t sBase = (uint32_t)verts.size();
    glm::vec3 leftN{-1.f, 0.f, 0.f};
    for (int j = 0; j <= zSegs; ++j) {
        float t = (float)j / zSegs;
        float z = -1.f + 2.f * t;
        float y = 0.5f * (1.f + std::cos(PI * z));
        verts.push_back({{-1.f, 0.f, z}, leftN, col});
        verts.push_back({{-1.f, y,   z}, leftN, col});
    }
    for (int j = 0; j < zSegs; ++j) {
        uint32_t b0 = sBase + j * 2, t0 = b0 + 1, b1 = b0 + 2, t1 = b0 + 3;
        idx.insert(idx.end(), {b0, b1, t1, b0, t1, t0});
    }

    // Right side wall (x=1, facing +X)
    sBase = (uint32_t)verts.size();
    glm::vec3 rightN{1.f, 0.f, 0.f};
    for (int j = 0; j <= zSegs; ++j) {
        float t = (float)j / zSegs;
        float z = -1.f + 2.f * t;
        float y = 0.5f * (1.f + std::cos(PI * z));
        verts.push_back({{1.f, 0.f, z}, rightN, col});
        verts.push_back({{1.f, y,   z}, rightN, col});
    }
    for (int j = 0; j < zSegs; ++j) {
        uint32_t b0 = sBase + j * 2, t0 = b0 + 1, b1 = b0 + 2, t1 = b0 + 3;
        idx.insert(idx.end(), {b1, b0, t0, b1, t0, t1});
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

// Build 3D world-space text lying flat on the ground (facing up, readable from above).
// Each pixel becomes a small quad on the XZ plane at the given Y height.
// Text flows along +X, with heading rotation around Y.
static void buildWorldText(const char* str, glm::vec3 origin, float heading,
                           float pixelSize, glm::vec4 col, VList& verts, IList& idx)
{
    glm::vec3 N{0.f, 1.f, 0.f};  // ground normal (up)
    float cs = std::cos(heading);
    float sn = std::sin(heading);
    // Text right = along heading direction in XZ
    glm::vec3 right{cs, 0.f, sn};
    glm::vec3 fwd{-sn, 0.f, cs};  // perpendicular in XZ (text "down" = forward)

    float cx = 0.f;
    for (const char* p = str; *p; ++p) {
        int ch = (int)(unsigned char)*p;
        if (ch < 32 || ch > 90) {
            if (ch >= 'a' && ch <= 'z') ch = ch - 'a' + 'A';
            else { cx += 6.f * pixelSize; continue; }
        }
        int gi = ch - 32;
        for (int row = 0; row < 7; ++row) {
            uint8_t bits = FONT_5x7[gi][row];
            for (int col_i = 0; col_i < 5; ++col_i) {
                if (bits & (0x10 >> col_i)) {
                    float lx = cx + col_i * pixelSize;
                    float lz = row * pixelSize;
                    // 4 corners of the pixel quad in world space
                    glm::vec3 p0 = origin + right * lx + fwd * lz;
                    glm::vec3 p1 = origin + right * (lx + pixelSize) + fwd * lz;
                    glm::vec3 p2 = origin + right * (lx + pixelSize) + fwd * (lz + pixelSize);
                    glm::vec3 p3 = origin + right * lx + fwd * (lz + pixelSize);
                    uint32_t b = (uint32_t)verts.size();
                    verts.push_back({p0, N, col});
                    verts.push_back({p1, N, col});
                    verts.push_back({p2, N, col});
                    verts.push_back({p3, N, col});
                    idx.insert(idx.end(), {b, b+1, b+2, b, b+2, b+3});
                }
            }
        }
        cx += 6.f * pixelSize;
    }
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
    cba.blendEnable         = VK_TRUE;
    cba.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    cba.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    cba.colorBlendOp        = VK_BLEND_OP_ADD;
    cba.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    cba.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    cba.alphaBlendOp        = VK_BLEND_OP_ADD;

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

    // Translucent pipeline: same as main but depth write OFF
    ds.depthWriteEnable = VK_FALSE;
    vkCreateGraphicsPipelines(dev, VK_NULL_HANDLE, 1, &gp, nullptr, &translucentPipe_);
    ds.depthWriteEnable = VK_TRUE;  // restore

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

    ground_ = record([](VList& v, IList& i){ makeGround(600.f, v, i); });

    glm::vec4 bodyCol { 0.15f, 0.45f, 0.85f, 0.55f };  // bright blue, translucent
    glm::vec4 rimCol  { 0.90f, 0.92f, 0.95f, 0.55f };  // white, translucent
    glm::vec4 tireCol { 0.15f, 0.15f, 0.15f, 0.55f };  // dark rubber, translucent

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
    glm::vec4 axleCol { 0.4f, 0.4f, 0.45f, 0.4f };  // dark steel, translucent
    axle_ = record([&](VList& v, IList& i){
        makeCylinder(0.015f, 0.5f, 8, axleCol, v, i);  // radius=1.5cm, half-length=0.5 (scaled)
    });

    // Subframe: flat box spanning the width between mount points
    glm::vec4 sfCol { 0.6f, 0.3f, 0.1f, 0.5f };  // rusty orange, translucent
    subframe_ = record([&](VList& v, IList& i){
        makeBox(Vehicle::BODY_HALF_W, 0.02f, 0.08f, sfCol, v, i);
    });

    // Cosine-profiled bump: unit template scaled per-instance
    glm::vec4 bumpCol { 0.85f, 0.75f, 0.15f, 1.f };
    unitBump_ = record([&](VList& v, IList& i){
        makeBumpMesh(16, bumpCol, v, i);
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

    // ---- Tire trail buffers (dynamic, updated each frame) ----
    trailVbuf_ = ctx.allocBuffer(TRAIL_MAX_VERTS * sizeof(Vertex),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        trailVmem_);
    trailIbuf_ = ctx.allocBuffer(TRAIL_MAX_IDX * sizeof(uint32_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        trailImem_);

    // ---- Ground label buffers (dynamic, updated each frame) ----
    labelVbuf_ = ctx.allocBuffer(LABEL_MAX_VERTS * sizeof(Vertex),
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        labelVmem_);
    labelIbuf_ = ctx.allocBuffer(LABEL_MAX_IDX * sizeof(uint32_t),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        labelImem_);

    std::printf("Meshes: %zu verts, %zu indices\n", allV.size(), allI.size());
    return true;
}

// ---- draw ------------------------------------------------------------------

void Renderer::drawScene(VkCommandBuffer cmd, const glm::mat4& vp, const Vehicle& veh,
                         const Playground& playground, bool hasTrails)
{
    auto push = [&](const glm::mat4& model) {
        PushConst pc{ vp, model };
        vkCmdPushConstants(cmd, layout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
    };
    auto drawSlice = [&](const Slice& s) {
        vkCmdDrawIndexed(cmd, s.idxCount, 1, s.firstIdx, 0, 0);
    };

    // === OPAQUE PASS (depth write ON) ===

    // Ground
    push(glm::mat4(1.f));
    drawSlice(ground_);

    // Tire trails (on ground)
    if (hasTrails && trailIdxCount_ > 0) {
        push(glm::mat4(1.f));
        VkDeviceSize trailOff = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &trailVbuf_, &trailOff);
        vkCmdBindIndexBuffer(cmd, trailIbuf_, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, trailIdxCount_, 1, 0, 0, 0);

        // Re-bind scene buffers
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbuf_, &trailOff);
        vkCmdBindIndexBuffer(cmd, ibuf_, 0, VK_INDEX_TYPE_UINT32);
    }

    // Ground labels (on ground, before vehicle so they're under it)
    if (labelIdxCount_ > 0) {
        push(glm::mat4(1.f));
        VkDeviceSize labelOff = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &labelVbuf_, &labelOff);
        vkCmdBindIndexBuffer(cmd, labelIbuf_, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, labelIdxCount_, 1, 0, 0, 0);

        // Re-bind scene buffers
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbuf_, &labelOff);
        vkCmdBindIndexBuffer(cmd, ibuf_, 0, VK_INDEX_TYPE_UINT32);
    }

    // Wheels, rims, and axles (draw before translucent body so they're visible through it)
    for (int i = 0; i < 4; ++i) {
        glm::mat4 wheelRot = glm::mat4(veh.bodyRotation);
        if (i < 2) {
            glm::vec3 bodyUp = veh.bodyRotation[1];
            wheelRot = glm::rotate(wheelRot, veh.frontSteerAngle, bodyUp);
        }
        glm::mat4 wheelT = glm::translate(glm::mat4(1.f), veh.wheelPos[i])
            * wheelRot;
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
            glm::vec3 up{0,1,0};
            glm::vec3 right = glm::normalize(glm::cross(up, dir));
            glm::vec3 realUp = glm::cross(dir, right);
            glm::mat4 axleM = glm::translate(glm::mat4(1.f), mid);
            axleM[0] = glm::vec4(dir * len, 0.f);
            axleM[1] = glm::vec4(realUp, 0.f);
            axleM[2] = glm::vec4(right, 0.f);
            push(axleM);
            drawSlice(axle_);
        }
    }

    // Bumps — unit mesh scaled and rotated to actual dimensions
    for (auto& b : playground.bumps) {
        glm::mat4 bumpT = glm::translate(glm::mat4(1.f),
            glm::vec3{b.xCenter, 0.f, b.zCenter})
            * glm::rotate(glm::mat4(1.f), b.heading, glm::vec3{0.f, 1.f, 0.f})
            * glm::scale(glm::mat4(1.f),
            glm::vec3{b.halfWidth, b.height, b.halfLength});
        push(bumpT);
        drawSlice(unitBump_);
    }

    // === TRANSLUCENT PASS (depth write OFF — so wheels/axles show through body) ===
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, translucentPipe_);

    // Body
    glm::mat4 bodyM = glm::translate(glm::mat4(1.f), veh.position)
        * glm::mat4(veh.bodyRotation);
    push(bodyM);
    drawSlice(body_);

    // Subframes
    {
        glm::vec3 fMid = (veh.mountPos[0] + veh.mountPos[1]) * 0.5f;
        glm::mat4 sfFront = glm::translate(glm::mat4(1.f), fMid)
            * glm::mat4(veh.bodyRotation);
        push(sfFront);
        drawSlice(subframe_);

        glm::vec3 rMid = (veh.mountPos[2] + veh.mountPos[3]) * 0.5f;
        glm::mat4 sfRear = glm::translate(glm::mat4(1.f), rMid)
            * glm::mat4(veh.bodyRotation);
        push(sfRear);
        drawSlice(subframe_);
    }
}

void Renderer::drawHud(VkCommandBuffer cmd, uint32_t W, uint32_t H, const HudData& hud)
{
    // Build HUD geometry in screen-space pixels
    VList hudV;
    IList hudI;

    float fw = (float)W;
    float fh = (float)H;

    // --- Speedometer (right side) ---
    {
        char speedStr[16];
        int spd = (int)(std::abs(hud.speedKmh) + 0.5f);
        std::snprintf(speedStr, sizeof(speedStr), "%3d", spd);
        float sx = fw - 190.f;
        buildText(speedStr, sx, 4.f, 3.f, {1.f, 1.f, 1.f, 1.f}, hudV, hudI);
        buildText("KM/H", sx + 60.f, 10.f, 2.f, {0.7f, 0.7f, 0.7f, 1.f}, hudV, hudI);
    }

    // --- Gear indicator ---
    {
        char gearStr[8];
        std::snprintf(gearStr, sizeof(gearStr), "G%d", hud.gear);
        buildText(gearStr, fw - 120.f, 10.f, 2.f, {0.9f, 0.9f, 0.3f, 1.f}, hudV, hudI);
    }

    // --- RPM bar ---
    {
        float barX = fw - 190.f;
        float barY = 28.f;
        float barW = 180.f;
        float barH = 8.f;

        buildRect(barX, barY, barW, barH, {0.15f, 0.15f, 0.15f, 1.f}, hudV, hudI);

        float frac = std::clamp(hud.rpm / hud.rpmLimit, 0.f, 1.f);
        glm::vec4 barCol = (frac < 0.7f) ? glm::vec4{0.2f, 0.7f, 0.3f, 1.f}
                         : (frac < 0.9f) ? glm::vec4{0.9f, 0.8f, 0.1f, 1.f}
                                         : glm::vec4{0.9f, 0.2f, 0.1f, 1.f};
        if (frac > 0.01f)
            buildRect(barX + 1, barY + 1, (barW - 2) * frac, barH - 2, barCol, hudV, hudI);
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

    glm::mat4 ortho = glm::ortho(0.f, fw, 0.f, fh, -1.f, 1.f);
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
                    const Playground& playground,
                    const TrailGeometry* trails)
{
    VkDeviceSize off = 0;

    // Upload tire trail geometry if provided
    trailIdxCount_ = 0;
    if (trails && trails->idxCount > 0 && trails->vertCount > 0) {
        uint32_t vc = std::min(trails->vertCount, TRAIL_MAX_VERTS);
        uint32_t ic = std::min(trails->idxCount, TRAIL_MAX_IDX);
        {
            VkDeviceSize vSz = vc * sizeof(Vertex);
            void* p; vkMapMemory(dev_, trailVmem_, 0, vSz, 0, &p);
            std::memcpy(p, trails->verts, vSz);
            vkUnmapMemory(dev_, trailVmem_);
        }
        {
            VkDeviceSize iSz = ic * sizeof(uint32_t);
            void* p; vkMapMemory(dev_, trailImem_, 0, iSz, 0, &p);
            std::memcpy(p, trails->indices, iSz);
            vkUnmapMemory(dev_, trailImem_);
        }
        trailIdxCount_ = ic;
    }

    // Build and upload ground label geometry
    labelIdxCount_ = 0;
    if (!playground.labels.empty()) {
        VList labelV;
        IList labelI;
        glm::vec4 labelCol{1.f, 1.f, 1.f, 0.9f};  // white, slightly transparent
        float pixelSize = 0.12f;  // 12cm per pixel — readable from above

        for (auto& lbl : playground.labels) {
            buildWorldText(lbl.text, lbl.position, lbl.heading, pixelSize, labelCol, labelV, labelI);
        }

        uint32_t vc = std::min((uint32_t)labelV.size(), LABEL_MAX_VERTS);
        uint32_t ic = std::min((uint32_t)labelI.size(), LABEL_MAX_IDX);
        if (vc > 0 && ic > 0) {
            {
                VkDeviceSize vSz = vc * sizeof(Vertex);
                void* p; vkMapMemory(dev_, labelVmem_, 0, vSz, 0, &p);
                std::memcpy(p, labelV.data(), vSz);
                vkUnmapMemory(dev_, labelVmem_);
            }
            {
                VkDeviceSize iSz = ic * sizeof(uint32_t);
                void* p; vkMapMemory(dev_, labelImem_, 0, iSz, 0, &p);
                std::memcpy(p, labelI.data(), iSz);
                vkUnmapMemory(dev_, labelImem_);
            }
            labelIdxCount_ = ic;
        }
    }

    // No HUD strip needed since scenario buttons are gone
    uint32_t hw = W / 2;
    uint32_t hh = H / 2;
    constexpr uint32_t GAP = 2;

    glm::vec3 pos = veh.position;
    glm::vec3 right = veh.bodyRotation[0];
    glm::vec3 up    = veh.bodyRotation[1];
    glm::vec3 fwd   = veh.bodyRotation[2];

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

    glm::vec3 lookTarget = pos;

    auto fixMirror = [](glm::mat4 vp) {
        for (int c = 0; c < 4; ++c) vp[c][0] = -vp[c][0];
        return vp;
    };

    // Top-left: BEHIND view
    quads[0] = { 0, 0, hw - GAP, hh - GAP,
        fixMirror(vkOrtho(-range*aspQ, range*aspQ, -range, range, 0.1f, 100.f) *
        glm::lookAt(pos - fwd * 10.f,
                     lookTarget, up))
    };

    // Top-right: SIDE view
    quads[1] = { hw + GAP, 0, hw - GAP, hh - GAP,
        fixMirror(vkOrtho(-range*aspQ, range*aspQ, -range, range, 0.1f, 100.f) *
        glm::lookAt(pos + right * 10.f,
                     lookTarget, up))
    };

    // Bottom-left: CHASE CAM — low angle behind the car
    {
        glm::vec3 chaseCam = pos - fwd * 5.f + glm::vec3{0.f, 1.8f, 0.f};
        glm::vec3 chaseTgt = pos + fwd * 4.f + glm::vec3{0.f, 0.5f, 0.f};
        quads[2] = { 0, hh + GAP, hw - GAP, hh - GAP,
            fixMirror(vkPersp(glm::radians(55.f), aspQ, 0.1f, 500.f) *
            glm::lookAt(chaseCam, chaseTgt, glm::vec3{0.f, 1.f, 0.f}))
        };
    }

    // Bottom-right: PERSPECTIVE
    {
        glm::vec3 camOff = glm::normalize(-fwd + right + up) * 6.f;
        quads[3] = { hw + GAP, hh + GAP, hw - GAP, hh - GAP,
            fixMirror(vkPersp(glm::radians(45.f), aspQ, 0.1f, 300.f) *
            glm::lookAt(pos + camOff, lookTarget, up))
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
        drawScene(cmd, q.vp, veh, playground, trailIdxCount_ > 0);
    }

    // HUD overlay (full-screen, on top of everything)
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
    drawHud(cmd, W, H, hud);
}

// ---- shutdown --------------------------------------------------------------

void Renderer::shutdown(VkDevice dev)
{
    vkDestroyBuffer(dev, labelIbuf_, nullptr);
    vkFreeMemory   (dev, labelImem_, nullptr);
    vkDestroyBuffer(dev, labelVbuf_, nullptr);
    vkFreeMemory   (dev, labelVmem_, nullptr);
    vkDestroyBuffer(dev, trailIbuf_, nullptr);
    vkFreeMemory   (dev, trailImem_, nullptr);
    vkDestroyBuffer(dev, trailVbuf_, nullptr);
    vkFreeMemory   (dev, trailVmem_, nullptr);
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
    vkDestroyPipeline      (dev, translucentPipe_, nullptr);
    vkDestroyPipeline      (dev, pipeline_, nullptr);
    vkDestroyPipelineLayout(dev, layout_,   nullptr);
}
