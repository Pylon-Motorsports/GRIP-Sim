#include "Renderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <fstream>
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
//
// All triangles use CCW winding when viewed from outside the surface.
// The pipeline uses VK_FRONT_FACE_CLOCKWISE to account for the Vulkan Y-flip
// in the projection matrix (proj[1][1] *= -1).

using VList = std::vector<Vertex>;
using IList = std::vector<uint32_t>;

static void makeBox(float hx, float hy, float hz, glm::vec4 col,
                    VList& verts, IList& idx)
{
    // 6 faces, 4 verts each.  Vertices listed CCW when viewed from outside.
    auto face = [&](glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 n) {
        uint32_t b = (uint32_t)verts.size();
        verts.push_back({ p0, n, col });
        verts.push_back({ p1, n, col });
        verts.push_back({ p2, n, col });
        verts.push_back({ p3, n, col });
        idx.insert(idx.end(), { b, b+1, b+2, b, b+2, b+3 });
    };
    face({-hx,-hy, hz}, { hx,-hy, hz}, { hx, hy, hz}, {-hx, hy, hz}, { 0, 0, 1}); // +Z
    face({ hx,-hy,-hz}, {-hx,-hy,-hz}, {-hx, hy,-hz}, { hx, hy,-hz}, { 0, 0,-1}); // -Z
    face({ hx,-hy, hz}, { hx,-hy,-hz}, { hx, hy,-hz}, { hx, hy, hz}, { 1, 0, 0}); // +X
    face({-hx,-hy,-hz}, {-hx,-hy, hz}, {-hx, hy, hz}, {-hx, hy,-hz}, {-1, 0, 0}); // -X
    face({-hx, hy, hz}, { hx, hy, hz}, { hx, hy,-hz}, {-hx, hy,-hz}, { 0, 1, 0}); // +Y
    face({-hx,-hy,-hz}, { hx,-hy,-hz}, { hx,-hy, hz}, {-hx,-hy, hz}, { 0,-1, 0}); // -Y
}

static void makeCylinder(float radius, float halfW, int segs, glm::vec4 col,
                         VList& verts, IList& idx)
{
    // Axis along X.  Left cap at x=-halfW, right cap at x=+halfW.
    const float TAU = 6.283185307f;

    uint32_t cL = (uint32_t)verts.size();
    verts.push_back({{ -halfW, 0, 0 }, { -1, 0, 0 }, col });
    uint32_t cR = (uint32_t)verts.size();
    verts.push_back({{  halfW, 0, 0 }, {  1, 0, 0 }, col });

    // Per-segment: capLeft, capRight, sideLeft, sideRight
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

        // Left cap:  CCW from -X viewpoint  →  center, ring[j], ring[i]
        idx.insert(idx.end(), { cL, rings[j].capL, rings[i].capL });

        // Right cap: CCW from +X viewpoint  →  center, ring[i], ring[j]
        idx.insert(idx.end(), { cR, rings[i].capR, rings[j].capR });

        // Side quad: CCW from outside
        // Two triangles: (sideL_i, sideL_j, sideR_j) and (sideL_i, sideR_j, sideR_i)
        idx.insert(idx.end(), {
            rings[i].sideL, rings[j].sideL, rings[j].sideR,
            rings[i].sideL, rings[j].sideR, rings[i].sideR });
    }
}

static void makeGround(float halfSize, VList& verts, IList& idx)
{
    // Quad at Y=0, normal +Y.  Alpha < 0.5 triggers checkerboard in shader.
    glm::vec4 col{ 0.4f, 0.45f, 0.3f, 0.f };
    glm::vec3 n{ 0, 1, 0 };
    float s = halfSize;
    // CCW from above (+Y): cross((s,0,s)-(-s,0,s), (s,0,-s)-(-s,0,s)) = +Y
    uint32_t b = (uint32_t)verts.size();
    verts.push_back({{ -s, 0,  s }, n, col });
    verts.push_back({{  s, 0,  s }, n, col });
    verts.push_back({{  s, 0, -s }, n, col });
    verts.push_back({{ -s, 0, -s }, n, col });
    idx.insert(idx.end(), { b, b+1, b+2, b, b+2, b+3 });
}

// ---- init ------------------------------------------------------------------

bool Renderer::init(const VulkanContext& ctx)
{
    VkDevice dev = ctx.device;

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
    rs.cullMode    = VK_CULL_MODE_BACK_BIT;
    rs.frontFace   = VK_FRONT_FACE_CLOCKWISE;  // accounts for Vulkan Y-flip
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

    // ---- Generate meshes ----
    // All indices are absolute — no vertex offset needed in draw calls.
    VList allV;
    IList allI;

    auto record = [&](auto fn) -> Slice {
        uint32_t iOff = (uint32_t)allI.size();
        fn(allV, allI);
        return { iOff, (uint32_t)allI.size() - iOff };
    };

    ground_ = record([](VList& v, IList& i){ makeGround(100.f, v, i); });

    glm::vec4 bodyCol  { 0.25f, 0.55f, 0.30f, 1.f };  // green
    glm::vec4 wheelCol { 0.20f, 0.20f, 0.20f, 1.f };  // dark gray

    body_  = record([&](VList& v, IList& i){
        makeBox(Vehicle::BODY_HALF_W, Vehicle::BODY_HALF_H, Vehicle::BODY_HALF_L, bodyCol, v, i);
    });
    wheel_ = record([&](VList& v, IList& i){
        makeCylinder(Vehicle::WHEEL_RADIUS, Vehicle::WHEEL_HALF_W, 16, wheelCol, v, i);
    });

    // ---- Upload ----
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

    std::printf("Meshes: %zu verts, %zu indices\n", allV.size(), allI.size());
    return true;
}

// ---- draw ------------------------------------------------------------------

void Renderer::drawScene(VkCommandBuffer cmd, const glm::mat4& vp, const Vehicle& veh)
{
    auto push = [&](const glm::mat4& model) {
        PushConst pc{ vp, model };
        vkCmdPushConstants(cmd, layout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
    };
    auto drawSlice = [&](const Slice& s) {
        vkCmdDrawIndexed(cmd, s.idxCount, 1, s.firstIdx, 0, 0);
    };

    float h = veh.heading;
    glm::mat4 vehicleT = glm::translate(glm::mat4(1.f), veh.position)
                        * glm::rotate(glm::mat4(1.f), h, glm::vec3{0,1,0});

    // Ground
    push(glm::mat4(1.f));
    drawSlice(ground_);

    // Body (position is the CG, already in veh.position)
    push(glm::translate(glm::mat4(1.f), veh.position)
       * glm::rotate(glm::mat4(1.f), veh.heading, glm::vec3{0,1,0}));
    drawSlice(body_);

    // 4 wheels (world positions from physics)
    for (int i = 0; i < 4; ++i) {
        push(glm::translate(glm::mat4(1.f), veh.wheelPos[i]));
        drawSlice(wheel_);
    }
}

void Renderer::draw(VkCommandBuffer cmd, uint32_t W, uint32_t H, const Vehicle& veh)
{
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
    VkDeviceSize off = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vbuf_, &off);
    vkCmdBindIndexBuffer(cmd, ibuf_, 0, VK_INDEX_TYPE_UINT32);

    uint32_t hw = W / 2;
    uint32_t hh = H / 2;
    constexpr uint32_t GAP = 2;

    float h     = veh.heading;
    glm::vec3 pos = veh.position;
    glm::vec3 fwd   { std::sin(h), 0.f, std::cos(h) };
    glm::vec3 right { std::cos(h), 0.f, -std::sin(h) };

    // Vulkan projection wrappers: Y-flip + remap depth from [-1,1] to [0,1]
    auto fixDepth = [](glm::mat4& p) {
        // Maps clip Z from [-1,1] (OpenGL) to [0,1] (Vulkan)
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

    float range = 8.f;
    float aspQ  = (float)(hw - GAP) / (float)(hh - GAP);  // quadrant aspect

    struct Quad { uint32_t x, y, w, h; glm::mat4 vp; };
    Quad quads[4];

    // Top-left: BEHIND view (ortho, looking at the rear of the car)
    quads[0] = { 0, 0, hw - GAP, hh - GAP,
        vkOrtho(-range*aspQ, range*aspQ, -range*0.4f, range*1.2f, 0.1f, 100.f) *
        glm::lookAt(pos - fwd * 15.f + glm::vec3{0, 2.f, 0},
                     pos + glm::vec3{0, 0.5f, 0},
                     glm::vec3{0, 1, 0})
    };

    // Top-right: SIDE view (ortho, from the right)
    quads[1] = { hw + GAP, 0, hw - GAP, hh - GAP,
        vkOrtho(-range*aspQ, range*aspQ, -range*0.4f, range*1.2f, 0.1f, 100.f) *
        glm::lookAt(pos + right * 15.f + glm::vec3{0, 2.f, 0},
                     pos + glm::vec3{0, 0.5f, 0},
                     glm::vec3{0, 1, 0})
    };

    // Bottom-left: TOP view (ortho, looking straight down)
    quads[2] = { 0, hh + GAP, hw - GAP, hh - GAP,
        vkOrtho(-range*aspQ, range*aspQ, -range, range, 0.1f, 100.f) *
        glm::lookAt(pos + glm::vec3{0, 50.f, 0},
                     pos,
                     fwd)
    };

    // Bottom-right: PERSPECTIVE (45° behind, beside, above)
    {
        glm::vec3 camOff = glm::normalize(-fwd + right + glm::vec3{0,1,0}) * 12.f;
        quads[3] = { hw + GAP, hh + GAP, hw - GAP, hh - GAP,
            vkPersp(glm::radians(50.f), aspQ, 0.1f, 300.f) *
            glm::lookAt(pos + camOff,
                         pos + glm::vec3{0, 0.3f, 0},
                         glm::vec3{0, 1, 0})
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

        drawScene(cmd, q.vp, veh);
    }
}

// ---- shutdown --------------------------------------------------------------

void Renderer::shutdown(VkDevice dev)
{
    vkDestroyBuffer(dev, ibuf_, nullptr);
    vkFreeMemory   (dev, imem_, nullptr);
    vkDestroyBuffer(dev, vbuf_, nullptr);
    vkFreeMemory   (dev, vmem_, nullptr);
    vkDestroyPipeline      (dev, pipeline_, nullptr);
    vkDestroyPipelineLayout(dev, layout_,   nullptr);
}
