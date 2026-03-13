#include "Renderer.hpp"
#include "Scenario.hpp"
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

// Car body: base box with chamfered corners + trapezoid cabin.
// Coordinates in body-local space (drawn at BODY_Y offset).
// This shape is also the collision volume (see Body::colliderCorners).
//
// Base box: full car width (covers tires), floor to beltline,
//           front/rear bumper overhangs, chamfered lower corners.
// Cabin:    windshield slopes from A-pillar to roof, rear window slopes
//           from roof to rear of body.  Sides taper inward so roof
//           is narrower than body.
static void makeCarBody(glm::vec4 col, VList& verts, IList& idx)
{
    // --- Dimensions (body-local coords) ---
    constexpr float W     = 0.88f;   // half-width
    constexpr float bot   = -0.25f;  // floor pan
    constexpr float belt  =  0.30f;  // beltline
    constexpr float front =  1.95f;  // front bumper
    constexpr float rear  = -1.85f;  // rear bumper
    constexpr float C     =  0.15f;  // corner chamfer
    constexpr float B     =  0.12f;  // bottom bevel height

    // Cabin
    constexpr float roofY     = 0.75f;
    constexpr float roofHW    = 0.72f;
    constexpr float beltHW    = 0.85f;
    constexpr float aZ        = 0.85f;   // A-pillar
    constexpr float wsTopZ    = 0.35f;   // windshield top
    constexpr float roofRearZ = -0.85f;  // roof rear edge
    constexpr float rwBaseZ   = -1.30f;  // rear window base

    // Wheel arches — match physics subframe attachment Z positions
    constexpr float archR = 0.42f;
    constexpr float fwZ   = 1.35f;   // front subframe Z
    constexpr float rwZ   = -1.25f;  // rear subframe Z
    constexpr int   ARCH_SEGS = 8;
    constexpr float PI = 3.14159265f;

    auto face = [&](glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3,
                    glm::vec3 n, glm::vec4 c) {
        uint32_t b = (uint32_t)verts.size();
        verts.push_back({ p0, n, c });
        verts.push_back({ p1, n, c });
        verts.push_back({ p2, n, c });
        verts.push_back({ p3, n, c });
        idx.insert(idx.end(), { b, b+1, b+2, b, b+2, b+3 });
    };

    auto tri = [&](glm::vec3 p0, glm::vec3 p1, glm::vec3 p2,
                   glm::vec3 n, glm::vec4 c) {
        uint32_t b = (uint32_t)verts.size();
        verts.push_back({ p0, n, c });
        verts.push_back({ p1, n, c });
        verts.push_back({ p2, n, c });
        idx.insert(idx.end(), { b, b+1, b+2 });
    };

    // Plan-view octagon outline
    float ox[8] = { -(W-C), (W-C),  W,       W,      (W-C), -(W-C), -W,      -W     };
    float oz[8] = {  front, front,  front-C, rear+C,  rear,  rear,   rear+C,  front-C };
    float by[8] = { bot+B, bot+B, bot, bot, bot+B, bot+B, bot, bot };

    glm::vec4 bodyCol = col;

    // --- Side faces: 6 short quads (skip right i=2, left i=6 — handled with arches) ---
    for (int i = 0; i < 8; ++i) {
        if (i == 2 || i == 6) continue;
        int j = (i + 1) % 8;
        float dx = ox[j] - ox[i];
        float dz = oz[j] - oz[i];
        float len = std::sqrt(dx*dx + dz*dz);
        glm::vec3 n = (len > 0.001f) ? glm::vec3(dz/len, 0.f, -dx/len) : glm::vec3(0.f);
        face({ox[i], by[i], oz[i]}, {ox[j], by[j], oz[j]},
             {ox[j], belt, oz[j]}, {ox[i], belt, oz[i]}, n, bodyCol);
    }

    // --- Right side with wheel arches ---
    // Arch curve: y = bot + archR * sin(π*t), z = wheelZ - archR + 2*archR*t
    // Right side traverses front-to-rear (z decreasing), so t goes 1→0
    {
        glm::vec3 n {1, 0, 0};
        auto panel = [&](float z0, float z1) {
            face({W, bot, z0}, {W, bot, z1}, {W, belt, z1}, {W, belt, z0}, n, bodyCol);
        };
        auto arch = [&](float wheelZ) {
            for (int s = 0; s < ARCH_SEGS; ++s) {
                float t0 = 1.f - (float)s / ARCH_SEGS;
                float t1 = 1.f - (float)(s+1) / ARCH_SEGS;
                float z0 = wheelZ - archR + 2*archR*t0;
                float z1 = wheelZ - archR + 2*archR*t1;
                float y0 = bot + archR * std::sin(PI * t0);
                float y1 = bot + archR * std::sin(PI * t1);
                face({W, y0, z0}, {W, y1, z1}, {W, belt, z1}, {W, belt, z0}, n, bodyCol);
            }
        };
        panel(front-C, fwZ+archR);   // front fender
        arch(fwZ);                    // front wheel arch
        panel(fwZ-archR, rwZ+archR);  // door
        arch(rwZ);                    // rear wheel arch
        panel(rwZ-archR, rear+C);     // rear quarter
    }

    // --- Left side with wheel arches (mirror: z increasing, rear to front) ---
    {
        glm::vec3 n {-1, 0, 0};
        auto panel = [&](float z0, float z1) {
            face({-W, bot, z0}, {-W, bot, z1}, {-W, belt, z1}, {-W, belt, z0}, n, bodyCol);
        };
        auto arch = [&](float wheelZ) {
            for (int s = 0; s < ARCH_SEGS; ++s) {
                float t0 = (float)s / ARCH_SEGS;
                float t1 = (float)(s+1) / ARCH_SEGS;
                float z0 = wheelZ - archR + 2*archR*t0;
                float z1 = wheelZ - archR + 2*archR*t1;
                float y0 = bot + archR * std::sin(PI * t0);
                float y1 = bot + archR * std::sin(PI * t1);
                face({-W, y0, z0}, {-W, y1, z1}, {-W, belt, z1}, {-W, belt, z0}, n, bodyCol);
            }
        };
        panel(rear+C, rwZ-archR);     // rear quarter
        arch(rwZ);                     // rear wheel arch
        panel(rwZ+archR, fwZ-archR);   // door
        arch(fwZ);                     // front wheel arch
        panel(fwZ+archR, front-C);     // front fender
    }

    // --- Bottom face (octagon fan) ---
    {
        uint32_t center = (uint32_t)verts.size();
        verts.push_back({{0.f, bot, 0.f}, {0,-1,0}, bodyCol});
        uint32_t ring[8];
        for (int i = 0; i < 8; ++i) {
            ring[i] = (uint32_t)verts.size();
            verts.push_back({{ox[i], by[i], oz[i]}, {0,-1,0}, bodyCol});
        }
        for (int i = 0; i < 8; ++i) {
            int j = (i + 1) % 8;
            idx.insert(idx.end(), { center, ring[j], ring[i] });
        }
    }

    // --- Top face (beltline octagon) ---
    {
        uint32_t center = (uint32_t)verts.size();
        verts.push_back({{0.f, belt, 0.f}, {0,1,0}, bodyCol});
        uint32_t ring[8];
        for (int i = 0; i < 8; ++i) {
            ring[i] = (uint32_t)verts.size();
            verts.push_back({{ox[i], belt, oz[i]}, {0,1,0}, bodyCol});
        }
        for (int i = 0; i < 8; ++i) {
            int j = (i + 1) % 8;
            idx.insert(idx.end(), { center, ring[i], ring[j] });
        }
    }

    // --- Cabin ---
    glm::vec4 glassCol { 0.10f, 0.12f, 0.20f, 0.45f };  // dark tinted glass
    glm::vec4 roofCol  = col * 0.85f;  roofCol.a = col.a;

    // Windshield
    {
        float dz = aZ - wsTopZ, dy = roofY - belt;
        float len = std::sqrt(dz*dz + dy*dy);
        glm::vec3 n = { 0.f, dz/len, dy/len };
        face({-beltHW, belt, aZ}, { beltHW, belt, aZ},
             { roofHW, roofY, wsTopZ}, {-roofHW, roofY, wsTopZ}, n, glassCol);
    }

    // Rear window
    {
        float dz = roofRearZ - rwBaseZ, dy = roofY - belt;
        float len = std::sqrt(dz*dz + dy*dy);
        glm::vec3 n = { 0.f, -dz/len, -dy/len };
        face({ roofHW, roofY, roofRearZ}, {-roofHW, roofY, roofRearZ},
             {-beltHW, belt, rwBaseZ}, { beltHW, belt, rwBaseZ}, n, glassCol);
    }

    // Roof
    face({-roofHW, roofY, wsTopZ}, { roofHW, roofY, wsTopZ},
         { roofHW, roofY, roofRearZ}, {-roofHW, roofY, roofRearZ}, {0,1,0}, roofCol);

    // Side windows (glass)
    face({ beltHW, belt, aZ}, { beltHW, belt, rwBaseZ},
         { roofHW, roofY, roofRearZ}, { roofHW, roofY, wsTopZ}, {1,0,0}, glassCol);
    face({-beltHW, belt, rwBaseZ}, {-beltHW, belt, aZ},
         {-roofHW, roofY, wsTopZ}, {-roofHW, roofY, roofRearZ}, {-1,0,0}, glassCol);

    // Fill panels (beltline, front fenders and rear quarters)
    face({ W, belt, front-C}, { W, belt, aZ},
         { beltHW, belt, aZ}, { W-C, belt, front}, {0,1,0}, bodyCol);
    face({-W, belt, aZ}, {-W, belt, front-C},
         {-(W-C), belt, front}, {-beltHW, belt, aZ}, {0,1,0}, bodyCol);
    face({ W, belt, rwBaseZ}, { W, belt, rear+C},
         { W-C, belt, rear}, { beltHW, belt, rwBaseZ}, {0,1,0}, bodyCol);
    face({-W, belt, rear+C}, {-W, belt, rwBaseZ},
         {-beltHW, belt, rwBaseZ}, {-(W-C), belt, rear}, {0,1,0}, bodyCol);

    // --- Headlights (warm white, slightly in front of bumper face) ---
    {
        glm::vec4 hlCol {0.95f, 0.90f, 0.70f, 0.85f};
        float z = front + 0.001f;
        face({0.30f, 0.02f, z}, {0.60f, 0.02f, z},
             {0.60f, 0.20f, z}, {0.30f, 0.20f, z}, {0,0,1}, hlCol);
        face({-0.60f, 0.02f, z}, {-0.30f, 0.02f, z},
             {-0.30f, 0.20f, z}, {-0.60f, 0.20f, z}, {0,0,1}, hlCol);
    }

    // --- Taillights (red, slightly behind rear bumper face) ---
    {
        glm::vec4 tlCol {0.90f, 0.10f, 0.10f, 0.85f};
        float z = rear - 0.001f;
        face({0.60f, 0.05f, z}, {0.30f, 0.05f, z},
             {0.30f, 0.20f, z}, {0.60f, 0.20f, z}, {0,0,-1}, tlCol);
        face({-0.30f, 0.05f, z}, {-0.60f, 0.05f, z},
             {-0.60f, 0.20f, z}, {-0.30f, 0.20f, z}, {0,0,-1}, tlCol);
    }

    (void)tri;  // unused but kept for potential future use
}

// (Bump mesh functions removed — terrain is now a single heightmap mesh)

static void makeGround(float halfSize, VList& verts, IList& idx)
{
    glm::vec4 col{ 0.4f, 0.45f, 0.3f, 0.f };
    glm::vec3 n{ 0, 1, 0 };
    float s = halfSize;
    float y = -6.f;  // below deepest terrain feature (bowl at -5m)
    uint32_t b = (uint32_t)verts.size();
    verts.push_back({{ -s, y,  s }, n, col });
    verts.push_back({{  s, y,  s }, n, col });
    verts.push_back({{  s, y, -s }, n, col });
    verts.push_back({{ -s, y, -s }, n, col });
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
    // Rows go in -fwd so text reads correctly from above (top-down view)
    glm::vec3 fwd{sn, 0.f, -cs};

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

    // Translucent pipeline: depth write OFF, depth bias enabled (for trail decals)
    ds.depthWriteEnable = VK_FALSE;
    rs.depthBiasEnable = VK_TRUE;
    VkDynamicState dynStatesTranslucent[] = {
        VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR, VK_DYNAMIC_STATE_DEPTH_BIAS };
    VkPipelineDynamicStateCreateInfo dynTranslucent{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynTranslucent.dynamicStateCount = 3;
    dynTranslucent.pDynamicStates    = dynStatesTranslucent;
    gp.pDynamicState = &dynTranslucent;
    vkCreateGraphicsPipelines(dev, VK_NULL_HANDLE, 1, &gp, nullptr, &translucentPipe_);
    ds.depthWriteEnable = VK_TRUE;  // restore
    rs.depthBiasEnable = VK_FALSE;
    gp.pDynamicState = &dyn;

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
        makeCarBody(bodyCol, v, i);
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
        makeCylinder(0.03f, 0.5f, 8, axleCol, v, i);  // radius=3cm, half-length=0.5 (scaled)
    });

    // Subframe: flat box spanning the full track width between mount points
    glm::vec4 sfCol { 0.6f, 0.3f, 0.1f, 0.5f };  // rusty orange, translucent
    subframe_ = record([&](VList& v, IList& i){
        makeBox(Vehicle::HALF_TRACK, 0.02f, 0.08f, sfCol, v, i);
    });

    // (Bump unit meshes removed — terrain is now a single heightmap mesh)

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

// ---- terrain upload ---------------------------------------------------------

void Renderer::uploadTerrain(const VulkanContext& ctx, const Terrain& terrain)
{
    VList verts;
    IList indices;
    constexpr int N = Terrain::N;

    // Build vertex grid
    verts.reserve(N * N);
    for (int iz = 0; iz < N; ++iz) {
        for (int ix = 0; ix < N; ++ix) {
            float x = terrain.worldX(ix);
            float z = terrain.worldZ(iz);
            float h = terrain.height(ix, iz);
            glm::vec3 normal = terrain.normalAt(x, z);
            glm::vec4 color = Terrain::surfaceColor(terrain.surfaceAt(ix, iz));
            verts.push_back({{x, h, z}, normal, color});
        }
    }

    // Build index buffer (two triangles per cell)
    indices.reserve((N - 1) * (N - 1) * 6);
    for (int iz = 0; iz < N - 1; ++iz) {
        for (int ix = 0; ix < N - 1; ++ix) {
            uint32_t bl = iz * N + ix;
            uint32_t br = bl + 1;
            uint32_t tl = bl + N;
            uint32_t tr = tl + 1;
            indices.insert(indices.end(), {bl, br, tr, bl, tr, tl});
        }
    }

    terrainIdxCount_ = (uint32_t)indices.size();

    // Upload vertex buffer
    {
        VkDeviceSize vSz = verts.size() * sizeof(Vertex);
        terrainVbuf_ = ctx.allocBuffer(vSz,
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            terrainVmem_);
        void* p; vkMapMemory(dev_, terrainVmem_, 0, vSz, 0, &p);
        std::memcpy(p, verts.data(), vSz);
        vkUnmapMemory(dev_, terrainVmem_);
    }

    // Upload index buffer
    {
        VkDeviceSize iSz = indices.size() * sizeof(uint32_t);
        terrainIbuf_ = ctx.allocBuffer(iSz,
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            terrainImem_);
        void* p; vkMapMemory(dev_, terrainImem_, 0, iSz, 0, &p);
        std::memcpy(p, indices.data(), iSz);
        vkUnmapMemory(dev_, terrainImem_);
    }

    std::printf("Terrain: %zu verts, %zu indices (%u triangles)\n",
                verts.size(), indices.size(), terrainIdxCount_ / 3);
}

// ---- sprung chase camera ---------------------------------------------------

void Renderer::updateChaseCam(float dt, const Vehicle& veh, const Terrain& terrain)
{
    constexpr float CHASE_DIST   = 6.f;   // meters behind
    constexpr float CHASE_HEIGHT = 2.5f;  // meters above vehicle origin
    constexpr float YAW_TAU      = 0.35f; // yaw spring time constant (seconds)
    constexpr float POS_TAU      = 0.3f;  // position spring time constant
    constexpr float CLIP_MARGIN  = 0.5f;  // minimum clearance above terrain
    constexpr float RESET_DIST   = 20.f;  // re-init if car teleports

    // Compute desired position from a heading
    auto desiredPosFromHeading = [&](float h) {
        float ch = std::cos(h), sh = std::sin(h);
        glm::vec3 behind(-sh, 0.f, -ch);  // opposite to heading direction
        return veh.position + behind * CHASE_DIST
             + glm::vec3(0.f, CHASE_HEIGHT, 0.f);
    };

    // First frame or teleport (reset): snap to target instantly
    if (!chaseCamInit_ ||
        glm::length(chaseCamPos_ - veh.position) > RESET_DIST) {
        chaseCamHeading_ = veh.heading;
        chaseCamPos_ = desiredPosFromHeading(veh.heading);
        chaseCamInit_ = true;
        return;
    }

    if (dt <= 0.f) return;

    // Target heading: direction of motion (XZ plane), falling back to
    // vehicle heading at low speed for smooth standstill behavior.
    float hzSpeed = std::sqrt(veh.velocity.x * veh.velocity.x
                            + veh.velocity.z * veh.velocity.z);
    float motionHeading = std::atan2(veh.velocity.x, veh.velocity.z);
    constexpr float MOTION_MIN_SPEED = 2.f;  // m/s
    constexpr float MOTION_FULL_SPEED = 6.f;
    float motionBlend = std::clamp((hzSpeed - MOTION_MIN_SPEED)
                                    / (MOTION_FULL_SPEED - MOTION_MIN_SPEED), 0.f, 1.f);
    // Blend between vehicle heading (at rest) and motion heading (at speed)
    float targetHeading;
    {
        float diff = motionHeading - veh.heading;
        constexpr float PI  = 3.14159265f;
        constexpr float TAU = 6.28318530f;
        while (diff >  PI) diff -= TAU;
        while (diff < -PI) diff += TAU;
        targetHeading = veh.heading + diff * motionBlend;
    }

    // --- Spring heading toward target (shortest angular path) ---
    float headingDiff = targetHeading - chaseCamHeading_;
    // Wrap to [-π, π]
    constexpr float PI  = 3.14159265f;
    constexpr float TAU = 6.28318530f;
    while (headingDiff >  PI) headingDiff -= TAU;
    while (headingDiff < -PI) headingDiff += TAU;
    float headingAlpha = 1.f - std::exp(-dt / YAW_TAU);
    chaseCamHeading_ += headingDiff * headingAlpha;

    // --- Spring position toward desired ---
    // Stiffen at higher speed to prevent zoom-in/out oscillation.
    // TAU 0.3 at rest → 0.08 at 30+ m/s (nearly rigid follow).
    float speedNorm = std::clamp(hzSpeed / 30.f, 0.f, 1.f);
    float posTau = POS_TAU * (1.f - 0.73f * speedNorm);
    glm::vec3 desiredPos = desiredPosFromHeading(chaseCamHeading_);
    float posAlpha = 1.f - std::exp(-dt / posTau);
    chaseCamPos_ += (desiredPos - chaseCamPos_) * posAlpha;

    // --- Anti-clip: push camera above terrain ---
    float terrainH = terrain.heightAt(chaseCamPos_.x, chaseCamPos_.z);
    float minY = terrainH + CLIP_MARGIN;
    if (chaseCamPos_.y < minY) {
        chaseCamPos_.y = minY;
    }
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

    // Terrain heightmap mesh (opaque, drawn before translucent vehicle)
    if (terrainIdxCount_ > 0) {
        push(glm::mat4(1.f));
        VkDeviceSize tOff = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &terrainVbuf_, &tOff);
        vkCmdBindIndexBuffer(cmd, terrainIbuf_, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, terrainIdxCount_, 1, 0, 0, 0);

        // Re-bind scene buffers
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbuf_, &tOff);
        vkCmdBindIndexBuffer(cmd, ibuf_, 0, VK_INDEX_TYPE_UINT32);
    }

    // Wheels, rims, and axles (after bumps so tires blend over obstacles,
    // before translucent body so they're visible through it)
    for (int i = 0; i < 4; ++i) {
        glm::mat4 wheelRot;
        if (i < 2) {
            // Steering in body-local frame (rotate around local Y), then to world
            glm::mat4 localSteer = glm::rotate(glm::mat4(1.f), veh.frontSteerAngle, {0,1,0});
            wheelRot = glm::mat4(veh.bodyRotation) * localSteer;
        } else {
            wheelRot = glm::mat4(veh.bodyRotation);
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

    // === TRANSLUCENT PASS (depth write OFF — so wheels/axles show through body) ===
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, translucentPipe_);
    vkCmdSetDepthBias(cmd, 0.f, 0.f, 0.f);  // default: no bias

    // Tire trails — slight depth bias pushes them in front of coplanar terrain
    if (hasTrails && trailIdxCount_ > 0) {
        vkCmdSetDepthBias(cmd, -2.f, 0.f, -2.f);
        push(glm::mat4(1.f));
        VkDeviceSize trailOff = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &trailVbuf_, &trailOff);
        vkCmdBindIndexBuffer(cmd, trailIbuf_, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, trailIdxCount_, 1, 0, 0, 0);
        vkCmdSetDepthBias(cmd, 0.f, 0.f, 0.f);  // reset for body/subframes

        // Re-bind scene buffers
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbuf_, &trailOff);
        vkCmdBindIndexBuffer(cmd, ibuf_, 0, VK_INDEX_TYPE_UINT32);
    }

    // Body — offset upward by BODY_Y so body bottom aligns with mount points
    glm::vec3 bodyCenter = veh.position
        + veh.bodyRotation * glm::vec3(0.f, Vehicle::BODY_Y, 0.f);
    glm::mat4 bodyM = glm::translate(glm::mat4(1.f), bodyCenter)
        * glm::mat4(veh.bodyRotation);
    push(bodyM);
    drawSlice(body_);

    // (subframes removed — arches show wheels directly)
}

void Renderer::drawHud(VkCommandBuffer cmd, uint32_t W, uint32_t H, const HudData& hud)
{
    // Build HUD geometry in screen-space pixels
    VList hudV;
    IList hudI;

    float fw = (float)W;
    float fh = (float)H;

    // --- Speedometer (right side) ---
    // Layout (bottom-up): speed + km/h, RPM bar, gear indicator
    float hudRight = fw - 10.f;   // right margin
    {
        char speedStr[16];
        int spd = (int)(std::abs(hud.speedKmh) + 0.5f);
        std::snprintf(speedStr, sizeof(speedStr), "%3d", spd);
        // 5×7 font at scale 3: each char = 18px wide. 3 chars = 54px + 2×3 gap = 60px
        float sx = hudRight - 170.f;
        buildText(speedStr, sx, 4.f, 3.f, {1.f, 1.f, 1.f, 1.f}, hudV, hudI);
        buildText("KM/H", sx + 62.f, 4.f, 1.5f, {0.6f, 0.6f, 0.6f, 1.f}, hudV, hudI);
    }

    // --- Gear indicator (large, right of speed) ---
    {
        char gearStr[4];
        std::snprintf(gearStr, sizeof(gearStr), "%d", hud.gear);
        buildText(gearStr, hudRight - 30.f, 4.f, 3.f, {0.9f, 0.9f, 0.3f, 1.f}, hudV, hudI);
    }

    // --- RPM bar ---
    {
        float barW = 170.f;
        float barX = hudRight - barW;
        float barY = 30.f;
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
                    const TrailGeometry* trails,
                    float frameDt)
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
    auto& terrainLabels = playground.terrain.labels();
    if (!terrainLabels.empty()) {
        VList labelV;
        IList labelI;
        glm::vec4 labelCol{1.f, 1.f, 1.f, 0.9f};  // white, slightly transparent
        float pixelSize = 0.12f;  // 12cm per pixel — readable from above

        for (auto& lbl : terrainLabels) {
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

    // Horizon-level directions: heading-only (no pitch/roll) for top views
    glm::vec3 hFwd   = glm::normalize(glm::vec3{fwd.x, 0.f, fwd.z});
    glm::vec3 hRight = glm::normalize(glm::vec3{right.x, 0.f, right.z});
    glm::vec3 hUp    = {0.f, 1.f, 0.f};

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

    // Top-left: BEHIND view (horizon-level, no pitch/roll tilt)
    quads[0] = { 0, 0, hw - GAP, hh - GAP,
        fixMirror(vkOrtho(-range*aspQ, range*aspQ, -range, range, 0.1f, 100.f) *
        glm::lookAt(pos - hFwd * 10.f,
                     lookTarget, hUp))
    };

    // Top-right: SIDE view (horizon-level, no pitch/roll tilt)
    quads[1] = { hw + GAP, 0, hw - GAP, hh - GAP,
        fixMirror(vkOrtho(-range*aspQ, range*aspQ, -range, range, 0.1f, 100.f) *
        glm::lookAt(pos + hRight * 10.f,
                     lookTarget, hUp))
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

    // Bottom-right: SPRUNG CHASE CAM — smoothly follows heading, ignores roll/pitch
    {
        updateChaseCam(frameDt, veh, playground.terrain);
        glm::vec3 chaseTgt = veh.position + glm::vec3(0.f, 0.5f, 0.f);
        quads[3] = { hw + GAP, hh + GAP, hw - GAP, hh - GAP,
            fixMirror(vkPersp(glm::radians(55.f), aspQ, 0.1f, 500.f) *
            glm::lookAt(chaseCamPos_, chaseTgt, glm::vec3{0.f, 1.f, 0.f}))
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
    // Clear depth so HUD is never occluded by scene geometry
    VkClearAttachment depthClear{};
    depthClear.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    depthClear.clearValue.depthStencil = {1.f, 0};
    VkClearRect clearRect{};
    clearRect.rect = {{0, 0}, {W, H}};
    clearRect.layerCount = 1;
    vkCmdClearAttachments(cmd, 1, &depthClear, 1, &clearRect);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
    drawHud(cmd, W, H, hud);
}

// ---- shutdown --------------------------------------------------------------

void Renderer::shutdown(VkDevice dev)
{
    if (terrainIbuf_) vkDestroyBuffer(dev, terrainIbuf_, nullptr);
    if (terrainImem_) vkFreeMemory   (dev, terrainImem_, nullptr);
    if (terrainVbuf_) vkDestroyBuffer(dev, terrainVbuf_, nullptr);
    if (terrainVmem_) vkFreeMemory   (dev, terrainVmem_, nullptr);
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
