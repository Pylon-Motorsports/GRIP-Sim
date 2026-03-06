#include "VehicleMesh.h"
#include <glm/gtc/constants.hpp>
#include <cmath>

namespace vehicle_geom {

// ---------------------------------------------------------------------------
// Box — 6 faces, each with 4 unique vertices (correct per-face normals).
// ---------------------------------------------------------------------------
VehicleMesh makeBox(float hx, float y_bot, float y_top, float hz_rear, float hz_front,
                    glm::vec4 color)
{
    VehicleMesh m;
    auto& V = m.vertices;
    auto& I = m.indices;

    // Corners: [left/right, bottom/top, rear/front] → x, y, z
    // X: ±hx, Y: [y_bot, y_top], Z: [-hz_rear, +hz_front]
    glm::vec3 c[8] = {
        {-hx, y_bot, -hz_rear}, {+hx, y_bot, -hz_rear},
        {+hx, y_top, -hz_rear}, {-hx, y_top, -hz_rear},
        {-hx, y_bot, +hz_front},{+hx, y_bot, +hz_front},
        {+hx, y_top, +hz_front},{-hx, y_top, +hz_front},
    };

    // Helper: add a quad (4 verts sharing a normal), emit 2 triangles
    auto addFace = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c2, glm::vec3 d, glm::vec3 n) {
        uint32_t base = static_cast<uint32_t>(V.size());
        V.push_back({a, n, color}); V.push_back({b, n, color});
        V.push_back({c2, n, color}); V.push_back({d, n, color});
        I.push_back(base);   I.push_back(base+1); I.push_back(base+2);
        I.push_back(base);   I.push_back(base+2); I.push_back(base+3);
    };

    addFace(c[0], c[4], c[7], c[3], {-1, 0, 0});  // -X left
    addFace(c[5], c[1], c[2], c[6], {+1, 0, 0});  // +X right
    addFace(c[0], c[1], c[5], c[4], { 0,-1, 0});  // -Y bottom
    addFace(c[7], c[6], c[2], c[3], { 0,+1, 0});  // +Y top
    addFace(c[1], c[0], c[3], c[2], { 0, 0,-1});  // -Z rear
    addFace(c[4], c[5], c[6], c[7], { 0, 0,+1});  // +Z front

    return m;
}

// ---------------------------------------------------------------------------
// Cylinder — axis along local +X (wheel axle).
// Side wall + two disc caps.
// ---------------------------------------------------------------------------
VehicleMesh makeCylinder(float radius, float halfLen, int segs, glm::vec4 color)
{
    VehicleMesh m;
    auto& V = m.vertices;
    auto& I = m.indices;

    const float PI2 = 2.f * glm::pi<float>();

    // ---- Side wall ----
    uint32_t sideBase = 0;
    for (int i = 0; i <= segs; ++i) {
        float a  = (PI2 * i) / segs;
        float s  = std::sin(a), c = std::cos(a);
        glm::vec3 n{0.f, s, c};
        V.push_back({{+halfLen, radius * s, radius * c}, n, color});  // front ring
        V.push_back({{-halfLen, radius * s, radius * c}, n, color});  // back ring
    }
    for (int i = 0; i < segs; ++i) {
        uint32_t b = sideBase + i * 2;
        I.push_back(b+0); I.push_back(b+2); I.push_back(b+3);
        I.push_back(b+0); I.push_back(b+3); I.push_back(b+1);
    }

    // ---- Front cap (+X face) ----
    uint32_t fCenter = static_cast<uint32_t>(V.size());
    V.push_back({{+halfLen, 0.f, 0.f}, {+1, 0, 0}, color});
    uint32_t fRingBase = static_cast<uint32_t>(V.size());
    for (int i = 0; i < segs; ++i) {
        float a = (PI2 * i) / segs;
        V.push_back({{+halfLen, radius * std::sin(a), radius * std::cos(a)}, {+1,0,0}, color});
    }
    for (int i = 0; i < segs; ++i) {
        uint32_t a = fRingBase + i, b = fRingBase + (i + 1) % segs;
        I.push_back(fCenter); I.push_back(a); I.push_back(b);
    }

    // ---- Back cap (-X face) ----
    uint32_t bCenter = static_cast<uint32_t>(V.size());
    V.push_back({{-halfLen, 0.f, 0.f}, {-1, 0, 0}, color});
    uint32_t bRingBase = static_cast<uint32_t>(V.size());
    for (int i = 0; i < segs; ++i) {
        float a = (PI2 * i) / segs;
        V.push_back({{-halfLen, radius * std::sin(a), radius * std::cos(a)}, {-1,0,0}, color});
    }
    for (int i = 0; i < segs; ++i) {
        uint32_t a = bRingBase + i, b = bRingBase + (i + 1) % segs;
        I.push_back(bCenter); I.push_back(b); I.push_back(a);  // reversed winding for back
    }

    return m;
}

// ---------------------------------------------------------------------------
// Beam — thin rectangular solid along local +Y, centred at origin.
// Used for lower control arm (oriented at runtime from wheel to body).
// ---------------------------------------------------------------------------
VehicleMesh makeBeam(float halfLength, float halfWidth, float halfHeight, glm::vec4 color)
{
    // Reuse makeBox with Y as the long axis
    // halfWidth → X, halfHeight → Z, halfLength → Y
    return makeBox(halfWidth, -halfLength, halfLength, halfHeight, halfHeight, color);
}

// ---------------------------------------------------------------------------
// Strut — thin cylinder along local +Y (vertical), centred at origin.
// Used for spring/damper strut visualization.
// ---------------------------------------------------------------------------
VehicleMesh makeStrut(float radius, float halfLength, int segs, glm::vec4 color)
{
    VehicleMesh m;
    auto& V = m.vertices;
    auto& I = m.indices;

    const float PI2 = 2.f * glm::pi<float>();

    // Side wall — axis along +Y
    uint32_t sideBase = 0;
    for (int i = 0; i <= segs; ++i) {
        float a = (PI2 * i) / segs;
        float s = std::sin(a), c = std::cos(a);
        glm::vec3 n{s, 0.f, c};
        V.push_back({{radius * s, +halfLength, radius * c}, n, color});  // top ring
        V.push_back({{radius * s, -halfLength, radius * c}, n, color});  // bottom ring
    }
    for (int i = 0; i < segs; ++i) {
        uint32_t b = sideBase + i * 2;
        I.push_back(b+0); I.push_back(b+2); I.push_back(b+3);
        I.push_back(b+0); I.push_back(b+3); I.push_back(b+1);
    }

    // Top cap (+Y)
    uint32_t tCenter = static_cast<uint32_t>(V.size());
    V.push_back({{0.f, +halfLength, 0.f}, {0, +1, 0}, color});
    uint32_t tRingBase = static_cast<uint32_t>(V.size());
    for (int i = 0; i < segs; ++i) {
        float a = (PI2 * i) / segs;
        V.push_back({{radius * std::sin(a), +halfLength, radius * std::cos(a)}, {0,+1,0}, color});
    }
    for (int i = 0; i < segs; ++i) {
        uint32_t a2 = tRingBase + i, b2 = tRingBase + (i + 1) % segs;
        I.push_back(tCenter); I.push_back(a2); I.push_back(b2);
    }

    // Bottom cap (-Y)
    uint32_t bCenter = static_cast<uint32_t>(V.size());
    V.push_back({{0.f, -halfLength, 0.f}, {0, -1, 0}, color});
    uint32_t bRingBase = static_cast<uint32_t>(V.size());
    for (int i = 0; i < segs; ++i) {
        float a = (PI2 * i) / segs;
        V.push_back({{radius * std::sin(a), -halfLength, radius * std::cos(a)}, {0,-1,0}, color});
    }
    for (int i = 0; i < segs; ++i) {
        uint32_t a2 = bRingBase + i, b2 = bRingBase + (i + 1) % segs;
        I.push_back(bCenter); I.push_back(b2); I.push_back(a2);  // reversed winding
    }

    return m;
}

// ---------------------------------------------------------------------------
// Tree batch — all trees as one mesh for efficient rendering.
// Each tree: 6-sided trunk cylinder (brown) + 6-sided canopy cone (green).
// ---------------------------------------------------------------------------
VehicleMesh makeTreeBatch(const std::vector<glm::vec3>& positions,
                          const std::vector<float>& heights,
                          const std::vector<float>& radii)
{
    VehicleMesh m;
    auto& V = m.vertices;
    auto& I = m.indices;

    static constexpr int SEGS = 6;
    static constexpr float PI2 = 2.f * glm::pi<float>();
    static constexpr glm::vec4 trunkColor  { 0.40f, 0.26f, 0.13f, 1.0f }; // brown
    static constexpr glm::vec4 canopyColor { 0.13f, 0.55f, 0.13f, 1.0f }; // forest green

    V.reserve(positions.size() * (SEGS * 4 + 4));
    I.reserve(positions.size() * SEGS * 12);

    for (size_t t = 0; t < positions.size(); ++t) {
        glm::vec3 base = positions[t];
        float h = heights[t];
        float r = radii[t];
        float trunkH  = h * 0.4f;   // trunk is 40% of total height
        float canopyH = h * 0.6f;   // canopy is 60%
        float canopyR = r * 5.f;    // canopy radius ~1m for 0.2m trunk

        // --- Trunk cylinder ---
        uint32_t trunkBase = static_cast<uint32_t>(V.size());
        for (int i = 0; i < SEGS; ++i) {
            float a = (PI2 * i) / SEGS;
            float s = std::sin(a), c = std::cos(a);
            glm::vec3 n { s, 0.f, c };
            glm::vec3 bottom = base + glm::vec3{ r * s, 0.f, r * c };
            glm::vec3 top    = base + glm::vec3{ r * s, trunkH, r * c };
            V.push_back({ bottom, n, trunkColor });
            V.push_back({ top,    n, trunkColor });
        }
        for (int i = 0; i < SEGS; ++i) {
            uint32_t b = trunkBase + i * 2;
            uint32_t next = trunkBase + ((i + 1) % SEGS) * 2;
            I.push_back(b);     I.push_back(next);     I.push_back(next + 1);
            I.push_back(b);     I.push_back(next + 1); I.push_back(b + 1);
        }

        // --- Canopy cone ---
        glm::vec3 coneBase = base + glm::vec3{ 0.f, trunkH, 0.f };
        glm::vec3 coneTip  = base + glm::vec3{ 0.f, trunkH + canopyH, 0.f };

        uint32_t coneRingBase = static_cast<uint32_t>(V.size());
        for (int i = 0; i < SEGS; ++i) {
            float a = (PI2 * i) / SEGS;
            float s = std::sin(a), c = std::cos(a);
            // Normal for cone: outward + upward component
            glm::vec3 n = glm::normalize(glm::vec3{ s, canopyR / canopyH, c });
            V.push_back({ coneBase + glm::vec3{ canopyR * s, 0.f, canopyR * c }, n, canopyColor });
        }
        uint32_t tipIdx = static_cast<uint32_t>(V.size());
        V.push_back({ coneTip, { 0.f, 1.f, 0.f }, canopyColor });

        for (int i = 0; i < SEGS; ++i) {
            uint32_t a = coneRingBase + i;
            uint32_t b = coneRingBase + (i + 1) % SEGS;
            I.push_back(a); I.push_back(b); I.push_back(tipIdx);
        }

        // Cone bottom cap
        uint32_t capCenter = static_cast<uint32_t>(V.size());
        V.push_back({ coneBase, { 0.f, -1.f, 0.f }, canopyColor });
        for (int i = 0; i < SEGS; ++i) {
            uint32_t a = coneRingBase + i;
            uint32_t b = coneRingBase + (i + 1) % SEGS;
            I.push_back(capCenter); I.push_back(b); I.push_back(a);
        }
    }

    return m;
}

} // namespace vehicle_geom
