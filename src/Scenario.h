#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

struct Bump {
    float xCenter;     // world X of bump center
    float zCenter;     // world Z of bump center
    float halfLength;  // half-extent along heading (cosine profile axis)
    float halfWidth;   // half-extent perpendicular to heading (flat-topped)
    float height;      // peak height (m)
    float heading;     // rotation around Y (radians), 0 = bump runs along Z
};

struct GroundLabel {
    const char* text;
    glm::vec3 position;   // world position (placed on ground)
    float heading;        // rotation around Y (radians)
};

struct Playground {
    std::vector<Bump> bumps;
    std::vector<GroundLabel> labels;
};

// Transform world (x,z) into bump-local coordinates.
// Local Z = along heading (cosine profile), local X = perpendicular.
inline void toBumpLocal(const Bump& b, float wx, float wz, float& lx, float& lz)
{
    float dx = wx - b.xCenter;
    float dz = wz - b.zCenter;
    float cs = std::cos(b.heading);
    float sn = std::sin(b.heading);
    // Rotate by -heading
    lx =  dx * cs + dz * sn;
    lz = -dx * sn + dz * cs;
}

// Query ground height at a world point.
// Uses cosine profile along bump's heading direction.
inline float groundHeight(const std::vector<Bump>& bumps, float x, float z)
{
    float h = 0.f;
    for (auto& b : bumps) {
        float lx, lz;
        toBumpLocal(b, x, z, lx, lz);
        if (std::abs(lx) > b.halfWidth) continue;
        float alz = std::abs(lz);
        if (alz >= b.halfLength) continue;
        float bh = b.height * 0.5f * (1.f + std::cos(3.14159265f * alz / b.halfLength));
        if (bh > h) h = bh;
    }
    return h;
}

// Compute ground surface normal at a world point.
inline glm::vec3 groundNormal(const std::vector<Bump>& bumps, float x, float z)
{
    float h = 0.f;
    float dhdlz = 0.f;  // gradient in bump-local Z
    float bestHeading = 0.f;
    for (auto& b : bumps) {
        float lx, lz;
        toBumpLocal(b, x, z, lx, lz);
        if (std::abs(lx) > b.halfWidth) continue;
        float alz = std::abs(lz);
        if (alz >= b.halfLength) continue;
        float bh = b.height * 0.5f * (1.f + std::cos(3.14159265f * alz / b.halfLength));
        if (bh > h) {
            h = bh;
            float sign = (lz >= 0.f) ? 1.f : -1.f;
            dhdlz = -b.height * 0.5f * (3.14159265f / b.halfLength)
                     * std::sin(3.14159265f * alz / b.halfLength) * sign;
            bestHeading = b.heading;
        }
    }
    // Rotate gradient back to world space.
    // In bump-local: normal = (0, 1, -dhdlz). Rotate local Z back to world.
    float cs = std::cos(bestHeading);
    float sn = std::sin(bestHeading);
    // Local Z direction in world = (-sin(h), 0, cos(h))
    // dh/dworld = dhdlz * localZ_direction
    float dwx = dhdlz * (-sn);
    float dwz = dhdlz * cs;
    return glm::normalize(glm::vec3{-dwx, 1.f, -dwz});
}

// Helper to make a bump at (r, theta) from origin, oriented radially outward
inline Bump radialBump(float r, float theta, float halfLen, float halfWid,
                       float height)
{
    return {
        r * std::sin(theta),  // xCenter
        r * std::cos(theta),  // zCenter
        halfLen, halfWid, height,
        theta                 // heading = radial direction
    };
}

inline Playground createPlayground()
{
    Playground pg;
    auto& bumps = pg.bumps;
    auto& labels = pg.labels;
    constexpr float PI = 3.14159265f;

    // Helper: label at distance r along angle theta
    auto label = [&](const char* text, float r, float theta) {
        float x = r * std::sin(theta);
        float z = r * std::cos(theta);
        labels.push_back({text, {x, 0.01f, z}, theta});
    };

    // Center label
    labels.push_back({"START", {0.f, 0.01f, 0.f}, 0.f});

    // ===== 0deg (N, +Z): SPEED BUMPS =======================================
    {
        float theta = 0.f;
        label("SPEED BUMPS", 18.f, theta);
        for (int i = 0; i < 8; ++i)
            bumps.push_back(radialBump(25.f + i * 8.f, theta, 0.30f, 4.f, 0.08f));
    }

    // ===== 45deg (NE): ROLLING HILLS ========================================
    {
        float theta = PI * 0.25f;
        label("ROLLING HILLS", 18.f, theta);
        for (int i = 0; i < 6; ++i)
            bumps.push_back(radialBump(30.f + i * 18.f, theta, 9.f, 4.f,
                            0.35f + 0.25f * std::sin(i * 1.3f)));
    }

    // ===== 90deg (E, +X): JUMP ==============================================
    {
        float theta = PI * 0.5f;
        label("JUMP", 18.f, theta);
        // Takeoff ramp
        bumps.push_back(radialBump(35.f, theta, 6.f, 3.f, 0.80f));
        // Landing ramp
        bumps.push_back(radialBump(55.f, theta, 8.f, 3.f, 0.50f));
        // Second bigger jump
        bumps.push_back(radialBump(80.f, theta, 8.f, 3.f, 1.20f));
    }

    // ===== 135deg (SE): STUTTER BUMPS =======================================
    {
        float theta = PI * 0.75f;
        label("STUTTER BUMPS", 18.f, theta);
        for (int i = 0; i < 25; ++i)
            bumps.push_back(radialBump(25.f + i * 2.2f, theta, 0.7f, 4.f, 0.04f));
    }

    // ===== 180deg (S, -Z): MOGULS ===========================================
    {
        float theta = PI;
        label("MOGULS", 18.f, theta);
        // Staggered bump field — offset bumps left/right of centerline
        for (int i = 0; i < 8; ++i) {
            float r = 28.f + i * 8.f;
            // Three bumps across the width, alternating offset
            float lateralShift = (i % 2) ? 1.5f : 0.f;
            for (int j = -1; j <= 1; ++j) {
                float offX = (j * 3.f + lateralShift);
                // Place bump at offset perpendicular to radial direction
                float perpX =  std::cos(theta) * offX;
                float perpZ = -std::sin(theta) * offX;
                bumps.push_back({
                    r * std::sin(theta) + perpX,
                    r * std::cos(theta) + perpZ,
                    2.5f, 1.5f,
                    0.25f + 0.10f * std::sin(i * 1.1f + j * 2.f),
                    theta
                });
            }
        }
    }

    // ===== 225deg (SW): WASHBOARD ===========================================
    {
        float theta = PI * 1.25f;
        label("WASHBOARD", 18.f, theta);
        for (int i = 0; i < 30; ++i)
            bumps.push_back(radialBump(25.f + i * 2.0f, theta, 0.5f, 4.f, 0.03f));
    }

    // ===== 270deg (W, -X): HALF PIPE ========================================
    {
        float theta = PI * 1.5f;
        label("HALF PIPE", 18.f, theta);
        // Two massive walls forming a channel, running radially outward.
        // Walls are 8m tall, 2m thick, extending 40m along the radial direction.
        // The cosine profile runs along the heading so walls taper at the ends.
        // Channel is ~6m wide (walls offset 4m from centerline each side).
        float wallOffset = 4.f;  // perpendicular distance from centerline
        // Perpendicular direction
        float perpX =  std::cos(theta);
        float perpZ = -std::sin(theta);
        for (int i = 0; i < 5; ++i) {
            float r = 25.f + i * 10.f;
            float cx = r * std::sin(theta);
            float cz = r * std::cos(theta);
            // Left wall
            bumps.push_back({
                cx + perpX * wallOffset,
                cz + perpZ * wallOffset,
                5.f, 2.f, 8.f, theta
            });
            // Right wall
            bumps.push_back({
                cx - perpX * wallOffset,
                cz - perpZ * wallOffset,
                5.f, 2.f, 8.f, theta
            });
        }
    }

    // ===== 315deg (NW): BANKED TURNS ========================================
    {
        float theta = PI * 1.75f;
        label("BANKED TURNS", 18.f, theta);
        // Left bank: raise one side progressively
        float perpX =  std::cos(theta);
        float perpZ = -std::sin(theta);
        for (int i = 0; i < 6; ++i) {
            float r = 28.f + i * 10.f;
            float cx = r * std::sin(theta);
            float cz = r * std::cos(theta);
            // Raise left side
            bumps.push_back({
                cx + perpX * 2.5f,
                cz + perpZ * 2.5f,
                5.f, 2.5f, 0.20f + 0.08f * i, theta
            });
        }
        // Then right bank
        for (int i = 0; i < 6; ++i) {
            float r = 95.f + i * 10.f;
            float cx = r * std::sin(theta);
            float cz = r * std::cos(theta);
            bumps.push_back({
                cx - perpX * 2.5f,
                cz - perpZ * 2.5f,
                5.f, 2.5f, 0.20f + 0.08f * i, theta
            });
        }
    }

    return pg;
}
