#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

struct Bump {
    float zCenter;     // position along Z (driving direction)
    float halfLength;  // half-extent along Z (bump "width")
    float height;      // peak height (m)
    float xMin, xMax;  // lateral extent
};

struct GroundLabel {
    const char* text;
    glm::vec3 position;   // world position (placed on ground)
    float heading;        // rotation around Y (radians), 0 = text faces +X
};

struct Playground {
    std::vector<Bump> bumps;
    std::vector<GroundLabel> labels;
};

// Query ground height at a world point.
// Uses cosine profile for smooth bump shape.
inline float groundHeight(const std::vector<Bump>& bumps, float x, float z)
{
    float h = 0.f;
    for (auto& b : bumps) {
        if (x < b.xMin || x > b.xMax) continue;
        float dz = std::abs(z - b.zCenter);
        if (dz >= b.halfLength) continue;
        float bh = b.height * 0.5f * (1.f + std::cos(3.14159265f * dz / b.halfLength));
        if (bh > h) h = bh;
    }
    return h;
}

// Compute ground surface normal at a world point.
inline glm::vec3 groundNormal(const std::vector<Bump>& bumps, float x, float z)
{
    float h = 0.f;
    float dhdz = 0.f;
    for (auto& b : bumps) {
        if (x < b.xMin || x > b.xMax) continue;
        float dz = z - b.zCenter;
        float adz = std::abs(dz);
        if (adz >= b.halfLength) continue;
        float bh = b.height * 0.5f * (1.f + std::cos(3.14159265f * adz / b.halfLength));
        if (bh > h) {
            h = bh;
            float sign = (dz >= 0.f) ? 1.f : -1.f;
            dhdz = -b.height * 0.5f * (3.14159265f / b.halfLength)
                   * std::sin(3.14159265f * adz / b.halfLength) * sign;
        }
    }
    return glm::normalize(glm::vec3{0.f, 1.f, -dhdz});
}

inline Playground createPlayground()
{
    Playground pg;
    auto& bumps = pg.bumps;
    auto& labels = pg.labels;

    // Helper: place label at ground level just before each section
    auto label = [&](const char* text, float z, float x = 0.f) {
        labels.push_back({text, {x, 0.01f, z}, 0.f});
    };

    // ===== 1. FLAT — z = 0..30 =============================================
    label("FLAT", 2.f, -3.f);
    // No bumps — flat ground for baseline testing

    // ===== 2. SPEED BUMPS — z = 35..100 ====================================
    label("SPEED BUMPS", 35.f, -3.f);
    for (int i = 0; i < 8; ++i) {
        bumps.push_back({
            40.f + i * 8.f,   // every 8m
            0.30f,            // 60cm total Z length
            0.08f,            // 8cm tall
            -5.f, 5.f         // full width
        });
    }

    // ===== 3. ONE-SIDE BUMPS — z = 105..175 ================================
    label("ONE-SIDE BUMPS", 105.f, -3.f);
    for (int i = 0; i < 10; ++i) {
        bool leftSide = (i % 2 == 0);
        bumps.push_back({
            110.f + i * 7.f,
            0.30f,
            0.08f,
            leftSide ? -2.f : 0.f,
            leftSide ?  0.f : 2.f,
        });
    }

    // ===== 4. ROLLING HILLS — z = 180..340 =================================
    label("ROLLING HILLS", 180.f, -3.f);
    for (int i = 0; i < 8; ++i) {
        bumps.push_back({
            190.f + i * 20.f,
            10.0f,            // 20m wavelength
            0.35f + 0.25f * std::sin(i * 1.3f),
            -5.f, 5.f
        });
    }

    // ===== 5. JUMP — z = 350..400 ==========================================
    label("JUMP", 350.f, -3.f);
    // Ramp up
    bumps.push_back({
        365.f,
        8.f,                  // 16m ramp
        0.80f,                // 80cm peak
        -3.f, 3.f
    });
    // Landing slope (negative after gap)
    bumps.push_back({
        390.f,
        6.f,
        0.40f,
        -3.f, 3.f
    });

    // ===== 6. STUTTER BUMPS — z = 410..460 =================================
    label("STUTTER BUMPS", 410.f, -3.f);
    for (int i = 0; i < 20; ++i) {
        bumps.push_back({
            415.f + i * 2.5f,
            0.8f,             // 1.6m wavelength
            0.04f,            // 4cm tall — rapid small bumps
            -5.f, 5.f
        });
    }

    // ===== 7. BANKED LEFT — z = 470..540 ===================================
    label("BANKED LEFT", 470.f, -3.f);
    // Left side raised, right side flat — simulates left bank
    for (int i = 0; i < 8; ++i) {
        bumps.push_back({
            480.f + i * 8.f,
            4.0f,
            0.25f + 0.05f * i,  // increasing bank
            -5.f, -0.5f         // left half only
        });
    }

    // ===== 8. BANKED RIGHT — z = 550..620 ==================================
    label("BANKED RIGHT", 550.f, -3.f);
    for (int i = 0; i < 8; ++i) {
        bumps.push_back({
            560.f + i * 8.f,
            4.0f,
            0.25f + 0.05f * i,
            0.5f, 5.f           // right half only
        });
    }

    // ===== 9. BIG HILL — z = 630..720 ======================================
    label("BIG HILL", 630.f, -3.f);
    bumps.push_back({
        675.f,
        30.f,                   // 60m wavelength — big gentle hill
        1.5f,                   // 1.5m tall!
        -5.f, 5.f
    });

    // ===== 10. QUARTER PIPE — z = 730..770 =================================
    label("QUARTER PIPE", 730.f, -3.f);
    // Steep ramp, one side only — wall to ride up
    bumps.push_back({
        750.f,
        6.f,
        1.2f,                   // 1.2m vertical wall
        -5.f, -1.f              // left side wall
    });
    bumps.push_back({
        750.f,
        6.f,
        1.2f,
        1.f, 5.f                // right side wall
    });

    // ===== 11. MOGULS — z = 780..880 =======================================
    label("MOGULS", 780.f, -3.f);
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 3; ++j) {
            float xOff = -2.f + j * 2.f + ((i % 2) ? 1.f : 0.f);
            bumps.push_back({
                790.f + i * 10.f,
                3.0f,
                0.20f + 0.08f * std::sin(i * 1.1f + j * 2.f),
                xOff - 1.f, xOff + 1.f
            });
        }
    }

    // ===== 12. DOUBLE JUMP — z = 890..950 ==================================
    label("DOUBLE JUMP", 890.f, -3.f);
    // First jump
    bumps.push_back({ 905.f, 6.f, 0.70f, -3.f, 3.f });
    // Second jump (bigger)
    bumps.push_back({ 935.f, 8.f, 1.00f, -3.f, 3.f });

    // ===== 13. WASHBOARD — z = 960..1030 ===================================
    label("WASHBOARD", 960.f, -3.f);
    for (int i = 0; i < 30; ++i) {
        bumps.push_back({
            965.f + i * 2.2f,
            0.6f,
            0.03f,             // 3cm — rapid tiny bumps
            -5.f, 5.f
        });
    }

    // ===== 14. HALF PIPE — z = 1040..1090 ==================================
    label("HALF PIPE", 1040.f, -3.f);
    // Both sides raised like a half pipe channel
    for (int i = 0; i < 6; ++i) {
        float z = 1050.f + i * 8.f;
        bumps.push_back({ z, 4.f, 0.80f, -5.f, -1.5f });  // left wall
        bumps.push_back({ z, 4.f, 0.80f,  1.5f, 5.f  });  // right wall
    }

    // ===== 15. RUNOUT — z = 1100+ ==========================================
    label("RUNOUT", 1100.f, -3.f);
    // Flat — safe braking zone

    return pg;
}
