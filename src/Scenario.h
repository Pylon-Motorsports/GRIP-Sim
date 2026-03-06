#pragma once
#include <vector>
#include <cmath>

struct Bump {
    float zCenter;     // position along Z (driving direction)
    float halfLength;  // half-extent along Z (bump "width")
    float height;      // peak height (m)
    float xMin, xMax;  // lateral extent
};

struct Scenario {
    const char* name;
    std::vector<Bump> bumps;
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

inline std::vector<Scenario> createScenarios()
{
    std::vector<Scenario> scenarios;

    // 1: Flat ground
    scenarios.push_back({"Flat", {}});

    // 2: Full-width speed bumps every 15m
    {
        std::vector<Bump> bumps;
        for (int i = 0; i < 12; ++i) {
            bumps.push_back({
                8.f + i * 15.f,  // start at z=8, every 15m
                0.30f,           // 60cm total length along Z
                0.08f,           // 8cm tall
                -5.f, 5.f        // full width
            });
        }
        scenarios.push_back({"Speed Bumps", std::move(bumps)});
    }

    // 3: Alternating left/right bumps every 10m
    {
        std::vector<Bump> bumps;
        for (int i = 0; i < 15; ++i) {
            bool leftSide = (i % 2 == 0);
            bumps.push_back({
                6.f + i * 10.f,
                0.30f,
                0.08f,
                leftSide ? -2.f : 0.f,
                leftSide ?  0.f : 2.f,
            });
        }
        scenarios.push_back({"One-Side Bumps", std::move(bumps)});
    }

    // 4: Rolling hills — gentle sine-wave undulations
    {
        std::vector<Bump> bumps;
        // Create overlapping wide bumps to approximate a rolling sine wave.
        // Each bump is a cosine hump; overlapping them creates continuous hills.
        for (int i = 0; i < 20; ++i) {
            bumps.push_back({
                5.f + i * 12.f,  // center every 12m
                6.0f,            // 12m total wavelength (halfLength=6m for wide hills)
                0.15f + 0.10f * std::sin(i * 1.3f),  // vary height 5-25cm
                -5.f, 5.f        // full width
            });
        }
        scenarios.push_back({"Hills", std::move(bumps)});
    }

    return scenarios;
}
