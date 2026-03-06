#pragma once
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

// One wheel with mass, ground contact, and gravity.
class Wheel {
public:
    float massKg   = 20.f;
    float radius   = 0.30f;
    glm::vec3 localOffset{0.f};  // position relative to body CG

    // Compute net external force on this wheel (gravity + ground normal).
    // worldPos: wheel center in world space
    // velY:     vertical velocity of the wheel (same as body for solid susp)
    // groundY:  height of the ground surface
    glm::vec3 computeForces(const glm::vec3& worldPos, float velY, float groundY) const
    {
        constexpr float G = 9.81f;
        glm::vec3 force{0.f, -massKg * G, 0.f};  // gravity

        float wheelBottom = worldPos.y - radius;
        float penetration = groundY - wheelBottom;
        if (penetration > 0.f) {
            // Ground normal: stiff spring + near-critical damping
            constexpr float K = 120000.f;   // N/m
            float C = 2.f * std::sqrt(K * massKg) * 0.9f;
            float normalMag = K * penetration - C * velY;
            force.y += std::max(normalMag, 0.f);  // normal can only push, not pull
        }

        return force;
    }

    bool onGround(const glm::vec3& worldPos, float groundY) const {
        return (worldPos.y - radius) <= groundY + 0.001f;
    }
};
