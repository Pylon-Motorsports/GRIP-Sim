#pragma once
#include <glm/glm.hpp>

struct Vehicle {
    glm::vec3 position { 0.f, 0.f, 0.f };
    float     heading  { 0.f };            // radians, 0 = +Z
    glm::vec3 wheelPos[4]{};               // world positions of wheel centers

    // Body (box)
    static constexpr float BODY_HALF_W = 0.30f;   // 0.60 m wide
    static constexpr float BODY_HALF_H = 0.25f;   // 0.50 m tall
    static constexpr float BODY_HALF_L = 1.30f;   // 2.60 m long
    static constexpr float BODY_Y      = 0.55f;   // center-of-body height

    // Wheels (cylinders, axis along local X)
    static constexpr float HALF_TRACK   = 0.75f;  // CG to wheel center (X)
    static constexpr float FRONT_AXLE   = 1.10f;  // CG to front axle (Z)
    static constexpr float REAR_AXLE    = 1.10f;  // CG to rear axle (Z)
    static constexpr float WHEEL_RADIUS = 0.30f;
    static constexpr float WHEEL_HALF_W = 0.10f;  // 0.20 m wide
    static constexpr float WHEEL_Y      = WHEEL_RADIUS;  // center height
};
