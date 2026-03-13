#pragma once
#include <glm/glm.hpp>

struct Vehicle {
    glm::vec3 position { 0.f, 0.f, 0.f };
    float     heading  { 0.f };            // radians, 0 = +Z
    float     pitch    { 0.f };            // radians, positive = nose up
    float     roll     { 0.f };            // radians, positive = right side down
    glm::mat3 bodyRotation { 1.f };        // body-local to world rotation (from physics)
    glm::vec3 velocity { 0.f, 0.f, 0.f }; // world-space velocity (for chase cam)
    glm::vec3 wheelPos[4]{};               // world positions of wheel/tire centers
    glm::vec3 mountPos[4]{};               // world positions of suspension mount points
    float     frontSteerAngle { 0.f };     // front wheel steering angle (radians)

    // Per-wheel data for tire trail rendering
    float     wheelSlipRatio[4]{};         // longitudinal slip ratio [-1, 1]
    float     wheelSlipAngle[4]{};         // lateral slip angle (radians)
    float     wheelNormalLoad[4]{};        // normal force (N), 0 = airborne
    float     wheelContactWidth[4]{};      // contact patch width (m)

    // Body (box)
    static constexpr float BODY_HALF_W = 0.80f;   // 1.60 m wide (BRZ ~1.78m)
    static constexpr float BODY_HALF_H = 0.25f;   // (used by collision, not visual)
    static constexpr float BODY_HALF_L = 1.30f;   // 2.60 m long
    static constexpr float BODY_Y      = 0.25f;   // visual body center offset above origin

    // Wheels (rim — visual only)
    static constexpr float HALF_TRACK   = 0.75f;  // CG to wheel center (X)
    static constexpr float FRONT_AXLE   = 1.03f;  // CG to front axle (Z) — ~53% front weight
    static constexpr float REAR_AXLE    = 0.97f;  // CG to rear axle (Z) — wheelbase 2.00m
    static constexpr float WHEEL_RADIUS = 0.30f;  // tire outer radius (physics)
    static constexpr float WHEEL_HALF_W = 0.10f;  // tire half-width

    // Rim (inner wheel, visual)
    static constexpr float RIM_RADIUS   = 0.17f;  // rim outer radius
    static constexpr float RIM_HALF_W   = 0.08f;  // rim half-width

    // Tire (rubber donut around rim, visual)
    static constexpr float TIRE_RADIUS  = WHEEL_RADIUS;  // same as physics radius
    static constexpr float TIRE_HALF_W  = 0.07f;         // narrower than rim so rim edges show

    static constexpr float WHEEL_Y      = WHEEL_RADIUS;  // center height
};
