#pragma once
#include <glm/glm.hpp>

// Base tire class: radial spring-damper with geometric contact patch.
//
// The single source of truth is radial deflection 'd': how much the
// tire's circular cross-section overlaps the ground plane.
//
//   d = (groundY + R) - hubY
//
// Physical limits:
//   d <= 0:      no contact, zero forces
//   0 < d < dMax: sidewall deforming, spring-damper active
//   d >= dMax:    rim contact — tire can't compress further
//
// Slip force computation is delegated to derived classes (BrushTire,
// LuGreTire) via the virtual computeSlipForces() method.

class Tire {
public:
    virtual ~Tire() = default;

    // --- Geometry ---
    float radius        = 0.30f;   // unloaded outer radius (m)
    float width         = 0.20f;   // tread width (m)
    float maxDeflection = 0.025f;  // dMax: rim contact limit (m), ~25mm for off-road

    // --- Radial spring-damper (sidewall + inflation pressure) ---
    float radialStiffness  = 200000.f;   // N/m (typical car tire 150k-250k)
    float radialDamping    = 2000.f;     // N·s/m (combined tire + suspension damping)
    float bumpStopStiffness = 20000000.f; // N/m² (progressive beyond maxDeflection)

    // --- Friction ---
    float mu                  = 1.15f;   // peak friction coefficient (200tw street tire, dry tarmac)
    float muSliding           = 0.85f;   // sliding/locked friction (~74% of peak)
    float rollingResistCoeff  = 0.015f;  // Crr

    // --- Per-step state (read by external code for HUD, etc.) ---
    float deflection         = 0.f;  // current d (m)
    float normalLoad         = 0.f;  // current Fn (N)
    float contactPatchLength = 0.f;  // chord length (m)
    float contactPatchArea   = 0.f;  // width × chord (m²)

    float computeDeflection(float hubY, float groundY) const;
    float computeNormalForce(float d, float dDot) const;
    void  updateContactPatch(float d);
    void  frictionCircleClamp(float& longForce, float& latForce, float Fn) const;
    float computeRollingResistance(float Fn, float forwardSpeed) const;

    virtual void computeSlipForces(float slipRatio, float slipAngle,
                                   float vLong, float vLat, float wheelSpeed,
                                   float Fn, float dt,
                                   float& Fx, float& Fy) = 0;

    // --- Volumetric contact interface (PressureTire) ---
    virtual bool usesVolumetricContact() const { return false; }

    struct VolumetricInput {
        glm::vec3 hubWorldPos;
        glm::vec3 axleWorldDir;
        glm::vec3 forwardDir;
        glm::vec3 hubVelocity;
        float vLong, vLat;
        float wheelAngularVel;
    };

    struct VolumetricResult {
        glm::vec3 worldForce{0.f};
        float normalLoad = 0.f;
        float contactArea = 0.f;
        float wheelTorqueReaction = 0.f;
        float rollingResistForce = 0.f;
    };

    virtual VolumetricResult computeVolumetricForces(
        const VolumetricInput& /*in*/, float /*dt*/) { return {}; }
};
