#pragma once
#include "Vehicle.hpp"
#include "Subframe.hpp"
#include "Terrain.hpp"
#include <glm/glm.hpp>

// Rigid body: mass, inertia, linear/angular state, aerodynamics, integration.
struct Body {
    static constexpr float GRAVITY = 10.5f;  // slightly above real 9.81 for planted feel

    static constexpr float BUSHING_PITCH_DAMPING = 5000.f;  // N·m·s/rad
    static constexpr float BUSHING_ROLL_DAMPING  = 3000.f;  // N·m·s/rad

    float massKg = 1200.f;

    // Linear state
    glm::vec3 pos{0.f};
    glm::vec3 vel{0.f};
    float heading      = 0.f;
    float forwardSpeed  = 0.f;

    // Angular state
    float pitch     = 0.f;
    float roll      = 0.f;
    float pitchRate = 0.f;
    float rollRate  = 0.f;
    float yawRate   = 0.f;

    // Moments of inertia
    float pitchInertia = 0.f;
    float rollInertia  = 0.f;
    float yawInertia   = 0.f;

    // Aerodynamics
    struct Aero {
        float dragCdA       = 0.70f;
        float downforceCfA  = 0.50f;
        float frontSplit    = 0.50f;
        float dragCopHeight = 0.18f;
        static constexpr float AIR_DENSITY = 1.225f;

        float dynPressure(float speed) const {
            return 0.5f * AIR_DENSITY * speed * speed;
        }
        float dragForce(float speed)      const { return dynPressure(speed) * dragCdA; }
        float downforceTotal(float speed)  const { return dynPressure(speed) * downforceCfA; }
    } aero;

    void computeInertia();
    BodyState bodyState(float lateralSpeed) const;
    void applyAero(const glm::vec3& fwd,
                   glm::vec3& bodyForce, glm::vec3& bodyTorque) const;
    void applyCollider(const Terrain& terrain, const BodyState& bs,
                       glm::vec3& bodyForce, glm::vec3& bodyTorque) const;
    void integrate(const glm::vec3& bodyForce, const glm::vec3& bodyTorque,
                   float totalMass, float dt);
    void reset();
};
