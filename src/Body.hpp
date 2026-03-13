#pragma once
#include "Mass.hpp"
#include <array>

struct VehicleState;

struct CollisionContact {
    glm::vec3 worldPoint;      // world-space contact point
    glm::vec3 normal;          // surface normal (pointing INTO the body / away from obstacle)
    float     penetration;     // positive = overlap depth (m)
};

class Body : public VehiclePhysicsComponent {
public:
    // Collision box dimensions (body-local, relative to CG)
    static constexpr float COLLIDER_FRONT   =  1.65f;  // front bumper (overhangs front axle at 1.35)
    static constexpr float COLLIDER_REAR    = -1.55f;  // rear bumper  (overhangs rear axle at -1.25)
    static constexpr float COLLIDER_HALF_W  =  0.86f;  // half-width (track + wheel bulge)
    static constexpr float COLLIDER_TOP_Y   =  0.25f;  // top of body (relative to CG at y=0.35)
    static constexpr float COLLIDER_BOT_Y   = -0.10f;  // bottom of body (mid-wheel height)

    Body(glm::vec3 cg);

    void init() override;
    void reset() override;

    Mass* mass();
    float totalMassKg() const;
    glm::vec3 inertia() const;

    void setRestitution(float e) { restitution_ = e; }

    // Returns 8 corners of the collision box in body-local coordinates.
    std::array<glm::vec3, 8> colliderCorners() const;

    // Compute collision response from external contacts.
    // Stores forces internally; consumed by compute() on next update().
    void applyCollisionResponse(const std::vector<CollisionContact>& contacts,
                                const VehicleState& state, float dt);

protected:
    ComponentOutput compute(const ComponentInput& input) override;

private:
    float restitution_  = 0.1f;   // 0=full absorb, 1=elastic bounce
    float stiffness_    = 300000.f;
    float damping_      = 40000.f;  // near-critical for 1300kg (absorb, don't bounce)
    float maxPenetration_ = 0.15f; // clamp to prevent explosion

    glm::vec3 pendingForce_  { 0.f };
    glm::vec3 pendingTorque_ { 0.f };
};
