#pragma once
#include "Mass.hpp"
#include <vector>

struct VehicleState;

struct CollisionContact {
    glm::vec3 worldPoint;      // world-space contact point
    glm::vec3 normal;          // surface normal (pointing INTO the body / away from obstacle)
    float     penetration;     // positive = overlap depth (m)
};

class Body : public VehiclePhysicsComponent {
public:
    // Collision volume dimensions (body-local, relative to CG).
    // Matches the visual mesh in Renderer.cpp makeCarBody().
    static constexpr float COLLIDER_HALF_W  =  0.88f;  // base box half-width
    static constexpr float COLLIDER_FRONT   =  1.95f;  // front bumper z
    static constexpr float COLLIDER_REAR    = -1.85f;  // rear bumper z
    static constexpr float COLLIDER_BOT_Y   = -0.25f;  // floor pan
    static constexpr float COLLIDER_BELT_Y  =  0.30f;  // beltline
    static constexpr float COLLIDER_ROOF_Y  =  0.75f;  // roofline
    static constexpr float COLLIDER_CHAMFER =  0.15f;  // corner chamfer distance
    static constexpr float COLLIDER_BEVEL   =  0.12f;  // bottom front/rear bevel height

    Body(glm::vec3 cg);

    void init() override;
    void reset() override;

    Mass* mass();
    float totalMassKg() const;
    glm::vec3 inertia() const;

    void setRestitution(float e) { restitution_ = e; }

    // Returns collision sample points in body-local coordinates.
    // Bottom octagon (8) + beltline octagon (8) + roof corners (4) = 20 points.
    std::vector<glm::vec3> colliderCorners() const;

    // Compute collision response from external contacts.
    // Stores forces internally; consumed by compute() on next update().
    void applyCollisionResponse(const std::vector<CollisionContact>& contacts,
                                const VehicleState& state, float dt);

protected:
    ComponentOutput compute(const ComponentInput& input) override;

private:
    float restitution_  = 0.03f;  // nearly inelastic — absorb energy on impact
    float stiffness_    = 300000.f;
    float damping_      = 40000.f;  // near-critical for 1300kg (absorb, don't bounce)
    float maxPenetration_ = 0.15f; // clamp to prevent explosion

    glm::vec3 pendingForce_  { 0.f };
    glm::vec3 pendingTorque_ { 0.f };
};
