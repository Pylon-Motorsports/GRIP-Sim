#include "Body.hpp"
#include "VehiclePhysics.hpp"
#include <algorithm>
#include <cmath>

Body::Body(glm::vec3 cg)
    : VehiclePhysicsComponent("Body", cg) {}

void Body::init() {
    addChild(std::make_unique<Mass>(1300.f,
        glm::vec3(500.f, 2000.f, 500.f)));

    VehiclePhysicsComponent::init();
}

void Body::reset() {
    pendingForce_  = glm::vec3(0.f);
    pendingTorque_ = glm::vec3(0.f);
    VehiclePhysicsComponent::reset();
}

Mass* Body::mass() {
    return findChild<Mass>("Mass");
}

float Body::totalMassKg() const {
    float m = 0.f;
    for (auto& c : children())
        if (auto* mc = dynamic_cast<const Mass*>(c.get()))
            m += mc->massKg;
    return m;
}

glm::vec3 Body::inertia() const {
    for (auto& c : children())
        if (auto* mc = dynamic_cast<const Mass*>(c.get()))
            return mc->inertia;
    return glm::vec3(500.f, 2000.f, 500.f);
}

std::vector<glm::vec3> Body::colliderCorners() const {
    // Collision sample points matching the visual car body mesh (makeCarBody).
    constexpr float W = COLLIDER_HALF_W;
    constexpr float C = COLLIDER_CHAMFER;
    constexpr float F = COLLIDER_FRONT;
    constexpr float R = COLLIDER_REAR;
    constexpr float B = COLLIDER_BEVEL;

    // Plan-view octagon outline (CW from front-left, same as makeCarBody)
    const float ox[8] = { -(W-C), (W-C),  W,       W,      (W-C), -(W-C), -W,      -W     };
    const float oz[8] = {  F,     F,      F-C,     R+C,     R,     R,      R+C,     F-C    };
    // Per-vertex bottom Y: front/rear bumper tips beveled up
    const float by[8] = { COLLIDER_BOT_Y+B, COLLIDER_BOT_Y+B,
                           COLLIDER_BOT_Y,   COLLIDER_BOT_Y,
                           COLLIDER_BOT_Y+B, COLLIDER_BOT_Y+B,
                           COLLIDER_BOT_Y,   COLLIDER_BOT_Y };

    std::vector<glm::vec3> pts;
    pts.reserve(20);

    // Bottom octagon (8 points, with beveled front/rear)
    for (int i = 0; i < 8; ++i)
        pts.push_back({ ox[i], by[i], oz[i] });

    // Beltline octagon (8 points)
    for (int i = 0; i < 8; ++i)
        pts.push_back({ ox[i], COLLIDER_BELT_Y, oz[i] });

    // Roof corners (4 points — narrower, shorter front-to-back)
    constexpr float roofHW   = 0.72f;
    constexpr float wsTopZ   = 0.35f;   // roof front edge
    constexpr float roofRearZ = -0.85f; // roof rear edge
    pts.push_back({ -roofHW, COLLIDER_ROOF_Y, wsTopZ });
    pts.push_back({  roofHW, COLLIDER_ROOF_Y, wsTopZ });
    pts.push_back({  roofHW, COLLIDER_ROOF_Y, roofRearZ });
    pts.push_back({ -roofHW, COLLIDER_ROOF_Y, roofRearZ });

    return pts;
}

void Body::applyCollisionResponse(const std::vector<CollisionContact>& contacts,
                                  const VehicleState& state, float dt) {
    if (contacts.empty() || dt <= 0.f) return;

    float mass = totalMassKg();
    if (mass < 1.f) mass = 1300.f;

    glm::vec3 cgWorld = state.position + state.bodyRotation * attachmentPoint_;

    // --- Pass 1: compute per-contact forces, accumulate total force + centroid ---
    glm::vec3 totalForce(0.f);
    glm::vec3 weightedCentroid(0.f);  // force-weighted average contact point
    float totalForceMag = 0.f;

    // Distribute mass budget across all active contacts so total impulse
    // respects the restitution limit even with many simultaneous contacts.
    int activeCount = 0;
    for (auto& c : contacts)
        if (c.penetration > 0.f) ++activeCount;
    float massPerContact = (activeCount > 0) ? mass / (float)activeCount : mass;

    for (auto& c : contacts) {
        if (c.penetration <= 0.f) continue;

        float pen = std::min(c.penetration, maxPenetration_);

        // Contact point velocity. The angular contribution is clamped
        // to prevent a feedback loop: large ω → large vInto → large force
        // → large torque → even larger ω next step.
        glm::vec3 r = c.worldPoint - cgWorld;
        float cgInto = -glm::dot(state.velocity, c.normal);
        glm::vec3 angularContrib = glm::cross(state.angularVel, r);
        float angInto = -glm::dot(angularContrib, c.normal);
        float cgSpeed = glm::length(state.velocity);
        // Angular contribution limited to 50% of CG speed to prevent
        // rotation-force-rotation feedback loops while still allowing
        // realistic contact dynamics
        float angLimit = cgSpeed * 0.5f;
        float clampedAngInto = std::clamp(angInto, -angLimit, angLimit);
        float vInto = cgInto + clampedAngInto;

        // Spring force: push out of overlap
        float springF = stiffness_ * pen;

        // Damping: absorb energy in both directions.
        // Approaching (vInto > 0): damper adds to spring force, decelerating approach.
        // Separating (vInto < 0): damper subtracts from spring force, absorbing
        // stored elastic energy during rebound (prevents excessive bounce).
        float dampF = damping_ * vInto;

        float forceMag = std::max(0.f, springF + dampF);

        // Restitution cap: limit per-contact impulse based on CG approach
        if (vInto > 0.f) {
            float capSpeed = std::min(vInto, std::max(cgInto, 0.f) + angLimit);
            float maxImpulse = (1.f + restitution_) * massPerContact * capSpeed;
            float maxForce   = maxImpulse / dt;
            forceMag = std::min(forceMag, maxForce);
        }

        if (forceMag <= 0.f) continue;

        totalForce += c.normal * forceMag;
        weightedCentroid += c.worldPoint * forceMag;
        totalForceMag += forceMag;
    }

    // --- Pass 2: compute torque from total force applied at centroid ---
    // Using the force-weighted centroid eliminates phantom torque from
    // nearly-symmetric simultaneous contacts while preserving genuine
    // asymmetric torque (e.g., single corner hitting first).
    glm::vec3 totalTorque(0.f);
    if (totalForceMag > 0.f) {
        glm::vec3 centroid = weightedCentroid / totalForceMag;
        glm::vec3 r = centroid - cgWorld;

        // Ground factor: reduce torque for steep contacts (ramps/walls)
        glm::vec3 forceDir = glm::normalize(totalForce);
        float groundFactor = std::clamp(
            (std::abs(forceDir.y) - 0.5f) / 0.5f, 0.f, 1.f);
        totalTorque = groundFactor * glm::cross(r, totalForce);

        // Angular damping during contact: friction and deformation at
        // the contact patch resist body rotation.  Clamped per-axis to
        // never reverse angular velocity (prevents overshoot oscillation).
        {
            constexpr float ANG_DAMP_RATE = 30000.f;  // N·m·s/rad
            float contactIntensity = std::min(totalForceMag / (mass * 9.81f), 2.f);
            glm::vec3 angDamp = -ANG_DAMP_RATE * contactIntensity * state.angularVel;

            // Clamp: torque can decelerate but never reverse rotation in one step
            glm::vec3 inertia = glm::vec3(500.f, 2000.f, 500.f);
            for (int a = 0; a < 3; ++a) {
                float maxT = std::abs(state.angularVel[a]) * inertia[a] / dt;
                angDamp[a] = std::clamp(angDamp[a], -maxT, maxT);
            }
            totalTorque += angDamp;
        }
    }

    pendingForce_  += totalForce;
    pendingTorque_ += totalTorque;
}

ComponentOutput Body::compute(const ComponentInput& /*input*/) {
    ComponentOutput out;
    out.force  = pendingForce_;
    out.torque = pendingTorque_;
    pendingForce_  = glm::vec3(0.f);
    pendingTorque_ = glm::vec3(0.f);
    return out;
}
