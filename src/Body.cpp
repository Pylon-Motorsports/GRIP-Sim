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

std::array<glm::vec3, 8> Body::colliderCorners() const {
    // 8 corners of the bounding box in body-local coordinates (relative to CG)
    return {{
        { -COLLIDER_HALF_W, COLLIDER_BOT_Y, COLLIDER_FRONT },  // 0: front-left-bottom
        {  COLLIDER_HALF_W, COLLIDER_BOT_Y, COLLIDER_FRONT },  // 1: front-right-bottom
        { -COLLIDER_HALF_W, COLLIDER_TOP_Y, COLLIDER_FRONT },  // 2: front-left-top
        {  COLLIDER_HALF_W, COLLIDER_TOP_Y, COLLIDER_FRONT },  // 3: front-right-top
        { -COLLIDER_HALF_W, COLLIDER_BOT_Y, COLLIDER_REAR  },  // 4: rear-left-bottom
        {  COLLIDER_HALF_W, COLLIDER_BOT_Y, COLLIDER_REAR  },  // 5: rear-right-bottom
        { -COLLIDER_HALF_W, COLLIDER_TOP_Y, COLLIDER_REAR  },  // 6: rear-left-top
        {  COLLIDER_HALF_W, COLLIDER_TOP_Y, COLLIDER_REAR  },  // 7: rear-right-top
    }};
}

void Body::applyCollisionResponse(const std::vector<CollisionContact>& contacts,
                                  const VehicleState& state, float dt) {
    if (contacts.empty() || dt <= 0.f) return;

    float mass = totalMassKg();
    if (mass < 1.f) mass = 1300.f;

    glm::vec3 totalForce(0.f);
    glm::vec3 totalTorque(0.f);

    for (auto& c : contacts) {
        if (c.penetration <= 0.f) continue;

        float pen = std::min(c.penetration, maxPenetration_);

        // Velocity of body at contact point
        glm::vec3 r = c.worldPoint - (state.position + state.bodyRotation * attachmentPoint_);
        glm::vec3 vPoint = state.velocity + glm::cross(state.angularVel, r);

        // Velocity into the surface (positive = approaching)
        float vInto = -glm::dot(vPoint, c.normal);

        // Spring force: push out of overlap
        float springF = stiffness_ * pen;

        // Damping: absorb energy (only when approaching)
        float dampF = (vInto > 0.f) ? damping_ * vInto : 0.f;

        float forceMag = springF + dampF;

        // Restitution cap: limit the total impulse so rebound velocity
        // is at most restitution * approach velocity
        if (vInto > 0.f) {
            float maxImpulse = (1.f + restitution_) * mass * vInto;
            float maxForce   = maxImpulse / dt;
            forceMag = std::min(forceMag, maxForce);
        }

        if (forceMag <= 0.f) continue;

        glm::vec3 contactForce = c.normal * forceMag;

        // Torque: smoothly blend lever-arm torque based on how ground-like
        // the contact is.  Pure ground (|normal.y|=1) gets full lever arm;
        // steep ramps/walls (|normal.y|<0.7) get reduced or zero lever arm
        // to prevent disproportionate launch off inclines.
        float groundFactor = std::clamp(
            (std::abs(c.normal.y) - 0.5f) / 0.5f, 0.f, 1.f);
        totalTorque += groundFactor * glm::cross(r, contactForce);

        totalForce += contactForce;
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
