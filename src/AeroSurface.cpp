#include "AeroSurface.hpp"
#include <cmath>

AeroSurface::AeroSurface(std::string name, glm::vec3 attachPt, float drag, float lift)
    : VehiclePhysicsComponent(std::move(name), attachPt)
    , dragCoeff(drag), liftCoeff(lift) {}

ComponentOutput AeroSurface::compute(const ComponentInput& input) {
    glm::vec3 localVel = glm::transpose(input.bodyRotation) * input.velocity;
    float vFwd    = localVel.z;
    float vFwdSq  = vFwd * std::abs(vFwd);

    glm::vec3 dragForce = input.bodyRotation * glm::vec3(0.f, 0.f, -dragCoeff * vFwdSq);
    glm::vec3 liftForce = glm::vec3(0.f, liftCoeff * vFwdSq, 0.f);

    return { .force = dragForce + liftForce };
}
