#include "AeroSurface.hpp"
#include <cmath>

AeroSurface::AeroSurface(std::string name, glm::vec3 attachPt, float drag, float lift)
    : VehiclePhysicsComponent(std::move(name), attachPt)
    , dragCoeff(drag), liftCoeff(lift) {}

ComponentOutput AeroSurface::compute(const ComponentInput& input) {
    float speed = glm::length(input.velocity);
    if (speed < 0.5f) return {};

    // Body-frame velocity to get forward speed component
    glm::vec3 localVel = glm::transpose(input.bodyRotation) * input.velocity;
    float vFwd = localVel.z;

    // How well the car is aligned with its velocity (1 = perfect, 0 = sideways)
    float fwdAlign = std::abs(vFwd) / speed;

    // Drag: opposes world velocity direction, magnitude from forward speed.
    // This prevents body-rotation-dependent torque oscillations in flight.
    float dragMag = dragCoeff * vFwd * std::abs(vFwd);
    glm::vec3 velDir = input.velocity / speed;
    glm::vec3 dragForce = -velDir * std::abs(dragMag);

    // Downforce: world -Y, scaled by forward speed squared and alignment.
    // Only meaningful when car is moving roughly forward.
    float liftMag = liftCoeff * vFwd * std::abs(vFwd) * fwdAlign;
    glm::vec3 liftForce = glm::vec3(0.f, liftMag, 0.f);

    return { .force = dragForce + liftForce };
}
