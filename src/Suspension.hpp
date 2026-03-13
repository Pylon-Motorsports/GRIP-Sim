#pragma once
#include <glm/glm.hpp>

// Mediates forces between a wheel and the body at one corner.
//
// The suspension connects to the body at a mounting point (the point
// on the body nearest the wheel center).  Forces from the wheel are
// transmitted to the body at this point, creating both a linear force
// and a torque about the body's center of gravity.
class Suspension {
public:
    // Mounting point on the body surface, in body-local coordinates
    // (relative to CG).  Set during init.
    glm::vec3 mountPoint{0.f};

    // Force from wheel transmitted up to the body (1:1 for now)
    glm::vec3 transmitToBody(const glm::vec3& wheelForce) const {
        return wheelForce;
    }

    // Torque on the body from a force applied at the mounting point.
    // torque = mountPoint × force  (cross product)
    glm::vec3 torqueOnBody(const glm::vec3& force) const {
        return glm::cross(mountPoint, force);
    }
};
