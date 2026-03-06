#pragma once
#include <glm/glm.hpp>

// Mediates forces between a wheel and the body at one corner.
// Solid axle: forces pass through 1:1 in both directions.
class Suspension {
public:
    // Force from wheel transmitted up to the body
    glm::vec3 transmitToBody(const glm::vec3& wheelForce) const {
        return wheelForce;  // solid: 1:1
    }

    // Force from body transmitted down to the wheel
    glm::vec3 transmitToWheel(const glm::vec3& bodyForce) const {
        return bodyForce;   // solid: 1:1
    }
};
