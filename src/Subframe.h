#pragma once
#include "Suspension.h"
#include "Wheel.h"

// Subframe: intermediate structure between left/right suspension and body.
//
// Each axle (front, rear) has one subframe holding the two corner
// suspensions. Forces currently pass through 1:1, but the subframe
// provides the natural attachment point for:
//   - Sway bars (couple left/right vertical forces)
//   - Steering rack (front subframe)
//   - Differential (rear subframe, or front for AWD)
//
// The subframe does NOT add a new dynamic degree of freedom — it's
// rigidly attached to the body. It's an organizational unit that
// groups per-axle concerns.

struct Subframe {
    // Left = index 0, Right = index 1
    Suspension suspension[2];
    Wheel*     wheel[2] = {nullptr, nullptr};  // non-owning pointers, set in init

    // Transmit vertical forces from both corners to the body.
    // Currently 1:1 pass-through. Sway bar would redistribute here:
    //   delta = (leftDeflection - rightDeflection) * swayBarRate
    //   leftForce  -= delta
    //   rightForce += delta
    struct AxleForces {
        glm::vec3 bodyForce{0.f};
        glm::vec3 bodyTorque{0.f};
    };

    AxleForces transmitVertical(const glm::vec3& leftForce,
                                const glm::vec3& rightForce) const
    {
        AxleForces af;
        glm::vec3 transL = suspension[0].transmitToBody(leftForce);
        glm::vec3 transR = suspension[1].transmitToBody(rightForce);
        af.bodyForce  = transL + transR;
        af.bodyTorque = suspension[0].torqueOnBody(transL)
                      + suspension[1].torqueOnBody(transR);
        return af;
    }

    // Transmit longitudinal forces (drive/brake) from both corners.
    // Differential would split drive torque here.
    float transmitLongitudinal(float leftLongF, float rightLongF) const
    {
        return leftLongF + rightLongF;
    }

    // Pitch torque from longitudinal forces at mount points.
    float pitchTorqueFromLongitudinal(float leftLongF, float rightLongF) const
    {
        return suspension[0].mountPoint.y * leftLongF
             + suspension[1].mountPoint.y * rightLongF;
    }
};
