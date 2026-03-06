#pragma once
#include "Suspension.h"
#include "Wheel.h"
#include <cmath>

// Subframe: intermediate structure between left/right suspension and body.
//
// Each axle (front, rear) has one subframe holding the two corner
// suspensions. Forces currently pass through 1:1, but the subframe
// provides the natural attachment point for:
//   - Sway bars (couple left/right vertical forces)
//   - Steering rack (front subframe — steerAngle)
//   - Differential (rear subframe, or front for AWD)
//
// The subframe does NOT add a new dynamic degree of freedom — it's
// rigidly attached to the body. It's an organizational unit that
// groups per-axle concerns.

struct Subframe {
    // Left = index 0, Right = index 1
    Suspension suspension[2];
    Wheel*     wheel[2] = {nullptr, nullptr};  // non-owning pointers, set in init

    // Steering angle for this axle (radians, positive = right).
    // Set by VehiclePhysics each frame. Rear subframe stays at 0.
    float steerAngle = 0.f;

    // Transmit vertical forces from both corners to the body.
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

    // Decompose body-frame velocity at a corner into wheel-local long/lat.
    // bodyVx = rightward velocity, bodyVz = forward velocity (both in body frame).
    void decomposeVelocity(float bodyVx, float bodyVz,
                           float& vLong, float& vLat) const
    {
        float cs = std::cos(steerAngle), ss = std::sin(steerAngle);
        vLong = bodyVz * cs + bodyVx * ss;
        vLat  = bodyVx * cs - bodyVz * ss;
    }

    // Transform wheel-local forces (longitudinal, lateral) to body frame.
    void transformForces(float fLong, float fLat,
                         float& bodyFx, float& bodyFz) const
    {
        float cs = std::cos(steerAngle), ss = std::sin(steerAngle);
        bodyFx = fLong * ss + fLat * cs;
        bodyFz = fLong * cs - fLat * ss;
    }

    // Pitch torque from forward forces at mount points.
    float pitchTorqueFromForward(float bodyFz0, float bodyFz1) const
    {
        return suspension[0].mountPoint.y * bodyFz0
             + suspension[1].mountPoint.y * bodyFz1;
    }

    // Roll torque from lateral forces at mount points.
    float rollTorqueFromLateral(float bodyFx0, float bodyFx1) const
    {
        return -(suspension[0].mountPoint.y * bodyFx0
               + suspension[1].mountPoint.y * bodyFx1);
    }

    // Yaw torque from horizontal forces at wheel positions.
    // tau_y = rz * Fx - rx * Fz for each corner.
    float yawTorqueFromHorizontal(float bodyFx0, float bodyFz0,
                                  float bodyFx1, float bodyFz1) const
    {
        float rz0 = wheel[0]->localOffset.z, rx0 = wheel[0]->localOffset.x;
        float rz1 = wheel[1]->localOffset.z, rx1 = wheel[1]->localOffset.x;
        return (rz0 * bodyFx0 - rx0 * bodyFz0)
             + (rz1 * bodyFx1 - rx1 * bodyFz1);
    }
};
