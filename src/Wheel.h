#pragma once
#include "Tire.h"
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

// Wheel (rim + brake rotor) paired 1:1 with a Tire.
//
// The wheel computes net torque from drive, bearing friction, and brakes.
// The tire handles the contact patch: converts torque to force, applies
// traction limits and rolling resistance via regularized Coulomb friction.
//
// Wheel and tire are linked 1:1 rotationally and (for now) laterally
// and vertically — no separate compliance between them yet.
class Wheel {
public:
    float massKg   = 20.f;
    float radius   = 0.17f;   // rim radius (visual only — tire.radius is physics)
    float halfWidth = 0.08f;  // rim half-width (visual)
    glm::vec3 localOffset{0.f};  // position relative to body CG

    // Rotational state (derived from vehicle speed when on ground)
    float angularVel = 0.f;  // rad/s (positive = forward rolling)

    // Internal friction
    float bearingFrictionNm = 2.f;    // kinetic bearing drag (Nm)
    float maxBrakeTorqueNm  = 500.f;  // max hydraulic brake torque per wheel (Nm)

    // The tire mounted on this wheel
    Tire tire;

    // Compute vertical forces (gravity + ground normal spring/damper).
    // Also updates tire.normalLoad for use in longitudinal calculations.
    glm::vec3 computeVerticalForces(const glm::vec3& worldPos, float velY, float groundY)
    {
        constexpr float G = 9.81f;
        glm::vec3 force{0.f, -massKg * G, 0.f};

        float penetration = groundY - (worldPos.y - tire.radius);
        if (penetration > 0.f) {
            constexpr float K = 120000.f;
            float C = 2.f * std::sqrt(K * massKg) * 0.9f;
            float normalMag = K * penetration - C * velY;
            normalMag = std::max(normalMag, 0.f);
            force.y += normalMag;
            tire.normalLoad = normalMag;
        } else {
            tire.normalLoad = 0.f;
        }

        return force;
    }

    // Compute longitudinal force at the contact patch.
    //   driveTorque:  from engine via drivetrain (Nm)
    //   brakePedal:   [0, 1]
    //   forwardSpeed: vehicle forward speed (m/s)
    //   isOnGround:   whether wheel has ground contact
    // Returns: longitudinal force (N), positive = forward
    float computeLongitudinalForce(float driveTorque, float brakePedal,
                                   float forwardSpeed, bool isOnGround)
    {
        // Update angular velocity from vehicle speed (no-slip, for now)
        if (isOnGround) {
            angularVel = forwardSpeed / tire.radius;
        }

        if (!isOnGround) return 0.f;

        // Regularized sign for bearing and brake friction
        float regSign = forwardSpeed / (std::abs(forwardSpeed) + tire.epsilon);

        // Net torque at the wheel hub
        float netTorque = driveTorque;
        netTorque -= bearingFrictionNm * regSign;
        netTorque -= maxBrakeTorqueNm * brakePedal * regSign;

        // Tire handles the contact patch physics
        return tire.computeForce(netTorque, forwardSpeed);
    }

    bool onGround(const glm::vec3& worldPos, float groundY) const {
        return (worldPos.y - tire.radius) <= groundY + 0.001f;
    }

    float groundSpeed() const { return angularVel * tire.radius; }
};
