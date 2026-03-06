#pragma once
#include "Tire.h"
#include <glm/glm.hpp>
#include <cmath>

// Wheel (rim + brake rotor) paired 1:1 with a Tire.
//
// The wheel tracks its own angular velocity as an independent degree of
// freedom. The tire's brush slip model couples wheel rotation to vehicle
// translation: engine torque spins the wheel, slip produces a tire force
// that accelerates the vehicle, and the tire reaction torque decelerates
// the wheel back toward ground speed.
//
// Force flow per step:
//   1. Tire deflection from hub vs ground geometry
//   2. Normal force from tire radial spring-damper
//   3. Contact patch area from deflection geometry
//   4. Slip ratio from wheel speed vs ground speed
//   5. Longitudinal force from brush model + rolling resistance
//   6. Integrate wheel angular velocity (drive + brake + bearing - tire reaction)

class Wheel {
public:
    float massKg   = 20.f;
    glm::vec3 localOffset{0.f};  // position relative to body CG

    // Rotational state — independent DOF, integrated each step
    float angularVel = 0.f;  // rad/s (positive = forward rolling)

    // Brakes and bearing
    float bearingFrictionNm = 2.f;
    float maxBrakeTorqueNm  = 500.f;

    // The tire mounted on this wheel
    Tire tire;

    struct Forces {
        float normalForce;       // vertical reaction (N), upward
        float longitudinalForce; // along forward dir (N), from brush slip model
        float rollingResistance; // along forward dir (N), direct body force
    };

    // Compute all tire forces and integrate wheel rotation.
    //
    //   hubY:         world Y of wheel center (from body position + offset)
    //   hubVelY:      vertical velocity of hub (m/s, positive = up)
    //   groundY:      terrain height below this wheel
    //   forwardSpeed: vehicle forward speed at this corner (m/s)
    //   driveTorque:  from engine via drivetrain (Nm), 0 for non-driven wheels
    //   brakePedal:   [0, 1]
    //   dt:           timestep (s)
    Forces computeForces(float hubY, float hubVelY, float groundY,
                         float forwardSpeed, float driveTorque,
                         float brakePedal, float dt)
    {
        Forces f{};

        // 1. Deflection
        float d = tire.computeDeflection(hubY, groundY);
        tire.deflection = d;

        // 2. Deflection rate: dDot > 0 means compressing.
        // d = (groundY + R) - hubY, so dDot = -hubVelY (flat ground approx)
        float dDot = -hubVelY;

        // 3. Normal force
        float Fn = tire.computeNormalForce(d, dDot);
        tire.normalLoad = Fn;
        f.normalForce = Fn;

        // 4. Contact patch
        tire.updateContactPatch(d);

        // Wheel rotational inertia (solid cylinder approximation)
        float I = 0.5f * massKg * tire.radius * tire.radius;

        if (d <= 0.f) {
            // Airborne: no ground forces, wheel spins freely with drag
            float regSign = angularVel / (std::abs(angularVel) + 0.2f);
            float dragTorque = -bearingFrictionNm * regSign;
            angularVel += (driveTorque + dragTorque) / I * dt;
            return f;
        }

        // 5. Slip ratio
        float wheelSpeed = angularVel * tire.radius;
        constexpr float slipEps = 0.5f;  // regularization for low speed
        float denom = std::max(std::abs(forwardSpeed), slipEps);
        float slipRatio = (wheelSpeed - forwardSpeed) / denom;

        // 6. Longitudinal force from brush model
        float Fx = tire.computeLongitudinalForce(slipRatio, Fn);
        f.longitudinalForce = Fx;

        // 7. Rolling resistance — applied directly to vehicle body, NOT through
        // the wheel/slip model. Rolling resistance is caused by asymmetric contact
        // patch deformation, not by a torque at the hub.
        f.rollingResistance = tire.computeRollingResistance(Fn, forwardSpeed);

        // 8. Integrate wheel angular velocity
        //    Torques on wheel: drive + bearing - tire reaction, then brake friction
        float regSign = angularVel / (std::abs(angularVel) + 0.2f);
        float bearingTorque = -bearingFrictionNm * regSign;
        float tireReaction  = -Fx * tire.radius;

        float unbrakeTorque = driveTorque + bearingTorque + tireReaction;

        // Brake is a friction device: opposes rotation up to its torque capacity.
        // When the wheel is nearly stopped and the brake can absorb all remaining
        // torque, the wheel locks — no oscillation, no velocity-dependent ramp.
        float brakeCap = maxBrakeTorqueNm * brakePedal;
        float netTorque;
        if (brakeCap > 0.f && std::abs(unbrakeTorque) <= brakeCap
            && std::abs(angularVel) < 1.f)
        {
            // Brake holds: absorbs all torque, wheel is locked
            netTorque = 0.f;
            angularVel = 0.f;
        } else {
            // Brake is sliding or not applied: opposes rotation
            float brakeTorque = -brakeCap * regSign;
            netTorque = unbrakeTorque + brakeTorque;
        }

        angularVel += netTorque / I * dt;

        return f;
    }

    // Ground speed from wheel rotation (for speedometer)
    float groundSpeed() const { return angularVel * tire.radius; }
};
