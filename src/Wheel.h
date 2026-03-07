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
//   5. Slip angle from lateral velocity vs forward velocity
//   6. Longitudinal + lateral forces from brush model, clamped to friction circle
//   7. Integrate wheel angular velocity (drive + brake + bearing - tire reaction)

class Wheel {
public:
    float massKg   = 20.f;
    glm::vec3 localOffset{0.f};  // position relative to body CG

    // Rotational state — independent DOF, integrated each step
    float angularVel = 0.f;  // rad/s (positive = forward rolling)

    // Per-step outputs (populated by computeForces, read by external code)
    float lastSlipRatio = 0.f;
    float lastSlipAngle = 0.f;

    // Brakes and bearing
    float bearingFrictionNm = 2.f;

    // Brake caliper: converts hydraulic pressure to clamping torque.
    // torque = pressure × pistonArea × padFriction × discRadius × 2 (both pads)
    float brakePistonAreaM2  = 0.0016f;  // 16 cm² per caliper
    float brakePadMu         = 0.4f;     // pad-to-disc friction
    float brakeDiscRadiusM   = 0.14f;    // effective radius of disc (m)

    float brakeTorqueFromPressure(float pressurePa) const {
        return pressurePa * brakePistonAreaM2 * brakePadMu * brakeDiscRadiusM * 2.f;
    }

    // The tire mounted on this wheel
    Tire tire;

    struct Forces {
        float normalForce;       // vertical reaction (N), upward
        float longitudinalForce; // along wheel heading (N), from brush slip model
        float lateralForce;      // perpendicular to wheel heading (N), from brush slip model
        float rollingResistance; // along wheel heading (N), direct body force
    };

    // Compute all tire forces and integrate wheel rotation.
    //
    //   hubY:         world Y of wheel center (from body position + offset)
    //   hubVelY:      vertical velocity of hub (m/s, positive = up)
    //   groundY:      terrain height below this wheel
    //   vLong:        velocity along wheel heading (m/s)
    //   vLat:         velocity perpendicular to wheel heading (m/s, positive = right)
    //   driveTorque:  from engine via drivetrain (Nm), 0 for non-driven wheels
    //   brakePressure: hydraulic line pressure (Pa)
    //   dt:           timestep (s)
    Forces computeForces(float hubY, float hubVelY, float groundY,
                         float vLong, float vLat,
                         float driveTorque, float brakePressure, float dt)
    {
        Forces f{};

        // 1. Deflection
        float d = tire.computeDeflection(hubY, groundY);
        tire.deflection = d;

        // 2. Deflection rate: dDot > 0 means compressing.
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

        // 5. Slip ratio (SAE normalization: bounded to ~[-1, 1])
        float wheelSpeed = angularVel * tire.radius;
        constexpr float slipEps = 0.5f;
        float denom = std::max({std::abs(wheelSpeed), std::abs(vLong), slipEps});
        float slipRatio = (wheelSpeed - vLong) / denom;

        // 6. Slip angle
        float slipAngle = std::atan2(vLat, std::max(std::abs(vLong), slipEps));

        lastSlipRatio = slipRatio;
        lastSlipAngle = slipAngle;

        // 7. Forces from brush model
        float Fx = tire.computeLongitudinalForce(slipRatio, Fn);
        float Fy = tire.computeLateralForce(slipAngle, Fn);

        // 8. Friction circle clamp
        tire.frictionCircleClamp(Fx, Fy, Fn);

        f.longitudinalForce = Fx;
        f.lateralForce = Fy;

        // 9. Rolling resistance — applied directly to vehicle body
        f.rollingResistance = tire.computeRollingResistance(Fn, vLong);

        // 10. Integrate wheel angular velocity
        float regSign = angularVel / (std::abs(angularVel) + 0.2f);
        float bearingTorque = -bearingFrictionNm * regSign;
        float tireReaction  = -Fx * tire.radius;

        float unbrakeTorque = driveTorque + bearingTorque + tireReaction;

        float brakeCap = brakeTorqueFromPressure(brakePressure);
        float netTorque;
        if (brakeCap > 0.f && std::abs(unbrakeTorque) <= brakeCap
            && std::abs(angularVel) < 1.f)
        {
            netTorque = 0.f;
            angularVel = 0.f;
        } else {
            float brakeTorque = -brakeCap * regSign;
            netTorque = unbrakeTorque + brakeTorque;
        }

        angularVel += netTorque / I * dt;

        return f;
    }

    // Ground speed from wheel rotation (for speedometer)
    float groundSpeed() const { return angularVel * tire.radius; }
};
