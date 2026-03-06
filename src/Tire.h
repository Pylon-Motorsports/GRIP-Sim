#pragma once
#include <algorithm>
#include <cmath>

// Tire contact patch model using regularized Coulomb friction.
//
// Instead of a discontinuous sign(v) function that goes to 0 at v=0,
// we use v / (|v| + epsilon) which creates a steep linear ramp near
// zero speed.  This naturally models static friction: at standstill,
// the tire provides a restoring force proportional to speed, up to
// the static friction limit (mu_s * N).
//
// No LOCK_SPEED threshold, no special "locked" mode — the physics
// handles the transition continuously.
//
// For now, 100% traction means mu_s is realistic (~1.0 on dry tarmac)
// but since all our forces are well within mu_s * N, the tire never
// exceeds its traction limit (no longitudinal slip yet).

class Tire {
public:
    float radius              = 0.30f;   // outer radius (m)
    float halfWidth           = 0.10f;   // half of tread width (m)
    float staticFrictionCoeff = 1.0f;    // mu_s (dry tarmac, 100% traction)
    float rollingResistCoeff  = 0.015f;  // C_rr (rolling resistance)

    // Regularization parameter: controls the width of the friction
    // transition near zero speed.  Must be wider than the max per-step
    // velocity change from braking to avoid sign-flip oscillation.
    // At 120 Hz with ~10 m/s^2 max decel: epsilon >= 0.08 m/s.
    float epsilon             = 0.2f;

    // Current normal load on this tire (N), set by suspension each step
    float normalLoad          = 0.f;

    // Compute the longitudinal force at the contact patch.
    //
    //   netWheelTorque: sum of drive, brake, and bearing torques (Nm)
    //                   (positive = accelerating forward)
    //   forwardSpeed:   vehicle forward speed at this corner (m/s)
    //
    // Returns: force (N), positive = forward
    float computeForce(float netWheelTorque, float forwardSpeed) const
    {
        // Force demanded by wheel torques
        float demandedForce = netWheelTorque / radius;

        // Rolling resistance: always opposes motion, proportional to normal load
        // Uses the same regularized sign to avoid discontinuity at zero
        float regSign = regularizedSign(forwardSpeed);
        float rollingResist = rollingResistCoeff * normalLoad * regSign;

        // Total force the tire needs to transmit
        float totalForce = demandedForce - rollingResist;

        // Traction limit: max force the contact patch can sustain
        float tractionLimit = staticFrictionCoeff * normalLoad;

        // Clamp to traction limit (when exceeded = wheel spin, not modeled yet)
        return std::clamp(totalForce, -tractionLimit, tractionLimit);
    }

    // Max force this tire can provide (for external queries)
    float maxTractionForce() const {
        return staticFrictionCoeff * normalLoad;
    }

private:
    // Continuous approximation of sign(v): v / (|v| + epsilon)
    // At v=0: returns 0
    // At |v| >> epsilon: returns ±1
    // Near zero: linear ramp with slope 1/epsilon (acts as viscous damping)
    float regularizedSign(float v) const {
        return v / (std::abs(v) + epsilon);
    }
};
