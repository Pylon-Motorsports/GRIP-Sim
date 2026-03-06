#pragma once
#include <cmath>
#include <algorithm>

// Tire modeled as a radial spring-damper with geometric contact patch
// and brush-type slip model.
//
// The single source of truth is radial deflection 'd': how much the
// tire's circular cross-section overlaps the ground plane.
//
//   d = (groundY + R) - hubY
//
// Physical limits:
//   d <= 0:      no contact, zero forces
//   0 < d < dMax: sidewall deforming, spring-damper active
//   d >= dMax:    rim contact — tire can't compress further
//
// dMax is a physical tire property (not a safety clamp). It represents
// the point where the rigid rim meets the ground through fully-compressed
// sidewall rubber. All computations are parameterized on d ∈ [0, dMax],
// so the bound is native to the model.

class Tire {
public:
    // --- Geometry ---
    float radius        = 0.30f;   // unloaded outer radius (m)
    float width         = 0.20f;   // tread width (m)
    float maxDeflection = 0.025f;  // dMax: rim contact limit (m), ~25mm for off-road

    // --- Radial spring-damper (sidewall + inflation pressure) ---
    float radialStiffness = 200000.f;  // N/m (typical car tire 150k-250k)
    float radialDamping   = 500.f;     // N·s/m (internal rubber damping)

    // --- Friction ---
    float mu                  = 1.0f;    // peak friction coefficient (dry tarmac)
    float rollingResistCoeff  = 0.015f;  // Crr

    // --- Brush model ---
    // Stiffness per unit contact patch area — controls how quickly grip
    // builds with slip before saturation. Larger patch = higher stiffness
    // = more linear grip range before sliding.
    float slipStiffnessPerArea        = 3.0e6f;  // N / m² / unit_slip (longitudinal)
    float lateralSlipStiffnessPerArea = 1.5e6f;  // N / m² / rad (lateral, lower → later saturation)

    // --- Per-step state (read by external code for HUD, etc.) ---
    float deflection         = 0.f;  // current d (m)
    float normalLoad         = 0.f;  // current Fn (N)
    float contactPatchLength = 0.f;  // chord length (m)
    float contactPatchArea   = 0.f;  // width × chord (m²)

    // Compute deflection from hub height and ground height.
    // Returns d clamped to [0, dMax].
    float computeDeflection(float hubY, float groundY) const
    {
        float d = (groundY + radius) - hubY;
        if (d <= 0.f) return 0.f;
        if (d >= maxDeflection) return maxDeflection;
        return d;
    }

    // Compute normal force from deflection and its rate of change.
    //   d:    current deflection [0, dMax]
    //   dDot: positive = compressing (hub moving toward ground)
    // Returns force magnitude (N), >= 0 (tire can push, not pull).
    float computeNormalForce(float d, float dDot) const
    {
        if (d <= 0.f) return 0.f;
        float f = radialStiffness * d + radialDamping * dDot;
        return std::max(f, 0.f);
    }

    // Compute contact patch geometry from circular segment at deflection d.
    // Chord length: L = 2 * sqrt(2Rd - d²)
    // Patch area:   A = width * L
    void updateContactPatch(float d)
    {
        if (d <= 0.f) {
            contactPatchLength = 0.f;
            contactPatchArea = 0.f;
            return;
        }
        float disc = 2.f * radius * d - d * d;
        contactPatchLength = 2.f * std::sqrt(std::max(disc, 0.f));
        contactPatchArea = width * contactPatchLength;
    }

    // Brush tire longitudinal force from slip ratio.
    //
    // Uses the cubic brush model for a smooth peak and saturation:
    //   |F| = mu*Fn * (3*|ks| - 3*|ks|² + |ks|³)   for |ks| < 1
    //   |F| = mu*Fn                                    for |ks| >= 1
    // where k = Cx / (3 * mu * Fn) and Cx = stiffnessPerArea * patchArea.
    //
    // At small slip this is linear: F ≈ Cx * s (proportional to patch area).
    // At large slip it saturates at mu * Fn.
    float computeLongitudinalForce(float slipRatio, float Fn) const
    {
        if (Fn <= 0.f || contactPatchArea <= 0.f) return 0.f;

        float Cx = slipStiffnessPerArea * contactPatchArea;
        float maxF = mu * Fn;
        float k = Cx / (3.f * maxF);
        float absKs = std::abs(k * slipRatio);

        float magnitude;
        if (absKs < 1.f) {
            magnitude = maxF * (3.f * absKs - 3.f * absKs * absKs
                                + absKs * absKs * absKs);
        } else {
            magnitude = maxF;
        }

        return (slipRatio >= 0.f) ? magnitude : -magnitude;
    }

    // Lateral force from slip angle using brush model.
    // slipAngle in radians; positive = velocity to right of heading.
    // Returns force that opposes the slip (negative when slipAngle positive).
    float computeLateralForce(float slipAngle, float Fn) const
    {
        if (Fn <= 0.f || contactPatchArea <= 0.f) return 0.f;

        float Cy = lateralSlipStiffnessPerArea * contactPatchArea;
        float maxF = mu * Fn;
        float k = Cy / (3.f * maxF);
        float absKs = std::abs(k * slipAngle);

        float magnitude;
        if (absKs < 1.f) {
            magnitude = maxF * (3.f * absKs - 3.f * absKs * absKs
                                + absKs * absKs * absKs);
        } else {
            magnitude = maxF;
        }

        // Opposes slip: positive slipAngle → negative force
        return (slipAngle >= 0.f) ? -magnitude : magnitude;
    }

    // Clamp combined longitudinal + lateral forces to friction circle.
    // Ensures total tire force magnitude doesn't exceed mu * Fn.
    void frictionCircleClamp(float& longForce, float& latForce, float Fn) const
    {
        float maxF = mu * Fn;
        float combined = std::sqrt(longForce * longForce + latForce * latForce);
        if (combined > maxF && combined > 0.f) {
            float scale = maxF / combined;
            longForce *= scale;
            latForce  *= scale;
        }
    }

    // Rolling resistance: opposes motion, proportional to normal load.
    // Uses a tight regularization (eps=0.05) since this is a small force
    // and doesn't need the wider band that prevents oscillation in larger forces.
    float computeRollingResistance(float Fn, float forwardSpeed) const
    {
        if (Fn <= 0.f) return 0.f;
        constexpr float eps = 0.05f;
        float regSign = forwardSpeed / (std::abs(forwardSpeed) + eps);
        return -rollingResistCoeff * Fn * regSign;
    }
};
