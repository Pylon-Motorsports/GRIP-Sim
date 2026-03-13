#include "BrushTire.hpp"
#include <cmath>
#include <algorithm>

void BrushTire::computeSlipForces(float slipRatio, float slipAngle,
                                   float vLong, float vLat, float /*wheelSpeed*/,
                                   float Fn, float dt,
                                   float& Fx, float& Fy)
{
    Fx = computeLongitudinalForce(slipRatio, Fn);
    Fy = computeLateralForce(slipAngle, Fn);

    // Stiction: at low contact-patch velocity with no significant wheel spin,
    // use implicit static friction to bring velocity to zero.
    {
        float patchSpeed = std::sqrt(vLong * vLong + vLat * vLat);
        constexpr float STICTION_SPEED = 1.0f;
        if (patchSpeed < STICTION_SPEED && std::abs(slipRatio) < 0.15f) {
            float wheelMass = std::max(Fn / 9.81f, 1.f);
            float sFx = -vLong * wheelMass / dt;
            float sFy = -vLat  * wheelMass / dt;
            frictionCircleClamp(sFx, sFy, Fn);
            float t = patchSpeed / STICTION_SPEED;
            Fx = sFx + (Fx - sFx) * t;
            Fy = sFy + (Fy - sFy) * t;
        }
    }

    // Friction circle with slip-dependent limit
    float slideFactor = std::min(std::abs(slipRatio) * 3.f, 1.f);
    float muEff = mu + (muSliding - mu) * slideFactor;
    float maxF2 = muEff * Fn;
    float combined2 = std::sqrt(Fx * Fx + Fy * Fy);
    if (combined2 > maxF2 && combined2 > 0.f) {
        float scale = maxF2 / combined2;
        Fx *= scale;
        Fy *= scale;
    }
}

float BrushTire::computeLongitudinalForce(float slipRatio, float Fn) const
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
        float slidingF = muSliding * Fn;
        float fade = 1.f / absKs;
        magnitude = slidingF + (maxF - slidingF) * fade;
    }

    return (slipRatio >= 0.f) ? magnitude : -magnitude;
}

float BrushTire::computeLateralForce(float slipAngle, float Fn) const
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
        float slidingF = muSliding * Fn;
        float fade = 1.f / absKs;
        magnitude = slidingF + (maxF - slidingF) * fade;
    }

    return (slipAngle >= 0.f) ? -magnitude : magnitude;
}
