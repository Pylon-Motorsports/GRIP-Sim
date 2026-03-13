#include "Tire.hpp"
#include <cmath>
#include <algorithm>

float Tire::computeDeflection(float hubY, float groundY) const
{
    float d = (groundY + radius) - hubY;
    if (d <= 0.f) return 0.f;
    return d;
}

float Tire::computeNormalForce(float d, float dDot) const
{
    if (d <= 0.f) return 0.f;
    float springD = std::min(d, maxDeflection);
    float f = radialStiffness * springD + radialDamping * dDot;
    // Progressive bump stop beyond max deflection (quadratic)
    if (d > maxDeflection) {
        float over = d - maxDeflection;
        f += bumpStopStiffness * over * over;
    }
    return std::max(f, 0.f);
}

void Tire::updateContactPatch(float d)
{
    if (d <= 0.f) {
        contactPatchLength = 0.f;
        contactPatchArea = 0.f;
        return;
    }
    float dClamped = std::min(d, maxDeflection);
    float disc = 2.f * radius * dClamped - dClamped * dClamped;
    contactPatchLength = 2.f * std::sqrt(std::max(disc, 0.f));
    contactPatchArea = width * contactPatchLength;
}

void Tire::frictionCircleClamp(float& longForce, float& latForce, float Fn) const
{
    float maxF = mu * Fn;
    float combined = std::sqrt(longForce * longForce + latForce * latForce);
    if (combined > maxF && combined > 0.f) {
        float scale = maxF / combined;
        longForce *= scale;
        latForce  *= scale;
    }
}

float Tire::computeRollingResistance(float Fn, float forwardSpeed) const
{
    if (Fn <= 0.f) return 0.f;
    constexpr float eps = 0.05f;
    float regSign = forwardSpeed / (std::abs(forwardSpeed) + eps);
    return -rollingResistCoeff * Fn * regSign;
}
