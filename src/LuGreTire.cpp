#include "LuGreTire.hpp"
#include <cmath>
#include <algorithm>

void LuGreTire::computeSlipForces(float /*slipRatio*/, float /*slipAngle*/,
                                   float vLong, float vLat, float wheelSpeed,
                                   float Fn, float dt,
                                   float& Fx, float& Fy)
{
    if (Fn <= 0.f || contactPatchArea <= 0.f) {
        Fx = Fy = 0.f;
        zx_ = 0.f;
        zy_ = 0.f;
        return;
    }

    float vsx = vLong - wheelSpeed;
    float vsy = vLat;
    float vs = std::sqrt(vsx * vsx + vsy * vsy);

    // Stribeck function
    float vRatio = vs / stribeckVel;
    float gv = Fn * (muDynamic + (muStatic - muDynamic) * std::exp(-vRatio * vRatio));

    // Semi-implicit Euler bristle dynamics
    float decay = (vs > 1e-6f) ? sigma0 * vs / gv : 0.f;
    float denom = 1.f + decay * dt;
    zx_ = (zx_ + vsx * dt) / denom;
    zy_ = (zy_ + vsy * dt) / denom;

    // Clamp bristle deflection
    float maxZ = gv / sigma0;
    float zMag = std::sqrt(zx_ * zx_ + zy_ * zy_);
    if (zMag > maxZ && zMag > 0.f) {
        float scale = maxZ / zMag;
        zx_ *= scale;
        zy_ *= scale;
    }

    // Bristle rate
    float dzx = vsx - decay * zx_;
    float dzy = vsy - decay * zy_;

    // Force on vehicle (reaction)
    Fx = -(sigma0 * zx_ + sigma1 * dzx + sigma2 * vsx);
    Fy = -(sigma0 * zy_ + sigma1 * dzy + sigma2 * vsy);

    frictionCircleClamp(Fx, Fy, Fn);
}

void LuGreTire::resetState()
{
    zx_ = 0.f;
    zy_ = 0.f;
}
