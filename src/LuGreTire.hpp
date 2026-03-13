#pragma once
#include "Tire.hpp"

// LuGre (Lund-Grenoble) friction model for tires.
class LuGreTire : public Tire {
public:
    float sigma0 = 80000.f;   // bristle stiffness (N/m)
    float sigma1 = 500.f;     // bristle damping (N·s/m)
    float sigma2 = 0.005f;    // viscous friction coefficient (N·s/m)
    float muStatic  = 1.25f;  // static friction coefficient
    float muDynamic = 1.15f;  // Coulomb (dynamic) friction coefficient
    float stribeckVel = 5.f;  // Stribeck velocity (m/s)

    void computeSlipForces(float slipRatio, float slipAngle,
                           float vLong, float vLat, float wheelSpeed,
                           float Fn, float dt,
                           float& Fx, float& Fy) override;

    void resetState();

private:
    float zx_ = 0.f;
    float zy_ = 0.f;
};
