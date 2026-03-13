#pragma once
#include "Tire.hpp"

// Brush (cubic) tire model: smooth force-slip curve with natural saturation.
class BrushTire : public Tire {
public:
    float slipStiffnessPerArea        = 3.0e6f;  // N / m² / unit_slip (longitudinal)
    float lateralSlipStiffnessPerArea = 1.5e6f;  // N / m² / rad (lateral)

    void computeSlipForces(float slipRatio, float slipAngle,
                           float vLong, float vLat, float wheelSpeed,
                           float Fn, float dt,
                           float& Fx, float& Fy) override;

    float computeLongitudinalForce(float slipRatio, float Fn) const;
    float computeLateralForce(float slipAngle, float Fn) const;
};
