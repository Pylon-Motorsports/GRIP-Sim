#pragma once
#include "Tire.hpp"
#include "Scenario.hpp"
#include <vector>

// Pressure-based tire model: multi-sample cylinder contact.
//
// The tire is a rigid cylinder sampled at CIRC_SAMPLES × WIDTH_SAMPLES
// points. Each sample that penetrates terrain contributes a normal force
// of (effectivePressure × cellArea) along the surface normal, plus
// per-sample distributed grip (brush cubic) based on local slip.
//
// Wall contact emerges naturally from side samples hitting pipe geometry.
// No separate wall collision code is needed.

class PressureTire : public Tire {
public:
    // Tire pressure and casing compliance
    float inflatePressurePa = 206843.f;  // 30 psi
    float casingFlex        = 0.15f;     // sidewall compliance factor

    // Per-sample damping (N·s/m per sample, ~60 total / ~5 penetrating samples)
    float contactDampingPerSample = 12.5f;

    // Brush model stiffness (same as BrushTire defaults)
    float slipStiffnessPerArea    = 3.0e6f;   // N/m²/unit_slip
    float latSlipStiffnessPerArea = 1.5e6f;    // N/m²/rad

    static constexpr int CIRC_SAMPLES  = 16;
    static constexpr int WIDTH_SAMPLES = 3;

    // Ground query (set per-frame from VehiclePhysics)
    void setBumps(const std::vector<Bump>* b) { bumps_ = b; }

    bool usesVolumetricContact() const override { return true; }

    VolumetricResult computeVolumetricForces(
        const VolumetricInput& in, float dt) override;

    // Required by pure virtual (unused — volumetric path handles everything)
    void computeSlipForces(float, float, float, float, float,
                           float, float, float& Fx, float& Fy) override
    { Fx = 0.f; Fy = 0.f; }

private:
    const std::vector<Bump>* bumps_ = nullptr;

    // Brush cubic force: same formula as BrushTire
    float brushForce(float slip, float Fn, float Cx) const;
};
