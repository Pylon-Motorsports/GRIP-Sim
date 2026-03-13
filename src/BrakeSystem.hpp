#pragma once
#include <algorithm>

class BrakeSystem {
public:
    float maxBrakeTorqueNm  = 4000.f;  // total across all 4 wheels at pedal=1.0
    float frontBias         = 0.60f;   // 60% front, 40% rear
    float handBrakeTorqueNm = 1500.f;  // total across both rear wheels

    struct Output {
        float fl = 0.f, fr = 0.f;     // front left/right brake torque (N·m)
        float rl = 0.f, rr = 0.f;     // rear left/right brake torque (N·m)
    };

    // Compute per-wheel brake torques.
    // brakePedal: 0..1, handBrake: 0..1
    Output update(float brakePedal, float handBrake) const;
};
