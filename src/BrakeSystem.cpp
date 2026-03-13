#include "BrakeSystem.hpp"

BrakeSystem::Output BrakeSystem::update(float brakePedal, float handBrake) const {
    brakePedal = std::clamp(brakePedal, 0.f, 1.f);
    handBrake  = std::clamp(handBrake, 0.f, 1.f);

    float totalBrake = maxBrakeTorqueNm * brakePedal;
    float frontTotal = totalBrake * frontBias;
    float rearTotal  = totalBrake * (1.f - frontBias);

    // Handbrake adds to rear only
    float hbPerWheel = handBrakeTorqueNm * handBrake * 0.5f;

    return {
        .fl = frontTotal * 0.5f,
        .fr = frontTotal * 0.5f,
        .rl = rearTotal * 0.5f + hbPerWheel,
        .rr = rearTotal * 0.5f + hbPerWheel,
    };
}
