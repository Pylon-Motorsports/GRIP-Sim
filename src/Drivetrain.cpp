#include "Drivetrain.hpp"
#include <algorithm>

float Drivetrain::engineTorqueAtRpm(float rpm) const {
    if (rpm < idleRpm)
        return maxEngineTorqueNm * (rpm / idleRpm);  // ramp from 0
    if (rpm <= 6400.f)
        return maxEngineTorqueNm;                     // flat torque plateau
    if (rpm <= redlineRpm) {
        // Linear drop-off above peak
        float t = (rpm - 6400.f) / (redlineRpm - 6400.f);
        return maxEngineTorqueNm * (1.f - 0.3f * t);
    }
    return 0.f;  // beyond redline
}

void Drivetrain::autoShift(float rpm) {
    if (rpm >= shiftUpRpm && currentGear < NUM_GEARS - 1)
        currentGear++;
    else if (rpm <= shiftDownRpm && currentGear > 0)
        currentGear--;
}

void Drivetrain::reset() {
    currentGear   = 0;
    engineRpm     = idleRpm;
    smoothFwdSpeed_ = 0.f;
}

Drivetrain::Output Drivetrain::update(float throttle, float vehicleFwdSpeed,
                                       float tireRadius) {
    throttle = std::clamp(throttle, 0.f, 1.f);

    float gearRatio    = gearRatios[currentGear];
    float overallRatio = gearRatio * finalDriveRatio;

    // Estimate engine RPM from vehicle speed.
    // Smooth the speed input to prevent wild RPM swings during tumbling
    // (body frame rotation causes forward speed projection to oscillate).
    smoothFwdSpeed_ += (vehicleFwdSpeed - smoothFwdSpeed_) * 0.15f;
    float wheelRps = std::abs(smoothFwdSpeed_) / std::max(tireRadius, 0.01f);
    float wheelRpm = wheelRps * 60.f / (2.f * 3.14159f);
    engineRpm = std::max(idleRpm, wheelRpm * overallRatio);

    autoShift(engineRpm);
    // Recompute after potential shift
    gearRatio    = gearRatios[currentGear];
    overallRatio = gearRatio * finalDriveRatio;
    engineRpm    = std::max(idleRpm, wheelRpm * overallRatio);

    float engineTorque = engineTorqueAtRpm(engineRpm) * throttle;

    // Engine braking: when throttle is low, the engine acts as a pump.
    // Drag torque scales linearly with RPM (friction + pumping losses).
    if (throttle < 0.05f) {
        float rpmNorm = (engineRpm - idleRpm) / (redlineRpm - idleRpm);
        rpmNorm = std::clamp(rpmNorm, 0.f, 1.f);
        float brakeTorque = engineBrakingNm * rpmNorm;
        // Oppose engine rotation (which tracks forward speed)
        engineTorque -= brakeTorque;
    }

    float axleTorque   = engineTorque * overallRatio * efficiency;

    // LSD split: for now, equal split (50/50).
    // With a real LSD, we'd bias toward the slower wheel using lsdTorqueBias.
    // Without per-wheel spin tracking, we default to 50/50.
    float baseTorque = axleTorque * 0.5f;

    return { baseTorque, baseTorque };
}
