#pragma once
#include <array>
#include <cmath>

class Drivetrain {
public:
    // Engine
    float maxEngineTorqueNm = 205.f;   // BRZ peak torque
    float idleRpm           = 800.f;
    float redlineRpm        = 7400.f;
    float shiftUpRpm        = 6500.f;
    float shiftDownRpm      = 2500.f;

    // Gearbox (BRZ 6-speed)
    static constexpr int NUM_GEARS = 6;
    std::array<float, NUM_GEARS> gearRatios = {
        3.626f, 2.188f, 1.541f, 1.213f, 1.000f, 0.767f
    };
    float finalDriveRatio   = 4.100f;
    float efficiency        = 0.90f;   // drivetrain losses
    float engineBrakingNm   = 40.f;   // peak engine braking torque at redline

    // LSD
    float lsdTorqueBias     = 2.0f;    // 1.0 = open diff, higher = tighter LSD

    // State
    int   currentGear       = 0;       // 0-based index
    float engineRpm         = 800.f;

    struct Output {
        float leftTorqueNm  = 0.f;
        float rightTorqueNm = 0.f;
    };

    // Compute per-wheel drive torques.
    // throttle: 0..1
    // vehicleFwdSpeed: forward speed in m/s (used for RPM estimation)
    // tireRadius: wheel radius in m
    Output update(float throttle, float vehicleFwdSpeed, float tireRadius);

    void reset();

private:
    float smoothFwdSpeed_ = 0.f;   // smoothed forward speed (prevents RPM spikes during tumbling)
    float engineTorqueAtRpm(float rpm) const;
    void autoShift(float rpm);
};
