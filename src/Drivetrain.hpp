#pragma once
#include "Engine.hpp"
#include "Wheel.hpp"

// Drivetrain: engine + transmission + brake hydraulics + steering input.
// RWD layout: drive torque goes to rear axle only.
struct Drivetrain {
    static constexpr float MAX_STEER_ANGLE = 0.61f;
    static constexpr float STEER_DEADZONE  = 0.05f;
    static constexpr float MAX_STEER_RATE  = 2.5f;

    static constexpr float MAX_BRAKE_PRESSURE = 4.0e6f;
    static constexpr float BRAKE_BIAS_FRONT   = 0.60f;
    static constexpr float PROP_VALVE_KNEE    = 0.4f;
    static constexpr float PROP_VALVE_SLOPE   = 0.3f;

    Engine engine;

    void init();

    float computeDriveTorque(float throttle, bool clutchIn,
                             const Wheel& rearL, const Wheel& rearR, float dt);

    void computeBrakePressure(float pedal,
                              float& frontPressure, float& rearPressure) const;

    float computeSteerAngle(float steerInput, float currentAngle,
                            float forwardSpeed, float dt) const;

    void reset();

    static float applyDeadzone(float value, float deadzone);
};
