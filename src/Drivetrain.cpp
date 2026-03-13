#include "Drivetrain.hpp"
#include <cmath>
#include <algorithm>

void Drivetrain::init()
{
    engine.torqueCurve = TorqueCurve({
        { 1000.f,  80.f },
        { 2000.f, 140.f },
        { 3000.f, 185.f },
        { 3500.f, 205.f },
        { 4500.f, 200.f },
        { 5500.f, 175.f },
        { 6500.f, 140.f },
        { 7400.f, 110.f },
    });
    engine.gearRatios[0] = 3.9f * 3.63f;  // 14.2 — 1st
    engine.gearRatios[1] = 3.9f * 2.19f;  // 8.5  — 2nd
    engine.gearRatios[2] = 3.9f * 1.54f;  // 6.0  — 3rd
    engine.gearRatios[3] = 3.9f * 1.13f;  // 4.4  — 4th
    engine.gearRatios[4] = 3.9f * 0.91f;  // 3.5  — 5th
    engine.numGears      = 5;
    engine.currentGear   = 0;
    engine.idleRpm         = 1200.f;
    engine.clutchEngageRpm = 1800.f;
    engine.clutchFullRpm   = 2500.f;
    engine.rpmLimit        = 7400.f;
    engine.engineBrakingNm = 40.f;
    engine.rpm             = 1200.f;
}

float Drivetrain::computeDriveTorque(float throttle, bool clutchIn,
                                      const Wheel& rearL, const Wheel& rearR, float dt)
{
    float rearAngVel = (rearL.angularVel + rearR.angularVel) * 0.5f;
    float totalDrive = engine.update(throttle, clutchIn, rearAngVel, dt);
    return totalDrive * 0.5f;
}

void Drivetrain::computeBrakePressure(float pedal,
                                       float& frontPressure, float& rearPressure) const
{
    frontPressure = pedal * MAX_BRAKE_PRESSURE * BRAKE_BIAS_FRONT;
    float rearPedal = (pedal <= PROP_VALVE_KNEE)
        ? pedal
        : PROP_VALVE_KNEE + (pedal - PROP_VALVE_KNEE) * PROP_VALVE_SLOPE;
    rearPressure = rearPedal * MAX_BRAKE_PRESSURE * (1.f - BRAKE_BIAS_FRONT);
}

float Drivetrain::computeSteerAngle(float steerInput, float currentAngle,
                                     float forwardSpeed, float dt) const
{
    float input = applyDeadzone(steerInput, STEER_DEADZONE);
    float speedFactor = 1.f / (1.f + std::abs(forwardSpeed) / 30.f);
    float target = input * MAX_STEER_ANGLE * speedFactor;
    float maxDelta = MAX_STEER_RATE * dt;
    float delta = target - currentAngle;
    return currentAngle + std::clamp(delta, -maxDelta, maxDelta);
}

void Drivetrain::reset()
{
    engine.rpm = engine.idleRpm;
    engine.currentGear = 0;
}

float Drivetrain::applyDeadzone(float value, float deadzone)
{
    if (std::abs(value) < deadzone) return 0.f;
    float sign = value > 0.f ? 1.f : -1.f;
    return sign * (std::abs(value) - deadzone) / (1.f - deadzone);
}
