#include "Engine.hpp"
#include <cmath>
#include <algorithm>
#include <cstddef>

TorqueCurve::TorqueCurve(std::initializer_list<Point> pts) : size_(pts.size())
{
    size_t i = 0;
    for (auto& p : pts) { if (i < MAX_PTS) pts_[i++] = p; }
}

float TorqueCurve::lookup(float rpm) const
{
    if (size_ == 0) return 0.f;
    if (rpm <= pts_[0].rpm)        return pts_[0].torqueNm;
    if (rpm >= pts_[size_-1].rpm)  return pts_[size_-1].torqueNm;

    for (size_t i = 1; i < size_; ++i) {
        if (rpm <= pts_[i].rpm) {
            float t = (rpm - pts_[i-1].rpm) / (pts_[i].rpm - pts_[i-1].rpm);
            return pts_[i-1].torqueNm + t * (pts_[i].torqueNm - pts_[i-1].torqueNm);
        }
    }
    return pts_[size_-1].torqueNm;
}

float Engine::update(float throttle, bool clutchIn, float wheelAngVel, float dt)
{
    constexpr float RAD_TO_RPM = 30.f / 3.14159265f;

    float ratio = gearRatios[currentGear];
    float drivenRpm = wheelAngVel * RAD_TO_RPM * ratio;

    // --- Auto shift logic ---
    shiftTimer_ -= dt;
    if (shiftTimer_ <= 0.f) {
        if (rpm >= shiftUpRpm && currentGear < numGears - 1) {
            currentGear++;
            shiftTimer_ = shiftCooldown;
            ratio = gearRatios[currentGear];
            drivenRpm = wheelAngVel * RAD_TO_RPM * ratio;
        } else if (rpm <= shiftDownRpm && currentGear > 0) {
            currentGear--;
            shiftTimer_ = shiftCooldown;
            ratio = gearRatios[currentGear];
            drivenRpm = wheelAngVel * RAD_TO_RPM * ratio;
        }
    }

    // --- Clutch engagement ---
    float clutchFactor;
    if (clutchIn) {
        clutchFactor = 0.f;
    } else {
        clutchFactor = std::clamp(
            (rpm - clutchEngageRpm) / (clutchFullRpm - clutchEngageRpm),
            0.f, 1.f);
    }

    float clutchCapacity = clutchFactor * clutchMaxTorque;

    // --- Engine torque ---
    float fullTorque = torqueCurve.lookup(rpm) * throttle;
    float brakeTorque = engineBrakingNm * (1.f - throttle);
    float netCrankTorque = fullTorque - brakeTorque;

    // --- Clutch torque transmission ---
    float rpmDiff = rpm - drivenRpm;
    float clutchTorque;

    if (clutchFactor >= 0.999f && std::abs(rpmDiff) < 50.f) {
        clutchTorque = netCrankTorque;
    } else {
        constexpr float VISCOUS_K = 0.1f;
        float viscousTorque = VISCOUS_K * rpmDiff;
        clutchTorque = std::clamp(viscousTorque, -clutchCapacity, clutchCapacity);
    }

    float wheelTorque = clutchTorque * ratio;

    // --- Engine RPM update ---
    float engineNetTorque = netCrankTorque - clutchTorque;
    float engineAlpha = engineNetTorque / engineInertia;
    rpm += engineAlpha * dt * RAD_TO_RPM;

    // Lock to wheel speed when fully engaged
    if (clutchFactor >= 0.999f && std::abs(rpmDiff) < 50.f) {
        rpm = drivenRpm;
    }

    rpm = std::max(rpm, idleRpm);
    if (rpm >= rpmLimit) {
        rpm = rpmLimit;
        wheelTorque = 0.f;
    }

    return wheelTorque;
}

float Engine::update(float throttle, float wheelAngVel, float dt)
{
    return update(throttle, false, wheelAngVel, dt);
}
