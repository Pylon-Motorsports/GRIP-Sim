#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>

// Piecewise-linear torque curve: RPM -> torque (Nm).
// Points must be sorted ascending by RPM.
class TorqueCurve {
public:
    struct Point { float rpm; float torqueNm; };

    TorqueCurve() = default;

    TorqueCurve(std::initializer_list<Point> pts) : size_(pts.size())
    {
        size_t i = 0;
        for (auto& p : pts) { if (i < MAX_PTS) pts_[i++] = p; }
    }

    float lookup(float rpm) const
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

    float maxRpm() const { return size_ == 0 ? 0.f : pts_[size_-1].rpm; }

private:
    static constexpr size_t MAX_PTS = 16;
    Point  pts_[MAX_PTS]{};
    size_t size_ = 0;
};

// Multi-gear automatic transmission with centrifugal clutch.
// - Automatic upshift at shiftUpRpm, downshift when RPM drops below shiftDownRpm
// - Centrifugal clutch in 1st gear prevents stalling
// - Manual clutch override (clutchIn) disengages drivetrain for free-revving/launches
class Engine {
public:
    TorqueCurve torqueCurve;

    // Gear ratios (final drive × gear ratio). Index 0 = 1st gear.
    static constexpr int MAX_GEARS = 6;
    float gearRatios[MAX_GEARS] = { 10.0f, 6.5f, 4.5f, 3.5f, 2.8f, 0.f };
    int   numGears              = 5;
    int   currentGear           = 0;  // 0-based (0 = 1st)

    // Shift points
    float shiftUpRpm    = 5800.f;   // auto upshift threshold
    float shiftDownRpm  = 2500.f;   // auto downshift threshold
    float shiftCooldown = 0.3f;     // seconds between shifts (prevents hunting)

    // Clutch
    float idleRpm          = 1200.f;
    float clutchEngageRpm  = 1800.f;
    float clutchFullRpm    = 2500.f;
    float clutchMaxTorque  = 120.f;

    // Engine
    float rpmLimit         = 6500.f;
    float engineBrakingNm  = 15.f;
    float engineInertia    = 0.05f;  // kg*m^2

    float rpm = 1200.f;

    // Read-only state for HUD
    int getGear() const { return currentGear + 1; }  // 1-based for display

    // Compute drive torque delivered to wheels.
    //   throttle   : [0, 1]
    //   clutchIn   : true = clutch disengaged (user override)
    //   wheelAngVel: average angular velocity of driven wheels (rad/s)
    //   dt         : physics timestep
    float update(float throttle, bool clutchIn, float wheelAngVel, float dt)
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
            } else if (rpm <= shiftDownRpm && currentGear > 0 && throttle < 0.5f) {
                currentGear--;
                shiftTimer_ = shiftCooldown;
                ratio = gearRatios[currentGear];
                drivenRpm = wheelAngVel * RAD_TO_RPM * ratio;
            }
        }

        // --- Clutch engagement ---
        // Centrifugal clutch: engages with RPM (prevents stalling).
        // Manual override: clutchIn fully disengages.
        float clutchFactor;
        if (clutchIn) {
            clutchFactor = 0.f;  // user disengaged
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
            wheelTorque = 0.f;  // rev limiter
        }

        return wheelTorque;
    }

    // Legacy single-arg update for backward compatibility (no clutch override)
    float update(float throttle, float wheelAngVel, float dt)
    {
        return update(throttle, false, wheelAngVel, dt);
    }

private:
    float shiftTimer_ = 0.f;
};
