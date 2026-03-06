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

// Single-cylinder engine with centrifugal clutch (go-kart / dune buggy style).
// Single speed — no gearbox.
//
// The centrifugal clutch is modeled as a slip clutch with torque capacity
// that grows with engine RPM.  When engine torque exceeds clutch capacity,
// the excess spins the engine freely (clutch slips).
class Engine {
public:
    TorqueCurve torqueCurve;
    float gearRatio        = 4.0f;    // overall reduction (engine RPM / wheel RPM)
    float idleRpm          = 1200.f;
    float clutchEngageRpm  = 1800.f;  // clutch shoes start to contact drum
    float clutchFullRpm    = 2500.f;  // clutch fully locked
    float clutchMaxTorque  = 120.f;   // max torque clutch can transmit (Nm at crank)
    float rpmLimit         = 6500.f;
    float engineBrakingNm  = 15.f;    // drag torque when throttle off (at crank)
    float engineInertia    = 0.05f;   // kg*m^2 (crank + flywheel)

    float rpm = 1200.f;

    // Compute drive torque delivered to the wheels (total, split equally to
    // both driven wheels by the caller).  Also updates engine RPM.
    //   throttle   : [0, 1]
    //   wheelAngVel: average angular velocity of driven wheels (rad/s)
    //   dt         : physics timestep
    float update(float throttle, float wheelAngVel, float dt)
    {
        constexpr float RAD_TO_RPM = 30.f / 3.14159265f;
        constexpr float RPM_TO_RAD = 3.14159265f / 30.f;

        float drivenRpm = wheelAngVel * RAD_TO_RPM * gearRatio;

        // Clutch engagement: 0 = disengaged, 1 = fully engaged
        float clutchFactor = std::clamp(
            (rpm - clutchEngageRpm) / (clutchFullRpm - clutchEngageRpm),
            0.f, 1.f);

        // Clutch torque capacity (how much torque it can transmit)
        float clutchCapacity = clutchFactor * clutchMaxTorque;

        // Engine torque at crank
        float fullTorque = torqueCurve.lookup(rpm) * throttle;
        float brakeTorque = engineBrakingNm * (1.f - throttle);
        float netCrankTorque = fullTorque - brakeTorque;

        // Torque transmitted through clutch: limited by capacity.
        // Modeled as viscous coupling clamped to friction capacity.
        // Positive clutchTorque = engine drives wheels forward.
        float rpmDiff = rpm - drivenRpm;
        float clutchTorque;

        if (clutchFactor >= 0.999f && std::abs(rpmDiff) < 50.f) {
            // Fully locked: all engine torque passes through
            clutchTorque = netCrankTorque;
        } else {
            // Slipping: viscous coupling proportional to RPM difference,
            // clamped to clutch friction capacity.  This naturally lets
            // the engine RPM decay toward driven RPM (or idle).
            constexpr float VISCOUS_K = 0.1f;  // Nm per RPM of slip
            float viscousTorque = VISCOUS_K * rpmDiff;
            clutchTorque = std::clamp(viscousTorque, -clutchCapacity, clutchCapacity);
        }

        // Torque at wheels (amplified by gear ratio)
        float wheelTorque = clutchTorque * gearRatio;

        // --- Engine RPM update ---
        // Engine torque minus what the clutch takes = net torque on crankshaft
        float engineNetTorque = netCrankTorque - clutchTorque;
        float engineAlpha = engineNetTorque / engineInertia;  // rad/s^2
        rpm += engineAlpha * dt * RAD_TO_RPM;

        // When clutch is locked, track wheel speed directly
        if (clutchFactor >= 0.999f && std::abs(rpmDiff) < 50.f) {
            rpm = drivenRpm;
        }

        // Clamp
        rpm = std::max(rpm, idleRpm);
        if (rpm >= rpmLimit) {
            rpm = rpmLimit;
            wheelTorque = 0.f;  // rev limiter
        }

        return wheelTorque;
    }
};
