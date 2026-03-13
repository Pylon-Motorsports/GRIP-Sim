#pragma once
#include <cstddef>
#include <initializer_list>

// Piecewise-linear torque curve: RPM -> torque (Nm).
class TorqueCurve {
public:
    struct Point { float rpm; float torqueNm; };

    TorqueCurve() = default;
    TorqueCurve(std::initializer_list<Point> pts);

    float lookup(float rpm) const;
    float maxRpm() const { return size_ == 0 ? 0.f : pts_[size_-1].rpm; }

private:
    static constexpr size_t MAX_PTS = 16;
    Point  pts_[MAX_PTS]{};
    size_t size_ = 0;
};

// Multi-gear automatic transmission with centrifugal clutch.
class Engine {
public:
    TorqueCurve torqueCurve;

    static constexpr int MAX_GEARS = 6;
    float gearRatios[MAX_GEARS] = { 10.0f, 6.5f, 4.5f, 3.5f, 2.8f, 0.f };
    int   numGears              = 5;
    int   currentGear           = 0;

    float shiftUpRpm    = 6800.f;
    float shiftDownRpm  = 2800.f;
    float shiftCooldown = 0.3f;

    float idleRpm          = 1200.f;
    float clutchEngageRpm  = 1800.f;
    float clutchFullRpm    = 2500.f;
    float clutchMaxTorque  = 350.f;

    float rpmLimit         = 6500.f;
    float engineBrakingNm  = 15.f;
    float engineInertia    = 0.15f;

    float rpm = 1200.f;

    int getGear() const { return currentGear + 1; }

    float update(float throttle, bool clutchIn, float wheelAngVel, float dt);
    float update(float throttle, float wheelAngVel, float dt);

private:
    float shiftTimer_ = 0.f;
};
