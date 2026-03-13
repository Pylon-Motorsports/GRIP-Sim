#pragma once
#include "VehiclePhysicsComponent.hpp"

// Slip and force outputs — available after update() for rendering, audio, telemetry.
struct TireOutput {
    float slipAngleRad  = 0.f;   // lateral slip angle (rad), signed
    float slipRatio     = 0.f;   // longitudinal slip ratio (unitless), signed
    float lateralForceN = 0.f;   // lateral force magnitude (N)
    float longForceN    = 0.f;   // longitudinal force magnitude (N)
    float normalLoadN   = 0.f;   // vertical load on this tire (N)
    bool  sliding       = false; // true when combined slip exceeds ~80% of grip circle
};

class Tire : public VehiclePhysicsComponent {
public:
    float radius      = 0.31f;   // m
    float width       = 0.215f;  // m
    bool  steered     = false;   // informational (routing handled externally)
    bool  driven      = false;   // informational (routing handled externally)

    Tire(std::string name, glm::vec3 attachPt, bool steered_, bool driven_);

    // Per-wheel inputs — set by Drivetrain, BrakeSystem, Vehicle before update()
    void  setDriveTorque(float nm) { driveTorqueNm_ = nm; }
    void  setBrakeTorque(float nm) { brakeTorqueNm_ = nm; }
    void  setSteerAngle(float rad) { steerAngle_    = rad; }

    float driveTorque() const { return driveTorqueNm_; }
    float brakeTorque() const { return brakeTorqueNm_; }
    float steerAngle()  const { return steerAngle_; }

    // Slip/force outputs — valid after update()
    const TireOutput& tireOutput() const { return tireOutput_; }
    float slipAngle()  const { return tireOutput_.slipAngleRad; }
    float slipRatio()  const { return tireOutput_.slipRatio; }
    bool  isSliding()  const { return tireOutput_.sliding; }

protected:
    Drawable generateDrawable(const ComponentInput& input) const override;

    float driveTorqueNm_ = 0.f;
    float brakeTorqueNm_ = 0.f;
    float steerAngle_    = 0.f;
    TireOutput tireOutput_;
};
