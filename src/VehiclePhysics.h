#pragma once
#include "Wheel.h"
#include "Subframe.h"
#include "Engine.h"
#include "Input.h"
#include "Vehicle.h"
#include "Scenario.h"
#include <glm/glm.hpp>
#include <vector>

class VehiclePhysics {
public:
    void init();
    void update(float dt, const InputState& input);

    // Export state for rendering
    void fillVehicle(Vehicle& veh) const;

    // Set the active bump list (nullptr or empty = flat ground)
    void setBumps(const std::vector<Bump>* bumps) { bumps_ = bumps; }

    // Reset vehicle to starting position
    void reset();

    // Read-only accessors for HUD / tests
    float getForwardSpeed()   const { return forwardSpeed_; }
    float getEngineRpm()      const { return engine_.rpm; }
    float getEngineRpmLimit() const { return engine_.rpmLimit; }
    int   getGear()           const { return engine_.getGear(); }
    float getHeading()        const { return heading_; }

    // Speedometer: average speed from front (non-driven) wheels
    float getFrontWheelSpeed() const {
        return (wheels_[0].groundSpeed() + wheels_[1].groundSpeed()) * 0.5f;
    }

private:
    static constexpr float GRAVITY = 9.81f;

    // Body linear state
    float     bodyMassKg_ = 300.f;
    glm::vec3 pos_{0.f};
    glm::vec3 vel_{0.f};
    float     heading_ = 0.f;
    float     forwardSpeed_ = 0.f;

    // Body angular state (pitch, roll, yaw)
    float pitch_     = 0.f;   // radians, positive = nose up
    float roll_      = 0.f;   // radians, positive = right side down
    float pitchRate_ = 0.f;   // rad/s
    float rollRate_  = 0.f;   // rad/s
    float yawRate_   = 0.f;   // rad/s, positive = turning right

    // Moments of inertia (approximate box: I = m*(a^2+b^2)/12)
    float pitchInertia_ = 0.f;  // computed in init
    float rollInertia_  = 0.f;
    float yawInertia_   = 0.f;

    // Steering
    static constexpr float MAX_STEER_ANGLE = 0.61f;   // ~35 degrees
    static constexpr float STEER_DEADZONE  = 0.05f;
    static constexpr float MAX_STEER_RATE  = 2.5f;    // rad/s at the wheels (steering rack limit)

    // Brakes: pedal [0,1] → hydraulic pressure (Pa), biased front/rear
    static constexpr float MAX_BRAKE_PRESSURE = 4.0e6f;  // 40 bar
    static constexpr float BRAKE_BIAS_FRONT   = 0.60f;   // 60% front, 40% rear

    // Drivetrain
    Engine     engine_;
    Wheel      wheels_[4];
    Subframe   front_;   // wheels 0 (left), 1 (right)
    Subframe   rear_;    // wheels 2 (left), 3 (right)

    // Active bump list (owned externally)
    const std::vector<Bump>* bumps_ = nullptr;

    // Build a BodyState snapshot for passing to subframes
    BodyState buildBodyState(float lateralSpeed) const;

    float totalMass() const;
    static float applyDeadzone(float value, float deadzone);
};
