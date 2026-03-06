#pragma once
#include "Wheel.h"
#include "Suspension.h"
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

    // Body angular state (pitch and roll)
    float pitch_     = 0.f;   // radians, positive = nose up
    float roll_      = 0.f;   // radians, positive = right side down
    float pitchRate_ = 0.f;   // rad/s
    float rollRate_  = 0.f;   // rad/s

    // Moments of inertia (approximate box: I = m*(a^2+b^2)/12)
    float pitchInertia_ = 0.f;  // computed in init
    float rollInertia_  = 0.f;

    // Drivetrain
    Engine     engine_;
    Wheel      wheels_[4];
    Suspension suspensions_[4];
    // Active bump list (owned externally)
    const std::vector<Bump>* bumps_ = nullptr;

    float wheelGroundY(int i) const;
    glm::vec3 wheelWorldPos(int i) const;
    glm::vec3 mountWorldPos(int i) const;
    glm::vec3 rotateByBody(const glm::vec3& v) const;
    glm::vec3 forwardDir() const;
    float     totalMass() const;
};
