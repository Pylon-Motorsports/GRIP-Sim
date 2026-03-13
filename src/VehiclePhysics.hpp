#pragma once
#include "Wheel.hpp"
#include "BrushTire.hpp"
#include "LuGreTire.hpp"
#include "Subframe.hpp"
#include "Body.hpp"
#include "Drivetrain.hpp"
#include "Input.hpp"
#include "Vehicle.hpp"
#include "Terrain.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <memory>

// VehiclePhysics: thin orchestrator that wires Body, Drivetrain,
// Subframes, and Wheels together each timestep.
class VehiclePhysics {
public:
    void init();
    void update(float dt, const InputState& input);

    // Export state for rendering
    void fillVehicle(Vehicle& veh) const;

    // Set terrain for ground queries (nullptr = flat ground)
    void setTerrain(const Terrain* t) { terrain_ = t; }

    // Reset vehicle to starting position
    void reset();

    // Read-only accessors for HUD / tests
    float getForwardSpeed()   const { return body_.forwardSpeed; }
    float getEngineRpm()      const { return drivetrain_.engine.rpm; }
    float getEngineRpmLimit() const { return drivetrain_.engine.rpmLimit; }
    int   getGear()           const { return drivetrain_.engine.getGear(); }
    float getHeading()        const { return body_.heading; }
    float getYawRate()        const { return body_.yawRate; }
    glm::vec3 getVelocity()  const { return body_.vel; }

    // Speedometer: average speed from front (non-driven) wheels
    float getFrontWheelSpeed() const {
        return (wheels_[0].groundSpeed() + wheels_[1].groundSpeed()) * 0.5f;
    }

private:
    Body       body_;
    Drivetrain drivetrain_;
    Wheel      wheels_[4];
    Subframe   front_;   // wheels 0 (left), 1 (right)
    Subframe   rear_;    // wheels 2 (left), 3 (right)

    // Active terrain (owned externally)
    const Terrain* terrain_ = nullptr;

    float totalMass() const;
};
