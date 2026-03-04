#pragma once
#include <glm/glm.hpp>
#include "VehicleState.h"
#include "input/InputFrame.h"

/// Module interface for vehicle physics.
/// Swap implementations to change handling model (arcade, bicycle, multi-body, etc.).
class IVehicleDynamics {
public:
    virtual ~IVehicleDynamics() = default;

    /// Reset to a given world position and heading (radians).
    virtual void reset(glm::vec3 position, float headingRad) = 0;

    /// Advance simulation by dt seconds given player input.
    /// Called at fixed physics rate (120 Hz).
    virtual void integrate(const InputFrame& input, float dt) = 0;

    /// Read-only access to current vehicle state.
    [[nodiscard]] virtual const VehicleState& state() const = 0;
};
