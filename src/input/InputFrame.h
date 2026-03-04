#pragma once

/// Snapshot of player input for a single simulation step.
/// Produced by IInputProvider; consumed by IVehicleDynamics and Engine.
struct InputFrame {
    float steer     { 0.f };   ///< -1 = full left, +1 = full right
    float throttle  { 0.f };   ///< 0..1
    float brake     { 0.f };   ///< 0..1
    float handbrake { 0.f };   ///< 0..1
    bool  resetCar  { false };
    bool  quit      { false };
};
