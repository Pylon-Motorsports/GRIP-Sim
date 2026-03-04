#pragma once

/// Tuning constants for a WRC-class rally car approximation.
/// Users can provide their own VehicleParams instance to IVehicleDynamics to
/// tune the handling without subclassing.
struct VehicleParams {
    float massKg             { 1350.f };
    float cgHeightM          { 0.45f };
    float wheelbaseM         { 2.53f };
    float lfDistM            { 1.05f };  ///< CG to front axle
    float lrDistM            { 1.48f };  ///< CG to rear axle
    float momentOfInertiaKgM2{ 1500.f };
    float wheelRadiusM       { 0.32f };

    float maxTorqueNm        { 450.f };
    float drivetrainEfficiency{ 0.88f };
    float diffRatio          { 3.7f };
    float gearRatios[6]      { 3.6f, 2.1f, 1.45f, 1.1f, 0.88f, 0.73f };
    int   numGears           { 6 };
    float idleRpm            { 800.f };
    float maxRpm             { 7500.f };
    float upshiftRpm         { 6800.f };
    float downshiftRpm       { 2500.f };

    float maxSteerAngleRad   { 0.45f }; ///< ~26 degrees
    float corneringStiffFront{ 85000.f }; ///< N/rad
    float corneringStiffRear { 75000.f }; ///< N/rad
    float peakNormalForceN   { 8000.f };  ///< Tyre saturation threshold

    float maxBrakeForceN     { 18000.f };
    float brakeBiasFront     { 0.60f };  ///< Fraction of brake force on front axle

    float Cd                 { 0.35f };
    float frontalAreaM2      { 2.2f };
    float airDensityKgM3     { 1.225f };
};
