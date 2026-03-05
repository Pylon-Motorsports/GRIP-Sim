#pragma once

/// Physics parameters tuned to a 2022 Subaru WRX (6MT, AWD).
/// All SI units unless noted.
struct MultiBodyParams {
    // --- Chassis ---
    float massKg            { 1500.f };   ///< Curb weight (~1491 kg actual)
    float cgHeightM         { 0.50f };    ///< CG height above ground
    float Iroll             { 450.f };    ///< Moment of inertia, roll  (about fwd axis, kg·m²)
    float Ipitch            { 1200.f };   ///< Moment of inertia, pitch (about lateral axis, kg·m²)
    float Iyaw              { 2000.f };   ///< Moment of inertia, yaw   (about up axis, kg·m²)

    // --- Geometry ---
    float wheelbaseM        { 2.670f };   ///< WRX wheelbase
    float lfDistM           { 1.180f };   ///< CG to front axle
    float lrDistM           { 1.490f };   ///< CG to rear axle
    float halfTrackFrontM   { 0.770f };   ///< Half-track width, front
    float halfTrackRearM    { 0.778f };   ///< Half-track width, rear

    // --- Wheel / tyre ---
    float wheelRadiusM      { 0.317f };   ///< 225/45R17 loaded radius
    float wheelInertiaKgM2  { 1.2f };     ///< Spin inertia per wheel

    // --- Suspension (same for all 4 corners, tuned front/rear separately) ---
    float springRateFrontNm { 24000.f };  ///< Front spring rate (N/m)
    float springRateRearNm  { 20000.f };  ///< Rear spring rate (N/m)
    float damperFrontNsm    { 3000.f };   ///< Front damper (N·s/m)
    float damperRearNsm     { 2800.f };   ///< Rear damper (N·s/m)
    float springRestLengthM { 0.35f };    ///< Unloaded spring length
    float maxBumpM          { 0.10f };    ///< Max suspension compression from static
    float maxReboundM       { 0.12f };    ///< Max suspension extension from static

    // --- Drivetrain (AWD, 6MT) ---
    float maxTorqueNm       { 350.f };    ///< Peak engine torque (258 lb·ft)
    float idleRpm           { 800.f };
    float maxRpm            { 6500.f };
    float upshiftRpm        { 6000.f };
    float downshiftRpm      { 2000.f };
    int   numGears          { 6 };
    float gearRatios[6]     { 3.636f, 2.235f, 1.521f, 1.137f, 0.971f, 0.756f };
    float finalDriveRatio   { 4.111f };
    float drivetrainEff     { 0.88f };
    float frontTorqueFrac   { 0.45f };    ///< AWD split: 45% front, 55% rear

    // --- Steering ---
    float maxSteerAngleRad  { 0.42f };    ///< ~24 degrees lock-to-lock per side

    // --- Tyres ---
    float tyreMu            { 1.05f };    ///< Combined peak friction coefficient
    float tyreLatStiff      { 75000.f };  ///< Cornering stiffness per tyre (N/rad)
    float tyreLongPeakSlip  { 0.12f };    ///< Slip ratio at peak longitudinal force

    // --- Brakes ---
    float maxBrakeN         { 20000.f };  ///< Total brake force (all 4 wheels)
    float brakeBiasFront    { 0.60f };    ///< Fraction of brake force on front axle

    // --- Aerodynamics ---
    float Cd                { 0.35f };
    float frontalAreaM2     { 2.20f };
    float airDensityKgM3    { 1.225f };
};
