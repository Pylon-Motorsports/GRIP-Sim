#pragma once

/// Physics parameters tuned to a 2022 Subaru BRZ (6MT, RWD).
/// All SI units unless noted.
struct MultiBodyParams {
    // --- Chassis ---
    float massKg            { 1275.f };   ///< Curb weight (~1275 kg actual)
    float cgHeightM         { 0.46f };    ///< CG height above ground (low boxer engine)
    float Iroll             { 350.f };    ///< Moment of inertia, roll  (about fwd axis, kg·m²)
    float Ipitch            { 1000.f };   ///< Moment of inertia, pitch (about lateral axis, kg·m²)
    float Iyaw              { 1700.f };   ///< Moment of inertia, yaw   (about up axis, kg·m²)

    // --- Geometry ---
    float wheelbaseM        { 2.575f };   ///< BRZ wheelbase
    float lfDistM           { 1.210f };   ///< CG to front axle (front-mid engine, 53% front)
    float lrDistM           { 1.365f };   ///< CG to rear axle
    float halfTrackFrontM   { 0.760f };   ///< Half-track width, front
    float halfTrackRearM    { 0.775f };   ///< Half-track width, rear

    // --- Wheel / tyre ---
    float wheelRadiusM      { 0.318f };   ///< 215/40R18 loaded radius
    float wheelInertiaKgM2  { 1.1f };     ///< Spin inertia per wheel

    // --- Suspension (front/rear independently tuned) ---
    float springRateFrontNm { 22000.f };  ///< Front spring rate (N/m)
    float springRateRearNm  { 26000.f };  ///< Rear spring rate (N/m) — stiffer rear for RWD
    float damperFrontNsm    { 2800.f };   ///< Front damper (N·s/m)
    float damperRearNsm     { 3200.f };   ///< Rear damper (N·s/m)
    float springRestLengthM { 0.35f };    ///< Unloaded spring length
    float maxBumpM          { 0.10f };    ///< Max suspension compression from rest
    float maxReboundM       { 0.12f };    ///< Max suspension extension from rest
    float swayBarFrontNmRad { 15000.f };  ///< Front sway bar stiffness (N·m/rad)
    float swayBarRearNmRad  { 10000.f };  ///< Rear sway bar stiffness (N·m/rad)
    float bumpStopRateNm    { 200000.f }; ///< Bump stop stiffness (N/m) — progressive rubber
    float bumpStopExponent  { 3.f };      ///< Bump stop force exponent (cubic = progressive)

    // --- Drivetrain (RWD, 6MT) ---
    float maxTorqueNm       { 250.f };    ///< Peak engine torque (184 lb·ft)
    float idleRpm           { 850.f };
    float maxRpm            { 7400.f };
    float upshiftRpm        { 6800.f };
    float downshiftRpm      { 2200.f };
    int   numGears          { 6 };
    float gearRatios[6]     { 3.626f, 2.188f, 1.541f, 1.213f, 1.000f, 0.767f };
    float finalDriveRatio   { 4.100f };
    float drivetrainEff     { 0.90f };    ///< RWD: simpler drivetrain, less loss
    float frontTorqueFrac   { 0.00f };    ///< RWD: 0% front, 100% rear
    float engineBrakeTorqueNm { 40.f };   ///< Engine braking torque at max RPM (N·m)

    // --- Steering ---
    float maxSteerAngleRad  { 0.44f };    ///< ~25 degrees lock-to-lock per side
    float steerSpeedFactor  { 80.f };    ///< Speed (m/s) at which effective steer halves (speed-sensitive steering)

    // --- Tyres ---
    float tyreMu            { 1.10f };    ///< Peak friction at reference load (Michelin Pilot Sport)
    float tyreLoadSens      { 0.00004f }; ///< Load sensitivity: μ drops per N above ref load
    float tyreLatStiff      { 70000.f };  ///< Cornering stiffness per tyre (N/rad)
    float tyreLongPeakSlip  { 0.10f };    ///< Slip ratio at peak longitudinal force
    float tyreRelaxLenM     { 0.30f };    ///< Lateral relaxation length (m) — first-order lag on slip angle
    float tyrePneumaticTrailM { 0.035f }; ///< Pneumatic trail (m) — self-aligning torque = trail * Fy

    // --- Brakes ---
    float maxBrakeN         { 18000.f };  ///< Total brake force (all 4 wheels)
    float brakeBiasFront    { 0.62f };    ///< Fraction of brake force on front axle

    // --- Aerodynamics ---
    float Cd                { 0.28f };    ///< BRZ is more aerodynamic than WRX
    float frontalAreaM2     { 1.96f };    ///< Lower, smaller frontal area
    float airDensityKgM3    { 1.225f };

    // --- Rolling resistance ---
    float rollingResistCrr  { 0.012f };   ///< Per-wheel Crr (sport tyre on tarmac)
};
