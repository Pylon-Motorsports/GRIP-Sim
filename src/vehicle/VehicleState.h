#pragma once
#include <glm/glm.hpp>

/// Full observable state of the simulated vehicle.
/// Produced each physics step by IVehicleDynamics; consumed by renderer, camera, pacenotes.
struct VehicleState {
    glm::vec3 position       { 0.f, 0.f, 0.f }; ///< World space, Y-up
    glm::vec3 velocity       { 0.f, 0.f, 0.f }; ///< World space m/s
    float     headingRad     { 0.f };            ///< Yaw: angle from +Z axis, clockwise = positive
    float     speedMs        { 0.f };            ///< Scalar forward speed
    float     slipAngleRad   { 0.f };            ///< Front axle slip angle
    float     throttle       { 0.f };            ///< 0..1 applied this step
    float     brake          { 0.f };            ///< 0..1 applied this step
    float     steer          { 0.f };            ///< -1..+1 applied this step
    float     weightFront    { 0.5f };           ///< Fraction of total weight on front axle
    float     engineRpm      { 800.f };
    int       currentGear    { 1 };
    int       segmentIndex   { 0 };              ///< Which GRIP segment the car occupies
    float     segmentProgress{ 0.f };            ///< 0..1 along current segment
    float     odoMeters      { 0.f };            ///< Total distance driven in metres
    float     rollRad        { 0.f };            ///< Body roll (rad): positive = right side down
    float     pitchRad       { 0.f };            ///< Body pitch (rad): positive = nose down
    float     terrainRollRad { 0.f };            ///< Ground slope roll at CG position (rad)
    float     terrainPitchRad{ 0.f };            ///< Ground slope pitch at CG position (rad)
    glm::vec3 wheelPos[4]   {};                  ///< World-space wheel hub centres [FL,FR,RL,RR]
    float     wheelGroundHeight[4] {};             ///< Ground Y at each wheel contact [FL,FR,RL,RR]
    float     suspLength[4]  {};                   ///< Current spring length per corner [FL,FR,RL,RR]
    glm::vec3 suspTopPos[4]  {};                   ///< World-space suspension top attachment [FL,FR,RL,RR]
};
