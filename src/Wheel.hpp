#pragma once
#include "Tire.hpp"
#include <glm/glm.hpp>
#include <memory>

// Wheel (rim + brake rotor) paired 1:1 with a Tire.
class Wheel {
public:
    float massKg   = 20.f;
    glm::vec3 localOffset{0.f};  // position relative to body CG

    // Rotational state
    float angularVel = 0.f;  // rad/s (positive = forward rolling)

    // Per-step outputs
    float lastSlipRatio = 0.f;
    float lastSlipAngle = 0.f;

    // Brakes and bearing
    float bearingFrictionNm = 2.f;
    float brakePistonAreaM2  = 0.0016f;
    float brakePadMu         = 0.4f;
    float brakeDiscRadiusM   = 0.14f;

    float brakeTorqueFromPressure(float pressurePa) const {
        return pressurePa * brakePistonAreaM2 * brakePadMu * brakeDiscRadiusM * 2.f;
    }

    std::unique_ptr<Tire> tire;

    struct Forces {
        float normalForce;
        float longitudinalForce;
        float lateralForce;
        float rollingResistance;
    };

    Forces computeForces(float hubY, float hubVelY, float groundY,
                         float vLong, float vLat,
                         float driveTorque, float brakePressure, float dt);

    struct VolumetricForces {
        glm::vec3 worldForce{0.f};
        float normalLoad = 0.f;
        float rollingResistForce = 0.f;
    };

    VolumetricForces computeVolumetricForces(
        const Tire::VolumetricInput& volIn,
        float driveTorque, float brakePressure, float dt);

    float groundSpeed() const { return angularVel * tire->radius; }
};
