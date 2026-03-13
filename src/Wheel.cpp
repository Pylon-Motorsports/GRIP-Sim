#include "Wheel.hpp"
#include <cmath>
#include <algorithm>

Wheel::Forces Wheel::computeForces(float hubY, float hubVelY, float groundY,
                                    float vLong, float vLat,
                                    float driveTorque, float brakePressure, float dt)
{
    Forces f{};

    // 1. Deflection
    float d = tire->computeDeflection(hubY, groundY);
    tire->deflection = d;

    // 2. Deflection rate: dDot > 0 means compressing.
    float dDot = -hubVelY;

    // 3. Normal force
    float Fn = tire->computeNormalForce(d, dDot);
    tire->normalLoad = Fn;
    f.normalForce = Fn;

    // 4. Contact patch
    tire->updateContactPatch(d);

    // Wheel rotational inertia (solid cylinder approximation)
    float I = 0.5f * massKg * tire->radius * tire->radius;

    if (d <= 0.f) {
        // Airborne: no ground forces, wheel spins freely with drag
        lastSlipRatio = 0.f;
        lastSlipAngle = 0.f;
        float regSign = angularVel / (std::abs(angularVel) + 0.2f);
        float dragTorque = -bearingFrictionNm * regSign;
        angularVel += (driveTorque + dragTorque) / I * dt;
        return f;
    }

    // 5. Slip ratio (SAE normalization: bounded to ~[-1, 1])
    float wheelSpeed = angularVel * tire->radius;
    constexpr float slipEps = 0.5f;
    float denom = std::max({std::abs(wheelSpeed), std::abs(vLong), slipEps});
    float slipRatio = (wheelSpeed - vLong) / denom;

    // 6. Slip angle
    float slipAngle = std::atan2(vLat, std::max(std::abs(vLong), slipEps));

    lastSlipRatio = slipRatio;
    lastSlipAngle = slipAngle;

    // 7. Forces from tire model (brush, LuGre, etc.)
    float Fx, Fy;
    tire->computeSlipForces(slipRatio, slipAngle,
                            vLong, vLat, wheelSpeed,
                            Fn, dt, Fx, Fy);

    f.longitudinalForce = Fx;
    f.lateralForce = Fy;

    // 8. Rolling resistance
    f.rollingResistance = tire->computeRollingResistance(Fn, vLong);

    // 9. Integrate wheel angular velocity
    float regSign = angularVel / (std::abs(angularVel) + 0.2f);
    float bearingTorque = -bearingFrictionNm * regSign;
    float tireReaction  = -Fx * tire->radius;

    float unbrakeTorque = driveTorque + bearingTorque + tireReaction;

    float brakeCap = brakeTorqueFromPressure(brakePressure);
    float netTorque;
    if (brakeCap > 0.f && std::abs(unbrakeTorque) <= brakeCap
        && std::abs(angularVel) < 1.f)
    {
        netTorque = 0.f;
        angularVel = 0.f;
    } else {
        float brakeTorqueVal = -brakeCap * regSign;
        netTorque = unbrakeTorque + brakeTorqueVal;
    }

    angularVel += netTorque / I * dt;

    return f;
}

Wheel::VolumetricForces Wheel::computeVolumetricForces(
    const Tire::VolumetricInput& volIn,
    float driveTorque, float brakePressure, float dt)
{
    VolumetricForces vf;

    auto vr = tire->computeVolumetricForces(volIn, dt);
    vf.worldForce = vr.worldForce;
    vf.normalLoad = vr.normalLoad;
    vf.rollingResistForce = vr.rollingResistForce;

    // Store for HUD
    tire->normalLoad = vr.normalLoad;
    tire->contactPatchArea = vr.contactArea;

    // Integrate wheel angular velocity
    float I = 0.5f * massKg * tire->radius * tire->radius;
    float regSign = angularVel / (std::abs(angularVel) + 0.2f);
    float bearingTorque = -bearingFrictionNm * regSign;
    float tireReaction = vr.wheelTorqueReaction;

    float unbrakeTorque = driveTorque + bearingTorque + tireReaction;
    float brakeCap = brakeTorqueFromPressure(brakePressure);
    float netTorque;
    if (brakeCap > 0.f && std::abs(unbrakeTorque) <= brakeCap
        && std::abs(angularVel) < 1.f)
    {
        netTorque = 0.f;
        angularVel = 0.f;
    } else {
        float brakeTorqueVal = -brakeCap * regSign;
        netTorque = unbrakeTorque + brakeTorqueVal;
    }
    angularVel += netTorque / I * dt;

    return vf;
}
