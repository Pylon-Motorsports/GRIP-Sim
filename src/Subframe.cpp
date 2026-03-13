#include "Subframe.hpp"
#include <cmath>
#include <algorithm>

Subframe::AxleForces Subframe::computeForces(
    const BodyState& bs,
    const float groundY[2],
    const glm::vec3 surfaceNormal[2],
    const float driveTorque[2],
    const float brakePressure[2], float dt)
{
    AxleForces result;
    float bodyFx[2], bodyFz[2];

    // LSD torque redistribution (1-way: power-on only)
    float adjDrive[2] = { driveTorque[0], driveTorque[1] };
    float totalDrive = driveTorque[0] + driveTorque[1];
    if (lsdLockRatio > 0.f && totalDrive > 10.f) {
        constexpr float LSD_K = 30.f;
        float lsdCapacity = std::abs(totalDrive) * 0.5f * lsdLockRatio;
        float wL = wheel[0]->angularVel;
        float wR = wheel[1]->angularVel;
        float transfer = std::clamp(LSD_K * (wL - wR), -lsdCapacity, lsdCapacity);
        adjDrive[0] -= transfer;
        adjDrive[1] += transfer;
    }

    bool volumetric = wheel[0]->tire->usesVolumetricContact();

    if (volumetric) {
        // --- Volumetric path: forces already in world frame ---
        glm::vec3 fwd = bs.forwardDir();
        glm::vec3 rht = bs.rightDir();

        for (int s = 0; s < 2; ++s) {
            Wheel& w = *wheel[s];
            glm::vec3 r = w.localOffset;

            float cornerVx = bs.lateralSpeed  + bs.yawRate * r.z;
            float cornerVz = bs.forwardSpeed  - bs.yawRate * r.x;
            float vLong, vLat;
            decomposeVelocity(cornerVx, cornerVz, vLong, vLat);

            glm::vec3 hubVel = bs.vel;
            hubVel.x += bs.yawRate * r.z;
            hubVel.y += bs.rollRate * r.x - bs.pitchRate * r.z;
            hubVel.z -= bs.yawRate * r.x;

            float cs = std::cos(steerAngle), ss = std::sin(steerAngle);
            glm::vec3 wheelFwd  = fwd * cs + rht * ss;
            glm::vec3 wheelAxle = rht * cs - fwd * ss;

            Tire::VolumetricInput volIn;
            volIn.hubWorldPos   = wheelWorldPos(s, bs);
            volIn.axleWorldDir  = wheelAxle;
            volIn.forwardDir    = wheelFwd;
            volIn.hubVelocity   = hubVel;
            volIn.vLong         = vLong;
            volIn.vLat          = vLat;
            volIn.wheelAngularVel = w.angularVel;

            auto vf = w.computeVolumetricForces(
                volIn, adjDrive[s], brakePressure[s], dt);

            result.bodyForce += vf.worldForce;
            result.bodyForce += wheelFwd * vf.rollingResistForce;

            glm::vec3 leverArm = wheelWorldPos(s, bs) - bs.pos;
            result.bodyTorque += glm::cross(leverArm, vf.worldForce);

            // Store slip metrics for HUD (zero when airborne)
            if (vf.normalLoad > 0.f) {
                constexpr float slipEps = 0.5f;
                float wheelSpd = w.angularVel * w.tire->radius;
                float denom = std::max({std::abs(wheelSpd), std::abs(vLong), slipEps});
                w.lastSlipRatio = (wheelSpd - vLong) / denom;
                w.lastSlipAngle = std::atan2(vLat, std::max(std::abs(vLong), slipEps));
            } else {
                w.lastSlipRatio = 0.f;
                w.lastSlipAngle = 0.f;
            }
        }
    } else {
        // --- Single-point path (BrushTire / LuGreTire) ---
        for (int s = 0; s < 2; ++s) {
            Wheel& w = *wheel[s];
            glm::vec3 r = w.localOffset;

            float cornerVx = bs.lateralSpeed  + bs.yawRate * r.z;
            float cornerVz = bs.forwardSpeed  - bs.yawRate * r.x;
            float vLong, vLat;
            decomposeVelocity(cornerVx, cornerVz, vLong, vLat);

            float hubVelY = bs.vel.y + bs.rollRate * r.x - bs.pitchRate * r.z;
            float hubY = wheelWorldPos(s, bs).y;

            Wheel::Forces wf = w.computeForces(
                hubY, hubVelY, groundY[s], vLong, vLat,
                adjDrive[s], brakePressure[s], dt);

            glm::vec3 nf = wf.normalForce * surfaceNormal[s];
            glm::vec3 transF = suspension[s].transmitToBody(nf);
            result.bodyForce  += transF;
            result.bodyTorque += suspension[s].torqueOnBody(transF);

            float fLong = wf.longitudinalForce + wf.rollingResistance;
            float fLat  = wf.lateralForce;
            transformForces(fLong, fLat, bodyFx[s], bodyFz[s]);
        }

        float totalFx = bodyFx[0] + bodyFx[1];
        float totalFz = bodyFz[0] + bodyFz[1];
        result.bodyForce += bs.forwardDir() * totalFz + bs.rightDir() * totalFx;

        result.bodyTorque.x += pitchTorqueFromForward(bodyFz[0], bodyFz[1]);
        result.bodyTorque.z += rollTorqueFromLateral(bodyFx[0], bodyFx[1]);
        result.bodyTorque.y += yawTorqueFromHorizontal(
            bodyFx[0], bodyFz[0], bodyFx[1], bodyFz[1]);
    }

    return result;
}

void Subframe::decomposeVelocity(float bodyVx, float bodyVz,
                                  float& vLong, float& vLat) const
{
    float cs = std::cos(steerAngle), ss = std::sin(steerAngle);
    vLong = bodyVz * cs + bodyVx * ss;
    vLat  = bodyVx * cs - bodyVz * ss;
}

void Subframe::transformForces(float fLong, float fLat,
                                float& bodyFx, float& bodyFz) const
{
    float cs = std::cos(steerAngle), ss = std::sin(steerAngle);
    bodyFx = fLong * ss + fLat * cs;
    bodyFz = fLong * cs - fLat * ss;
}

float Subframe::pitchTorqueFromForward(float bodyFz0, float bodyFz1) const
{
    return suspension[0].mountPoint.y * bodyFz0
         + suspension[1].mountPoint.y * bodyFz1;
}

float Subframe::rollTorqueFromLateral(float bodyFx0, float bodyFx1) const
{
    return -(suspension[0].mountPoint.y * bodyFx0
           + suspension[1].mountPoint.y * bodyFx1);
}

float Subframe::yawTorqueFromHorizontal(float bodyFx0, float bodyFz0,
                                         float bodyFx1, float bodyFz1) const
{
    float rz0 = wheel[0]->localOffset.z, rx0 = wheel[0]->localOffset.x;
    float rz1 = wheel[1]->localOffset.z, rx1 = wheel[1]->localOffset.x;
    return (rz0 * bodyFx0 - rx0 * bodyFz0)
         + (rz1 * bodyFx1 - rx1 * bodyFz1);
}
