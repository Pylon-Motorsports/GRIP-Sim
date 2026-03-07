#pragma once
#include "Suspension.h"
#include "Wheel.h"
#include <cmath>
#include <glm/glm.hpp>

// Snapshot of rigid-body state, passed to subframes so they can compute
// velocities at their own wheel positions without reaching into the body.
struct BodyState {
    glm::vec3 pos{0.f};
    glm::vec3 vel{0.f};
    float heading = 0.f, pitch = 0.f, roll = 0.f;
    float forwardSpeed = 0.f, lateralSpeed = 0.f;
    float pitchRate = 0.f, rollRate = 0.f, yawRate = 0.f;

    // Rotate a body-local vector to world frame.
    // Order: roll (about Z) -> pitch (about X) -> heading (about Y)
    glm::vec3 rotateLocal(const glm::vec3& v) const {
        float sr = std::sin(roll), cr = std::cos(roll);
        glm::vec3 r1{v.x * cr - v.y * sr, v.x * sr + v.y * cr, v.z};
        float sp = std::sin(pitch), cp = std::cos(pitch);
        glm::vec3 r2{r1.x, r1.y * cp - r1.z * sp, r1.y * sp + r1.z * cp};
        float sh = std::sin(heading), ch = std::cos(heading);
        return {r2.x * ch + r2.z * sh, r2.y, -r2.x * sh + r2.z * ch};
    }

    glm::vec3 forwardDir() const {
        return {std::sin(heading), 0.f, std::cos(heading)};
    }

    glm::vec3 rightDir() const {
        return {std::cos(heading), 0.f, -std::sin(heading)};
    }
};

// Subframe: intermediate structure between left/right suspension and body.
//
// Each axle (front, rear) has one subframe holding the two corner
// suspensions.  The subframe owns the complete force computation for
// its two wheels: tire forces, velocity decomposition through steering,
// and torque computation at suspension mount points.
//
// Force flow:
//   Body passes BodyState + terrain info + drive/brake
//   -> Subframe computes hub velocities from body angular rates
//   -> Wheel/Tire compute contact forces
//   -> Suspension transmits forces to body at mount points
//   -> Subframe returns total force + torque on body

struct Subframe {
    // Left = index 0, Right = index 1
    Suspension suspension[2];
    Wheel*     wheel[2] = {nullptr, nullptr};  // non-owning pointers, set in init

    // Steering angle for this axle (radians, positive = right).
    // Set by VehiclePhysics each frame. Rear subframe stays at 0.
    float steerAngle = 0.f;

    // Limited-slip differential (0 = open diff, >0 = clutch-type LSD).
    // 1-way: only engages under power (drive torque > 0).
    // Lock ratio determines what fraction of per-wheel torque can transfer.
    float lsdLockRatio = 0.f;  // 0 = open, 0.25 = mild LSD, 1.0 = locked

    struct AxleForces {
        glm::vec3 bodyForce{0.f};   // total force on body (world frame)
        glm::vec3 bodyTorque{0.f};  // torque about CG (x=pitch, y=yaw, z=roll)
    };

    // World position of wheel hub for a given side (0=left, 1=right).
    glm::vec3 wheelWorldPos(int side, const BodyState& bs) const {
        return bs.pos + bs.rotateLocal(wheel[side]->localOffset);
    }

    // World position of suspension mount point for a given side.
    glm::vec3 mountWorldPos(int side, const BodyState& bs) const {
        return bs.pos + bs.rotateLocal(suspension[side].mountPoint);
    }

    // Compute all forces from this axle on the body.
    //
    // The subframe handles everything internally: hub velocities from body
    // angular rates, tire force computation, steering decomposition, and
    // torque calculation at mount points.
    //
    //   bs:             body rigid-body state
    //   groundY:        terrain height at each wheel [left, right]
    //   surfaceNormal:  ground normal at each wheel [left, right]
    //   driveTorque:    engine torque per wheel [left, right]
    //   brakePressure:  hydraulic line pressure per wheel [left, right] (Pa)
    //   dt:             timestep (s)
    AxleForces computeForces(
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
            constexpr float LSD_K = 30.f;  // Nm per rad/s of speed difference
            float lsdCapacity = std::abs(totalDrive) * 0.5f * lsdLockRatio;
            float wL = wheel[0]->angularVel;
            float wR = wheel[1]->angularVel;
            float transfer = std::clamp(LSD_K * (wL - wR), -lsdCapacity, lsdCapacity);
            adjDrive[0] -= transfer;
            adjDrive[1] += transfer;
        }

        for (int s = 0; s < 2; ++s) {
            Wheel& w = *wheel[s];
            glm::vec3 r = w.localOffset;

            // Corner velocity in body frame (CG vel + yaw contribution)
            float cornerVx = bs.lateralSpeed  + bs.yawRate * r.z;
            float cornerVz = bs.forwardSpeed  - bs.yawRate * r.x;

            // Decompose into wheel-local via steering rotation
            float vLong, vLat;
            decomposeVelocity(cornerVx, cornerVz, vLong, vLat);

            // Hub vertical velocity from body angular rates (omega x r)
            float hubVelY = bs.vel.y + bs.rollRate * r.x - bs.pitchRate * r.z;

            // Hub world Y position
            float hubY = wheelWorldPos(s, bs).y;

            // Compute tire forces
            Wheel::Forces wf = w.computeForces(
                hubY, hubVelY, groundY[s], vLong, vLat,
                adjDrive[s], brakePressure[s], dt);

            // --- Vertical forces along surface normal, through suspension ---
            glm::vec3 nf = wf.normalForce * surfaceNormal[s];
            glm::vec3 transF = suspension[s].transmitToBody(nf);
            result.bodyForce  += transF;
            result.bodyTorque += suspension[s].torqueOnBody(transF);

            // --- Horizontal forces: transform wheel-local to body frame ---
            float fLong = wf.longitudinalForce + wf.rollingResistance;
            float fLat  = wf.lateralForce;
            transformForces(fLong, fLat, bodyFx[s], bodyFz[s]);
        }

        // Convert horizontal forces from body frame to world frame
        float totalFx = bodyFx[0] + bodyFx[1];
        float totalFz = bodyFz[0] + bodyFz[1];
        result.bodyForce += bs.forwardDir() * totalFz + bs.rightDir() * totalFx;

        // Pitch torque from forward forces at mount points
        result.bodyTorque.x += pitchTorqueFromForward(bodyFz[0], bodyFz[1]);

        // Roll torque from lateral forces at mount points
        result.bodyTorque.z += rollTorqueFromLateral(bodyFx[0], bodyFx[1]);

        // Yaw torque: tire contact forces create torque about CG.
        // The rigid suspension arm transmits both force AND moment to the
        // mount point, so the effective moment arm is at the wheel position
        // (contact patch), not the mount point.
        result.bodyTorque.y += yawTorqueFromHorizontal(
            bodyFx[0], bodyFz[0], bodyFx[1], bodyFz[1]);

        return result;
    }

private:
    // Decompose body-frame velocity at a corner into wheel-local long/lat.
    void decomposeVelocity(float bodyVx, float bodyVz,
                           float& vLong, float& vLat) const
    {
        float cs = std::cos(steerAngle), ss = std::sin(steerAngle);
        vLong = bodyVz * cs + bodyVx * ss;
        vLat  = bodyVx * cs - bodyVz * ss;
    }

    // Transform wheel-local forces (longitudinal, lateral) to body frame.
    void transformForces(float fLong, float fLat,
                         float& bodyFx, float& bodyFz) const
    {
        float cs = std::cos(steerAngle), ss = std::sin(steerAngle);
        bodyFx = fLong * ss + fLat * cs;
        bodyFz = fLong * cs - fLat * ss;
    }

    // Pitch torque from forward forces at mount points.
    float pitchTorqueFromForward(float bodyFz0, float bodyFz1) const
    {
        return suspension[0].mountPoint.y * bodyFz0
             + suspension[1].mountPoint.y * bodyFz1;
    }

    // Roll torque from lateral forces at mount points.
    float rollTorqueFromLateral(float bodyFx0, float bodyFx1) const
    {
        return -(suspension[0].mountPoint.y * bodyFx0
               + suspension[1].mountPoint.y * bodyFx1);
    }

    // Yaw torque from horizontal forces at wheel contact patch positions.
    float yawTorqueFromHorizontal(float bodyFx0, float bodyFz0,
                                  float bodyFx1, float bodyFz1) const
    {
        float rz0 = wheel[0]->localOffset.z, rx0 = wheel[0]->localOffset.x;
        float rz1 = wheel[1]->localOffset.z, rx1 = wheel[1]->localOffset.x;
        return (rz0 * bodyFx0 - rx0 * bodyFz0)
             + (rz1 * bodyFx1 - rx1 * bodyFz1);
    }
};
