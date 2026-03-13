#pragma once
#include "Suspension.hpp"
#include "Wheel.hpp"
#include <cmath>
#include <glm/glm.hpp>

// Snapshot of rigid-body state, passed to subframes.
struct BodyState {
    glm::vec3 pos{0.f};
    glm::vec3 vel{0.f};
    float heading = 0.f, pitch = 0.f, roll = 0.f;
    float forwardSpeed = 0.f, lateralSpeed = 0.f;
    float pitchRate = 0.f, rollRate = 0.f, yawRate = 0.f;

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
struct Subframe {
    Suspension suspension[2];
    Wheel*     wheel[2] = {nullptr, nullptr};

    float steerAngle = 0.f;
    float lsdLockRatio = 0.f;

    struct AxleForces {
        glm::vec3 bodyForce{0.f};
        glm::vec3 bodyTorque{0.f};
    };

    glm::vec3 wheelWorldPos(int side, const BodyState& bs) const {
        return bs.pos + bs.rotateLocal(wheel[side]->localOffset);
    }

    glm::vec3 mountWorldPos(int side, const BodyState& bs) const {
        return bs.pos + bs.rotateLocal(suspension[side].mountPoint);
    }

    AxleForces computeForces(
        const BodyState& bs,
        const float groundY[2],
        const glm::vec3 surfaceNormal[2],
        const float driveTorque[2],
        const float brakePressure[2], float dt);

private:
    void decomposeVelocity(float bodyVx, float bodyVz,
                           float& vLong, float& vLat) const;
    void transformForces(float fLong, float fLat,
                         float& bodyFx, float& bodyFz) const;
    float pitchTorqueFromForward(float bodyFz0, float bodyFz1) const;
    float rollTorqueFromLateral(float bodyFx0, float bodyFx1) const;
    float yawTorqueFromHorizontal(float bodyFx0, float bodyFz0,
                                  float bodyFx1, float bodyFz1) const;
};
