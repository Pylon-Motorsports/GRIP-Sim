#include "SimpleTire.hpp"
#include <cmath>
#include <algorithm>

SimpleTire::SimpleTire(std::string name, glm::vec3 attachPt, bool steered_, bool driven_)
    : Tire(std::move(name), attachPt, steered_, driven_) {}

ComponentOutput SimpleTire::compute(const ComponentInput& input) {
    tireOutput_ = {};
    if (input.normalLoad <= 0.f) return {};

    tireOutput_.normalLoadN = input.normalLoad;

    float sa   = steerAngle();
    float grip = input.surfaceGrip;

    // --- Velocity in tire-local frame (body frame rotated by steer angle) ---
    glm::vec3 bodyVel = glm::transpose(input.bodyRotation) * input.velocity;
    float cs = std::cos(sa), sn = std::sin(sa);
    float vFwd =  bodyVel.z * cs + bodyVel.x * sn;
    float vLat = -bodyVel.z * sn + bodyVel.x * cs;
    float speed = std::sqrt(vFwd * vFwd + vLat * vLat);

    // --- Slip angle (lateral) ---
    float slipAngle = 0.f;
    if (speed > 0.1f)
        slipAngle = std::atan2(vLat, std::abs(vFwd));
    tireOutput_.slipAngleRad = slipAngle;

    // --- Slip ratio (longitudinal) ---
    // wheelSpeed = vFwd when no wheelspin; slip = (wheelSpeed - vFwd) / max(|wheelSpeed|, |vFwd|)
    float wheelLinearSpeed = vFwd; // no wheel spin model yet — approximate from drive torque
    float driveForceRaw = driveTorqueNm_ / std::max(radius, 0.01f);
    // Estimate wheel speed perturbation from torque imbalance
    float slipRatio = 0.f;
    if (speed > 0.5f) {
        float maxLon = input.normalLoad * longMuStatic * grip;
        if (maxLon > 1.f)
            slipRatio = std::clamp(driveForceRaw / maxLon, -1.f, 1.f);
    }
    tireOutput_.slipRatio = slipRatio;

    // --- Combined slip saturation (grip circle) ---
    float absSlipAngle = std::abs(slipAngle);
    float absSR = std::abs(slipRatio);
    // Normalized combined slip: sqrt(lat^2 + lon^2) where each is 0..1 of its peak
    float latNorm = std::min(absSlipAngle / 0.14f, 1.f); // ~8° peak slip angle
    float lonNorm = absSR;
    float combinedSlip = std::sqrt(latNorm * latNorm + lonNorm * lonNorm);
    bool sliding = combinedSlip > 0.8f;
    tireOutput_.sliding = sliding;

    // --- Friction mu: blend from static to dynamic based on saturation ---
    float saturation = std::min(combinedSlip, 1.f);
    float muLat = lateralMuStatic + saturation * (lateralMuDynamic - lateralMuStatic);
    float muLon = longMuStatic    + saturation * (longMuDynamic    - longMuStatic);

    float maxLat = input.normalLoad * muLat * grip;
    float maxLon = input.normalLoad * muLon * grip;

    // --- Lateral force ---
    float Fy = 0.f;
    if (speed > 0.5f) {
        // Pacejka-lite: sin curve with peak at ~8° then fading
        Fy = -maxLat * std::sin(1.4f * slipAngle);
    } else {
        // Low-speed stiction
        float blend = speed / 0.5f;
        float ramp  = std::max(-1.f, std::min(1.f, -vLat * 20.f));
        Fy = ramp * maxLat * (1.f - blend);
    }
    tireOutput_.lateralForceN = Fy;

    // --- Longitudinal force ---
    float driveForce = driveForceRaw;
    driveForce = std::clamp(driveForce, -maxLon, maxLon);

    float brakeForce = 0.f;
    if (brakeTorqueNm_ > 0.01f && std::abs(vFwd) > 0.01f) {
        float sign = (vFwd > 0.f) ? -1.f : 1.f;
        brakeForce = sign * std::min(brakeTorqueNm_ / std::max(radius, 0.01f), maxLon);
    }
    float Fx = driveForce + brakeForce;

    // Low-speed stiction
    if (speed < 0.5f && std::abs(driveTorqueNm_) < 0.01f) {
        float blend = speed / 0.5f;
        float ramp  = std::max(-1.f, std::min(1.f, -vFwd * 20.f));
        float holdForce = ramp * maxLon * (1.f - blend);
        float brakeNorm = std::min(brakeTorqueNm_ / std::max(radius, 0.01f) / maxLon, 1.f);
        float holdMul = 0.05f + 0.95f * brakeNorm;
        Fx += holdForce * holdMul;
    }
    tireOutput_.longForceN = Fx;

    // --- Grip circle clamp: combined force can't exceed friction circle ---
    float totalForce = std::sqrt(Fx * Fx + Fy * Fy);
    float maxTotal = input.normalLoad * muLat * grip; // use lateral (higher) as circle radius
    if (totalForce > maxTotal && totalForce > 0.f) {
        float scale = maxTotal / totalForce;
        Fx *= scale;
        Fy *= scale;
        tireOutput_.lateralForceN = Fy;
        tireOutput_.longForceN = Fx;
        tireOutput_.sliding = true;
    }

    // --- Rotate tire-local forces to world frame ---
    glm::vec3 tireForce = input.bodyRotation *
        glm::vec3(Fy * cs + Fx * sn, 0.f, -Fy * sn + Fx * cs);

    return { .force = tireForce };
}
