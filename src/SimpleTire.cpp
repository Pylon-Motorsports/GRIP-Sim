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

    // --- Static grip limits (used for individual force clamping) ---
    float maxLonStatic = input.normalLoad * longMuStatic * grip;
    float maxLatStatic = input.normalLoad * lateralMuStatic * grip;

    // --- Slip ratio (for reporting / trail rendering) ---
    float slipRatio = 0.f;
    if (speed > 0.5f && maxLonStatic > 1.f) {
        float netLon = driveTorqueNm_ / std::max(radius, 0.01f);
        if (brakeTorqueNm_ > 0.01f && std::abs(vFwd) > 0.01f) {
            float bSign = (vFwd > 0.f) ? -1.f : 1.f;
            netLon += bSign * brakeTorqueNm_ / std::max(radius, 0.01f);
        }
        slipRatio = std::clamp(netLon / maxLonStatic, -1.f, 1.f);
    }
    tireOutput_.slipRatio = slipRatio;

    // --- Lateral force (Pacejka-lite, clamped to static mu) ---
    float Fy = 0.f;
    if (speed > 0.5f) {
        // sin(C * atan(B * x)): B=15 steep initial slope, C=1.5 peak at ~8°
        Fy = -maxLatStatic * std::sin(1.5f * std::atan(15.f * slipAngle));
        // Low-speed lateral fade: contact patch needs velocity to deform.
        // Quadratic below 3 m/s prevents full-lock spin from standstill.
        float speedFade = std::min(speed * speed / 9.f, 1.f);
        Fy *= speedFade;
    } else {
        // Low-speed stiction
        float blend = speed / 0.5f;
        float ramp  = std::clamp(-vLat * 20.f, -1.f, 1.f);
        Fy = ramp * maxLatStatic * (1.f - blend);
    }
    tireOutput_.lateralForceN = Fy;

    // --- Longitudinal force (clamped to static mu) ---
    float driveForceRaw = driveTorqueNm_ / std::max(radius, 0.01f);
    float brakeForceRaw = brakeTorqueNm_ / std::max(radius, 0.01f);

    float driveForce = std::clamp(driveForceRaw, -maxLonStatic, maxLonStatic);

    float brakeForce = 0.f;
    if (brakeTorqueNm_ > 0.01f && std::abs(vFwd) > 0.01f) {
        float sign = (vFwd > 0.f) ? -1.f : 1.f;
        brakeForce = sign * std::min(brakeForceRaw, maxLonStatic);
    }
    float Fx = driveForce + brakeForce;

    // Low-speed stiction
    if (speed < 0.5f && std::abs(driveTorqueNm_) < 0.01f) {
        float blend = speed / 0.5f;
        float ramp  = std::clamp(-vFwd * 20.f, -1.f, 1.f);
        float holdForce = ramp * maxLonStatic * (1.f - blend);
        float brakeNorm = std::min(brakeForceRaw / std::max(maxLonStatic, 1.f), 1.f);
        float holdMul = 0.05f + 0.95f * brakeNorm;
        Fx += holdForce * holdMul;
    }
    tireOutput_.longForceN = Fx;

    // --- Friction ellipse: combined force can't exceed grip envelope ---
    // (Fx/maxLon)^2 + (Fy/maxLat)^2 <= 1
    // Pure lon or pure lat never exceeds 1.0 (already clamped above).
    // Only combined loading can exceed the ellipse.
    float lonNorm = std::abs(Fx) / std::max(maxLonStatic, 1.f);
    float latNorm = std::abs(Fy) / std::max(maxLatStatic, 1.f);
    float ellipse = lonNorm * lonNorm + latNorm * latNorm;

    bool sliding = false;
    if (ellipse > 1.f) {
        float scale = 1.f / std::sqrt(ellipse);
        Fx *= scale;
        Fy *= scale;
        tireOutput_.lateralForceN = Fy;
        tireOutput_.longForceN = Fx;
        sliding = true;
    }

    // Brake demand exceeding grip = wheel lockup = sliding
    if (brakeForceRaw > maxLonStatic * 0.95f && speed > 2.f)
        sliding = true;

    tireOutput_.sliding = sliding;

    // --- Rotate tire-local forces to world frame ---
    glm::vec3 tireForce = input.bodyRotation *
        glm::vec3(Fy * cs + Fx * sn, 0.f, -Fy * sn + Fx * cs);

    return { .force = tireForce };
}
