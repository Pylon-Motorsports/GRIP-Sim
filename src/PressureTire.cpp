#include "PressureTire.hpp"
#include <cmath>
#include <algorithm>

static constexpr float PI  = 3.14159265f;
static constexpr float TAU = 6.28318530f;

float PressureTire::brushForce(float slip, float Fn, float Cx) const
{
    if (Fn <= 0.f) return 0.f;
    float maxF = mu * Fn;
    if (maxF < 0.001f) return 0.f;
    float k = Cx * std::abs(slip) / (3.f * maxF);
    float mag;
    if (k < 1.f)
        mag = maxF * (3.f * k - 3.f * k * k + k * k * k);
    else
        mag = maxF;
    return (slip >= 0.f) ? mag : -mag;
}

Tire::VolumetricResult PressureTire::computeVolumetricForces(
    const VolumetricInput& in, float dt)
{
    VolumetricResult result;

    float effectivePressure = casingFlex * inflatePressurePa;
    float cellArc   = TAU * radius / CIRC_SAMPLES;
    float cellWidth = width / WIDTH_SAMPLES;
    float cellArea  = cellArc * cellWidth;

    // Build coordinate frame from input directions
    glm::vec3 fwd  = in.forwardDir;
    glm::vec3 axle = in.axleWorldDir;
    glm::vec3 up   = glm::cross(fwd, axle);

    // Normalize (should already be unit, but be safe)
    float upLen = glm::length(up);
    if (upLen > 0.01f) up /= upLen;
    else up = {0.f, 1.f, 0.f};

    float totalNormalLoad = 0.f;
    float totalWheelTorque = 0.f;
    int penetratingCount = 0;

    for (int i = 0; i < CIRC_SAMPLES; ++i) {
        float theta = (float)i * TAU / CIRC_SAMPLES;
        float sinT = std::sin(theta);
        float cosT = std::cos(theta);

        for (int j = 0; j < WIDTH_SAMPLES; ++j) {
            float wOffset = ((float)j - (WIDTH_SAMPLES - 1) * 0.5f) * cellWidth;

            // Sample position on cylinder surface (world frame)
            // theta=0 is straight down from hub
            glm::vec3 samplePos = in.hubWorldPos
                + fwd  * (radius * sinT)
                + axle * wOffset
                - up   * (radius * cosT);

            // Query terrain (3D contact: handles floors, walls, and ends)
            TerrainContact contact;
            if (bumps_ && !bumps_->empty()) {
                contact = sampleTerrainContact(*bumps_, samplePos);
            } else {
                contact.penetration = -samplePos.y;
                contact.normal = {0.f, 1.f, 0.f};
            }

            if (contact.penetration <= 0.f) continue;

            ++penetratingCount;

            // --- Normal force (pressure × area) ---
            float Fn_i = effectivePressure * cellArea;

            // --- Damping ---
            float vInto = -glm::dot(in.hubVelocity, contact.normal);
            float Fdamp = contactDampingPerSample * vInto;
            float Fn_total = std::max(Fn_i + Fdamp, 0.f);

            totalNormalLoad += Fn_total;
            result.worldForce += Fn_total * contact.normal;

            // --- Per-sample grip (brush cubic) ---
            // Surface velocity from wheel rotation at this angle
            float vSurfFwd = in.wheelAngularVel * radius * cosT;

            // Local slip ratio
            constexpr float slipEps = 0.5f;
            float denom = std::max({std::abs(vSurfFwd), std::abs(in.vLong), slipEps});
            float SR = (vSurfFwd - in.vLong) / denom;

            // Slip angle (same for all samples — depends on body velocity)
            float SA = std::atan2(in.vLat, std::max(std::abs(in.vLong), slipEps));

            // Brush forces per sample
            float CxLocal = slipStiffnessPerArea * cellArea;
            float CyLocal = latSlipStiffnessPerArea * cellArea;
            float Fx_i = brushForce(SR, Fn_total, CxLocal);
            float Fy_i = brushForce(SA, Fn_total, CyLocal);

            // Friction circle clamp per sample
            frictionCircleClamp(Fx_i, Fy_i, Fn_total);

            // Transform grip to world frame
            // Fx along forward, Fy opposes lateral (negative axle direction)
            result.worldForce += fwd * Fx_i - axle * Fy_i;

            // Wheel torque reaction (lever arm depends on angle)
            totalWheelTorque += -Fx_i * radius * cosT;
        }
    }

    result.normalLoad = totalNormalLoad;
    result.contactArea = penetratingCount * cellArea;
    result.wheelTorqueReaction = totalWheelTorque;

    // Rolling resistance
    if (totalNormalLoad > 0.f) {
        result.rollingResistForce = computeRollingResistance(
            totalNormalLoad, in.vLong);
    }

    // Update shared state for HUD/diagnostics
    normalLoad = totalNormalLoad;
    contactPatchArea = result.contactArea;
    deflection = (penetratingCount > 0) ? 0.005f : 0.f;  // approximate for HUD

    return result;
}
