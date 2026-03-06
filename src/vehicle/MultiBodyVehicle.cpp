#include "MultiBodyVehicle.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <algorithm>

static constexpr float G = 9.81f;
static constexpr float EPS = 0.01f;   // velocity epsilon for slip calculations

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Body-frame position of the suspension spring attachment (top of spring).
///
/// Layout in body frame (X=right, Y=up, Z=fwd):
///   FL: (+halfTrackFront,  cornerY,  +lf)
///   FR: (-halfTrackFront,  cornerY,  +lf)
///   RL: (+halfTrackRear,   cornerY,  -lr)
///   RR: (-halfTrackRear,   cornerY,  -lr)
///
/// cornerY is computed so that at static equilibrium the CG sits at params_.cgHeightM.
glm::vec3 MultiBodyVehicle::cornerBodyPos(int c) const
{
    float L_static = staticSpringLength(c);

    // body_corner_world_Y = wheel_radius + L_static
    // Body corner Y in body frame = body_corner_world_Y - cg_height
    float cornerY = params_.wheelRadiusM + L_static - params_.cgHeightM;

    bool front = (c == FL || c == FR);
    bool left  = (c == FL || c == RL);

    float hx = front ? params_.halfTrackFrontM : params_.halfTrackRearM;
    float hz  = front ? params_.lfDistM        : -params_.lrDistM;

    return { left ? +hx : -hx, cornerY, hz };
}

float MultiBodyVehicle::staticSpringLength(int c) const
{
    bool front = (c == FL || c == FR);
    float k = front ? params_.springRateFrontNm : params_.springRateRearNm;

    // Weight on this corner (static, no acceleration)
    float axleLoad = params_.massKg * G * (front ? params_.lrDistM : params_.lfDistM)
                   / params_.wheelbaseM;
    float cornerLoad = axleLoad / 2.f;

    float deflection = cornerLoad / k;
    return params_.springRestLengthM - deflection;
}

glm::mat3 MultiBodyVehicle::bodyToWorld() const
{
    // R = Ry(yaw) * Rz(roll) * Rx(pitch)
    using glm::mat4; using glm::mat3; using glm::vec3; using glm::rotate;
    mat3 Ry = mat3(rotate(mat4(1.f), yaw_,   vec3(0.f, 1.f, 0.f)));
    mat3 Rz = mat3(rotate(mat4(1.f), roll_,  vec3(0.f, 0.f, 1.f)));
    mat3 Rx = mat3(rotate(mat4(1.f), pitch_, vec3(1.f, 0.f, 0.f)));
    return Ry * Rz * Rx;
}

float MultiBodyVehicle::engineTorque(float throttle, float rpm) const
{
    float rpmNorm = rpm / params_.maxRpm;
    // Peak at 60% of max RPM, tapers at both ends
    float curve = 1.f - 0.5f * (rpmNorm - 0.6f) * (rpmNorm - 0.6f) / 0.36f;
    curve = std::clamp(curve, 0.2f, 1.f);

    // Engine braking when throttle is closed: negative torque proportional to RPM.
    // Models pumping losses and internal friction.
    float driveTorque = throttle * params_.maxTorqueNm * curve;
    float brakingTorque = (1.f - throttle) * params_.engineBrakeTorqueNm
                        * (rpm / params_.maxRpm);
    return driveTorque - brakingTorque;
}

void MultiBodyVehicle::autoShift()
{
    if (engineRpm_ > params_.upshiftRpm && currentGear_ < params_.numGears)
        ++currentGear_;
    else if (engineRpm_ < params_.downshiftRpm && currentGear_ > 1)
        --currentGear_;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

MultiBodyVehicle::MultiBodyVehicle(MultiBodyParams params)
    : params_(params)
{
    reset({}, 0.f);
}

void MultiBodyVehicle::reset(glm::vec3 position, float headingRad)
{
    pos_  = position;
    yaw_  = headingRad;

    float groundY = terrain_.empty() ? 0.f : terrain_.heightAt(position.x, position.z, position.y);
    if (groundY <= TerrainQuery::NO_GROUND + 1.f) groundY = 0.f;
    pos_.y = groundY + params_.cgHeightM;   // place CG at correct height above ground

    // Initialize pitch to match terrain slope so suspension starts in equilibrium.
    // Without this, spawning on a slope causes instant suspension bottoming.
    // Sign: pitch_ > 0 = nose-down. Uphill (front higher) → nose-up → negative pitch.
    float terrainPitch = 0.f;
    if (!terrain_.empty()) {
        float fwdX = std::sin(headingRad), fwdZ = std::cos(headingRad);
        float halfWB = params_.wheelbaseM * 0.5f;
        float hFront = terrain_.heightAt(position.x + fwdX * halfWB, position.z + fwdZ * halfWB, groundY);
        float hRear  = terrain_.heightAt(position.x - fwdX * halfWB, position.z - fwdZ * halfWB, groundY);
        if (hFront > TerrainQuery::NO_GROUND + 1.f && hRear > TerrainQuery::NO_GROUND + 1.f)
            terrainPitch = -std::atan2(hFront - hRear, params_.wheelbaseM);
    }

    vel_      = {};
    roll_     = 0.f;
    pitch_    = terrainPitch;
    yawRate_  = 0.f;
    rollRate_ = 0.f;
    pitchRate_= 0.f;

    engineRpm_   = params_.idleRpm;
    currentGear_ = 1;

    // Initialize suspension to static equilibrium
    for (int c = 0; c < 4; ++c) {
        float L_static = staticSpringLength(c);
        susp_[c].currentLength = L_static;
        susp_[c].prevLength    = L_static;
        susp_[c].normalForce   = 0.f;
        wheels_[c] = WheelState{};
        wheelGroundHeight_[c] = groundY;
    }

    state_ = VehicleState{};
    state_.position    = pos_;
    state_.headingRad  = yaw_;
    state_.currentGear = currentGear_;
    state_.engineRpm   = engineRpm_;
}

// ---------------------------------------------------------------------------
// integrate — called at fixed 120 Hz
// ---------------------------------------------------------------------------
void MultiBodyVehicle::integrate(const InputFrame& input, float dt)
{
    const float m  = params_.massKg;
    const float rw = params_.wheelRadiusM;
    const float Iyaw   = params_.Iyaw;
    const float Iroll  = params_.Iroll;
    const float Ipitch = params_.Ipitch;

    // -----------------------------------------------------------------------
    // 1. Body rotation matrix (body → world)
    // -----------------------------------------------------------------------
    glm::mat3 R = bodyToWorld();

    // Body-frame basis vectors in world space
    glm::vec3 fwd   = glm::vec3(R[2]);   // body +Z → world forward
    glm::vec3 right = glm::vec3(R[0]);   // body +X → world right

    // -----------------------------------------------------------------------
    // 2. Steering angle (speed-sensitive)
    //    At higher speeds, effective steering angle is reduced. This models
    //    both real steering geometry (Ackermann, compliance) and the fact that
    //    large steering angles at high speed instantly saturate the tyres.
    //    Factor = 1 / (1 + speed/refSpeed), so at refSpeed the angle halves.
    // -----------------------------------------------------------------------
    float speed = glm::length(vel_);
    float steerFactor = 1.f / (1.f + speed / params_.steerSpeedFactor);
    float steerAngle = input.steer * params_.maxSteerAngleRad * steerFactor;
    wheels_[FL].steerAngle = steerAngle;
    wheels_[FR].steerAngle = steerAngle;
    wheels_[RL].steerAngle = 0.f;
    wheels_[RR].steerAngle = 0.f;

    // -----------------------------------------------------------------------
    // 3. Suspension: compute normal forces from spring + damper
    //    [S2 fix] Forces act along the spring axis (hub→body), not purely vertical.
    //    [B2 fix] Damper uses unclamped desired length for velocity sensing.
    //    [H8 fix] Progressive bump stops (polynomial) replace linear 3x hack.
    // -----------------------------------------------------------------------
    float totalNz   = 0.f;
    float frontNz   = 0.f;

    for (int c = 0; c < 4; ++c) {
        glm::vec3 bodyPos = cornerBodyPos(c);
        glm::vec3 cornerW = pos_ + R * bodyPos;

        float groundY = terrain_.empty() ? 0.f
                      : terrain_.heightAt(cornerW.x, cornerW.z, wheelGroundHeight_[c]);

        // Desired hub position: wheel resting on ground
        float desiredHubY = groundY + rw;

        // Spring length if hub were at desired position
        float desiredSpringLen = cornerW.y - desiredHubY;

        // Travel limits
        float L_static = staticSpringLength(c);
        float minLen = L_static - params_.maxBumpM;
        float maxLen = L_static + params_.maxReboundM;

        float springLen = std::clamp(desiredSpringLen, minLen, maxLen);
        springLen = std::max(springLen, 0.01f);

        float actualHubY = cornerW.y - springLen;

        bool offMesh = (!terrain_.empty() && groundY <= TerrainQuery::NO_GROUND + 1.f);
        bool touchesGround = !offMesh && (desiredSpringLen <= maxLen);
        // Preserve NO_GROUND for truly off-mesh wheels so barrel-roll detection works.
        wheelGroundHeight_[c] = offMesh ? TerrainQuery::NO_GROUND
                              : (touchesGround ? groundY : (actualHubY - rw));

        float deflection = params_.springRestLengthM - springLen;

        // Damper velocity: use unclamped-to-unclamped comparison so the damper
        // correctly senses the direction of motion even at travel limits.
        // Previously mixed clamped current vs unclamped previous, which caused
        // the damper to fight rebound when bottomed out (corner gets stuck).
        float currentDesiredDefl = params_.springRestLengthM - desiredSpringLen;
        float prevDesiredDefl    = params_.springRestLengthM - susp_[c].prevLength;
        float deflRate = (currentDesiredDefl - prevDesiredDefl) / dt;

        bool front = (c == FL || c == FR);
        float k = front ? params_.springRateFrontNm : params_.springRateRearNm;
        float d = front ? params_.damperFrontNsm    : params_.damperRearNsm;

        float Nz = k * deflection + d * deflRate;

        // [H8 fix] Progressive bump stop (polynomial)
        if (desiredSpringLen < minLen) {
            float penetration = minLen - desiredSpringLen;
            float maxPen = params_.maxBumpM;
            float normPen = std::min(penetration / maxPen, 1.f);
            float bumpForce = params_.bumpStopRateNm * maxPen
                            * std::pow(normPen, params_.bumpStopExponent);
            Nz += bumpForce;
        }

        // No force when fully extended (wheel airborne)
        if (!touchesGround)
            Nz = 0.f;

        Nz = std::max(0.f, Nz);   // can't pull; wheel lifts off

        // [B2 fix] Store unclamped desired length for next frame's damper velocity.
        // Clamp prevLength to a sane range to prevent NaN from extreme off-mesh heights.
        susp_[c].currentLength = springLen;
        susp_[c].prevLength    = std::clamp(desiredSpringLen, -1.f, 2.f);
        susp_[c].normalForce   = Nz;
        wheels_[c].normalForce = Nz;
        wheels_[c].inContact   = (Nz > 0.f);

        totalNz += Nz;
        if (c == FL || c == FR) frontNz += Nz;
    }

    // -----------------------------------------------------------------------
    // 3b. Sway bars — transfer load between left/right on each axle
    // -----------------------------------------------------------------------
    {
        // Front axle: FL vs FR
        float frontDiffDefl = (params_.springRestLengthM - susp_[FL].currentLength)
                            - (params_.springRestLengthM - susp_[FR].currentLength);
        float frontRollAngle = frontDiffDefl / (2.f * params_.halfTrackFrontM);
        float frontSwayForce = params_.swayBarFrontNmRad * frontRollAngle
                             / params_.halfTrackFrontM;
        susp_[FL].normalForce += frontSwayForce;
        susp_[FR].normalForce -= frontSwayForce;
        wheels_[FL].normalForce = std::max(0.f, susp_[FL].normalForce);
        wheels_[FR].normalForce = std::max(0.f, susp_[FR].normalForce);

        // Rear axle: RL vs RR
        float rearDiffDefl = (params_.springRestLengthM - susp_[RL].currentLength)
                           - (params_.springRestLengthM - susp_[RR].currentLength);
        float rearRollAngle = rearDiffDefl / (2.f * params_.halfTrackRearM);
        float rearSwayForce = params_.swayBarRearNmRad * rearRollAngle
                            / params_.halfTrackRearM;
        susp_[RL].normalForce += rearSwayForce;
        susp_[RR].normalForce -= rearSwayForce;
        wheels_[RL].normalForce = std::max(0.f, susp_[RL].normalForce);
        wheels_[RR].normalForce = std::max(0.f, susp_[RR].normalForce);

        // Recalculate totals
        totalNz = 0.f; frontNz = 0.f;
        for (int c = 0; c < 4; ++c) {
            wheels_[c].inContact = (wheels_[c].normalForce > 0.f);
            totalNz += wheels_[c].normalForce;
            if (c == FL || c == FR) frontNz += wheels_[c].normalForce;
        }
    }

    // -----------------------------------------------------------------------
    // 4. Tyre forces
    //    [S5 fix] Contact velocity includes yaw component of angular velocity.
    //    [M1 fix] Load-sensitive friction: μ decreases with load.
    //    [H3 fix] Relaxation-length filtered slip angle replaces latRamp hack.
    // -----------------------------------------------------------------------
    glm::vec3 totalTyreForce  { 0.f };
    glm::vec3 totalTyreTorque { 0.f };

    // Drivetrain
    float gearRatio  = params_.gearRatios[currentGear_ - 1];
    float tEngine    = engineTorque(input.throttle, engineRpm_);
    float driveAtWhl = tEngine * gearRatio * params_.finalDriveRatio * params_.drivetrainEff;

    // Drive split (RWD = 0% front, 100% rear)
    float driveFrac[4] = {
        params_.frontTorqueFrac / 2.f,
        params_.frontTorqueFrac / 2.f,
        (1.f - params_.frontTorqueFrac) / 2.f,
        (1.f - params_.frontTorqueFrac) / 2.f
    };

    // Brake force per wheel
    float brakeTotal = input.brake * params_.maxBrakeN;
    float brakeFwdFrac[4] = {
        params_.brakeBiasFront / 2.f,
        params_.brakeBiasFront / 2.f,
        (1.f - params_.brakeBiasFront) / 2.f,
        (1.f - params_.brakeBiasFront) / 2.f
    };

    // Reference load for load sensitivity (static corner load, average of front/rear)
    float NzRef = m * G / 4.f;

    // [S5] Yaw angular velocity vector (only yaw, no pitch/roll — those would
    // cause phantom contact velocity at standstill due to body oscillation).
    glm::vec3 yawAngVel { 0.f, yawRate_, 0.f };

    for (int c = 0; c < 4; ++c) {
        if (!wheels_[c].inContact) continue;

        float Nz = wheels_[c].normalForce;

        // [M1 fix] Load-sensitive friction: μ drops with load above reference.
        // mu(Nz) = mu0 - loadSens * (Nz - NzRef)
        // Clamped to [0.4, mu0] to prevent negative or unreasonable values.
        float mu = params_.tyreMu - params_.tyreLoadSens * (Nz - NzRef);
        mu = std::clamp(mu, 0.4f, params_.tyreMu + 0.1f);
        float Fmax = mu * Nz;

        // Wheel forward direction in world space
        float totalYaw = yaw_ + wheels_[c].steerAngle;
        glm::vec3 wFwd { std::sin(totalYaw), 0.f, std::cos(totalYaw) };
        glm::vec3 wLat { std::cos(totalYaw), 0.f, -std::sin(totalYaw) };

        // Contact point and velocity
        glm::vec3 bodyPos    = cornerBodyPos(c);
        glm::vec3 cornerW    = pos_ + R * bodyPos;
        glm::vec3 contactPt  = cornerW; contactPt.y = wheelGroundHeight_[c];
        glm::vec3 rContact   = contactPt - pos_;

        // [S5 fix] Include yaw angular velocity in contact velocity.
        // v_contact = v_cg + omega_yaw × r_contact
        glm::vec3 vContact = vel_ + glm::cross(yawAngVel, rContact);

        float vLong = glm::dot(vContact, wFwd);
        float vLat  = glm::dot(vContact, wLat);
        float vLongAbs = std::abs(vLong);

        // --- Longitudinal slip ratio ---
        float vWheel = wheels_[c].spinRate * rw;
        float kappa  = (vWheel - vLong)
                     / (std::max(std::abs(vWheel), vLongAbs) + EPS);

        // Longitudinal force (tanh traction curve, smooth through zero)
        float Fx_raw = Fmax * std::tanh(kappa / params_.tyreLongPeakSlip);

        // --- Lateral slip angle ---
        // [H3 fix] Use relaxation-length first-order filter instead of latRamp hack.
        // Raw slip angle (well-conditioned at zero speed via relaxation length):
        //   alpha_raw = -atan2(vLat, |vLong| + relax * spinRate)
        // The relaxation length term keeps the denominator nonzero at standstill.
        float relaxDenom = vLongAbs + params_.tyreRelaxLenM * std::abs(wheels_[c].spinRate);
        float alpha_raw = (relaxDenom > EPS)
                        ? -std::atan2(vLat, relaxDenom)
                        : 0.f;

        // First-order lag: tau = relaxLen / max(|vLong|, minSpeed)
        float tau = params_.tyreRelaxLenM / std::max(vLongAbs, 0.5f);
        float blend = std::min(1.f, dt / tau);
        wheels_[c].alphaFiltered += blend * (alpha_raw - wheels_[c].alphaFiltered);
        float alpha = wheels_[c].alphaFiltered;

        // Lateral force (tanh curve for smooth saturation, matching longitudinal)
        float alphaMax = Fmax / params_.tyreLatStiff;  // angle at which linear = Fmax
        float Fy_raw = Fmax * std::tanh(alpha / alphaMax);

        // Low-speed lateral force fade: tyre lateral force requires rolling to
        // deform the contact patch. At standstill there's no contact patch
        // deformation, so Fy should be zero. Without this, body oscillation at
        // rest creates phantom lateral velocity → large slip angle → large Fy
        // → more body oscillation (positive feedback loop).
        // Quadratic fade: force ramps slowly near zero, full above fadeSpeed.
        float contactSpeed = std::max(vLongAbs, std::abs(vWheel));
        constexpr float latFadeSpeed = 3.0f;
        float latFadeLinear = std::min(1.f, contactSpeed / latFadeSpeed);
        float latFade = latFadeLinear * latFadeLinear;  // quadratic — steep near zero
        Fy_raw *= latFade;

        // Friction circle: combined limit
        float Fcombined = std::sqrt(Fx_raw * Fx_raw + Fy_raw * Fy_raw);
        if (Fcombined > Fmax) {
            float scale = Fmax / Fcombined;
            Fx_raw *= scale;
            Fy_raw *= scale;
        }

        // Forces in world frame
        glm::vec3 Fworld = wFwd * Fx_raw + wLat * Fy_raw;
        totalTyreForce  += Fworld;
        totalTyreTorque += glm::cross(rContact, Fworld);

        // Self-aligning torque (SAT): pneumatic trail creates a yaw moment
        // that opposes the slip angle, stabilizing the car at speed.
        // Trail decreases as tyre saturates (contact patch shifts forward).
        float saturation = (Fmax > 0.f) ? std::abs(Fy_raw) / Fmax : 0.f;
        float effectiveTrail = params_.tyrePneumaticTrailM * (1.f - saturation);
        float Mz_sat = -effectiveTrail * Fy_raw;  // yaw moment about contact point
        totalTyreTorque.y += Mz_sat;

        wheels_[c].Fx = Fx_raw;
        wheels_[c].Fy = Fy_raw;

        // --- Wheel spin integration ---
        float T_drive = driveAtWhl * driveFrac[c];
        float T_ground = -Fx_raw * rw;

        // [B1 fix] Brake torque opposes current spin direction.
        float T_brake = 0.f;
        if (std::abs(wheels_[c].spinRate) > 0.1f)
            T_brake = -std::copysign(1.f, wheels_[c].spinRate)
                    * brakeTotal * brakeFwdFrac[c];
        else if (brakeTotal > 0.f) {
            // Wheel nearly stopped: cancel net torque to hold wheel still.
            float T_net = T_drive + T_ground;
            T_brake = -T_net;
            // At low speed, hold at ground speed (not zero) to prevent slip-ratio
            // discontinuity on brake release. At high speed, hold at zero (locked
            // wheel) — normal braking behavior.
            // Also clear the tyre relaxation filter so stored cornering slip from
            // before braking doesn't create phantom lateral force after release.
            float groundSpin = std::max(0.f, vLong / rw);
            if (groundSpin < 1.0f) {
                wheels_[c].spinRate = groundSpin;
                wheels_[c].alphaFiltered = 0.f;
            }
        }

        float alpha_wheel = (T_drive + T_brake + T_ground) / params_.wheelInertiaKgM2;
        wheels_[c].spinRate += alpha_wheel * dt;
        wheels_[c].spinRate = std::max(0.f, wheels_[c].spinRate);  // no reverse (POC)
    }

    // -----------------------------------------------------------------------
    // 5. Suspension forces on body
    //    [S2 fix] Force acts along spring axis (hub → body attachment), not
    //    purely vertical. This naturally provides roll/pitch damping through
    //    the suspension geometry.
    // -----------------------------------------------------------------------
    glm::vec3 totalSuspForce  { 0.f };
    glm::vec3 totalSuspTorque { 0.f };

    for (int c = 0; c < 4; ++c) {
        float Nz = susp_[c].normalForce;
        if (Nz < 0.01f) continue;

        glm::vec3 bodyPos  = cornerBodyPos(c);
        glm::vec3 cornerW  = pos_ + R * bodyPos;  // body-side attachment
        glm::vec3 hubW     = cornerW;
        hubW.y = cornerW.y - susp_[c].currentLength;  // wheel hub position

        // Spring axis direction: hub → body attachment (normalized)
        glm::vec3 springAxis = cornerW - hubW;
        float springAxisLen = glm::length(springAxis);
        if (springAxisLen > 0.001f)
            springAxis /= springAxisLen;
        else
            springAxis = { 0.f, 1.f, 0.f };

        // Force acts along spring axis, magnitude = Nz
        glm::vec3 F_susp = springAxis * Nz;

        glm::vec3 rCorner  = cornerW - pos_;
        totalSuspForce  += F_susp;
        totalSuspTorque += glm::cross(rCorner, F_susp);
    }

    // -----------------------------------------------------------------------
    // 5b. Rolling resistance (per-wheel, proportional to normal load)
    // -----------------------------------------------------------------------
    {
        float spd = glm::length(vel_);
        if (spd > EPS) {
            float totalRrForce = 0.f;
            for (int c = 0; c < 4; ++c)
                totalRrForce += params_.rollingResistCrr * wheels_[c].normalForce;
            glm::vec3 rrForce = -(vel_ / spd) * totalRrForce;
            totalTyreForce += rrForce;
        }
    }

    // -----------------------------------------------------------------------
    // 6. Aerodynamic drag
    // -----------------------------------------------------------------------
    speed = glm::length(vel_);  // reuse variable from section 2
    glm::vec3 dragForce { 0.f };
    if (speed > EPS) {
        float dragMag = 0.5f * params_.airDensityKgM3 * params_.Cd
                      * params_.frontalAreaM2 * speed * speed;
        dragForce = -(vel_ / speed) * dragMag;
    }

    // -----------------------------------------------------------------------
    // 6b. Slope gravity
    // -----------------------------------------------------------------------
    glm::vec3 slopeForce { 0.f };
    if (!terrain_.empty()) {
        // Only compute slope if all four wheels are on the mesh
        bool allOnMesh = true;
        for (int c = 0; c < 4; ++c)
            if (wheelGroundHeight_[c] <= TerrainQuery::NO_GROUND + 1.f) allOnMesh = false;
        if (allOnMesh) {
            float frontGround = (wheelGroundHeight_[FL] + wheelGroundHeight_[FR]) * 0.5f;
            float rearGround  = (wheelGroundHeight_[RL] + wheelGroundHeight_[RR]) * 0.5f;
            float dY = frontGround - rearGround;
            float dZ = params_.wheelbaseM;
            float sinSlope = dY / std::sqrt(dY * dY + dZ * dZ);
            slopeForce = -fwd * (m * G * sinSlope);
        }
    }

    // -----------------------------------------------------------------------
    // 7. Equations of motion
    //    Supplementary rotational damping models bushing compliance, tyre
    //    sidewall damping, and chassis structural damping. These are real
    //    physical effects not captured by the spring/damper torque pathway.
    //    Values: ~15-25% of critical damping for each axis.
    // -----------------------------------------------------------------------
    glm::vec3 gravity { 0.f, -m * G, 0.f };
    glm::vec3 totalForce  = gravity + totalSuspForce + totalTyreForce + dragForce + slopeForce;

    // -----------------------------------------------------------------------
    // 7a. Terrain slope probes (yaw-only directions to prevent feedback)
    //     Must come before bushing damping and centering which use these.
    // -----------------------------------------------------------------------
    float terrainPitch = 0.f;
    float terrainRoll  = 0.f;
    if (!terrain_.empty()) {
        float sinY = std::sin(yaw_), cosY = std::cos(yaw_);
        glm::vec3 yawFwd   { sinY, 0.f, cosY };
        glm::vec3 yawRight { cosY, 0.f, -sinY };
        float hint = pos_.y;

        glm::vec3 probeFwd  = pos_ + yawFwd * (params_.wheelbaseM * 0.5f);
        glm::vec3 probeRear = pos_ - yawFwd * (params_.wheelbaseM * 0.5f);
        float hFront = terrain_.heightAt(probeFwd.x, probeFwd.z, hint);
        float hRear  = terrain_.heightAt(probeRear.x, probeRear.z, hint);
        if (hFront > TerrainQuery::NO_GROUND + 1.f && hRear > TerrainQuery::NO_GROUND + 1.f)
            terrainPitch = -std::atan2(hFront - hRear, params_.wheelbaseM);

        float avgTrack = params_.halfTrackFrontM + params_.halfTrackRearM;
        glm::vec3 probeLeft  = pos_ - yawRight * (avgTrack * 0.5f);
        glm::vec3 probeRight = pos_ + yawRight * (avgTrack * 0.5f);
        float hLeft  = terrain_.heightAt(probeLeft.x, probeLeft.z, hint);
        float hRight = terrain_.heightAt(probeRight.x, probeRight.z, hint);
        if (hLeft > TerrainQuery::NO_GROUND + 1.f && hRight > TerrainQuery::NO_GROUND + 1.f)
            terrainRoll = std::atan2(hRight - hLeft, avgTrack);
    }

    // Count how many wheels have terrain beneath them (on-mesh).
    // This is different from suspension contact: a wheel can be unloaded (no Nz)
    // during cornering but still have ground below it. We only reduce stabilization
    // when wheels are truly off the mesh edge — that means the car should fall.
    int onMeshWheels = 0;
    for (int c = 0; c < 4; ++c)
        if (wheelGroundHeight_[c] > TerrainQuery::NO_GROUND + 1.f) ++onMeshWheels;
    // Sharp scaling: full at 4, half at 3, zero at 0-2.
    float contactFrac = (onMeshWheels >= 4) ? 1.f
                      : (onMeshWheels == 3) ? 0.5f
                      : 0.f;

    // Bushing damping: reduced when rate is restoring (toward terrain slope).
    // Without this, damping fights the suspension restoring torque, trapping the body at limits.
    float rollDampCoeff  = 6000.f * contactFrac;
    float pitchDampCoeff = 4000.f * contactFrac;
    {
        float rollDev  = roll_  - terrainRoll;
        float pitchDev = pitch_ - terrainPitch;
        bool rollRestoring  = (rollDev  > 0.f && rollRate_  < 0.f) ||
                              (rollDev  < 0.f && rollRate_  > 0.f);
        bool pitchRestoring = (pitchDev > 0.f && pitchRate_ < 0.f) ||
                              (pitchDev < 0.f && pitchRate_ > 0.f);
        if (rollRestoring  && std::abs(rollDev)  > 0.08f) rollDampCoeff  *= 0.35f;
        if (pitchRestoring && std::abs(pitchDev) > 0.06f) pitchDampCoeff *= 0.35f;
    }

    glm::vec3 bushingDamping {
        -pitchRate_ * pitchDampCoeff,
        0.f,
        -rollRate_  * rollDampCoeff
    };

    // Roll center constraint: cubic centering spring toward terrain slope.
    // Scaled by contact fraction: with 2 wheels off-mesh, the centering halves,
    // allowing the car to barrel-roll off edges naturally.
    constexpr float kCenter = 6000.f;
    constexpr float thr = 0.02f;
    auto cubicCenter = [kCenter, thr, contactFrac](float deviation) -> float {
        float ratio = deviation / thr;
        return -kCenter * contactFrac * deviation * (1.f + ratio * ratio);
    };
    glm::vec3 centeringTorque {
        cubicCenter(pitch_ - terrainPitch),
        0.f,
        cubicCenter(roll_  - terrainRoll)
    };

    // When wheels are off-mesh on one side, gravity tips the car off the edge.
    // The suspension spring-axis force has a tipping instability that opposes
    // the correct direction, so we add an explicit gravitational torque.
    glm::vec3 edgeTipTorque { 0.f };
    if (onMeshWheels <= 2 && onMeshWheels > 0) {
        bool flOn = wheelGroundHeight_[FL] > TerrainQuery::NO_GROUND + 1.f;
        bool frOn = wheelGroundHeight_[FR] > TerrainQuery::NO_GROUND + 1.f;
        bool rlOn = wheelGroundHeight_[RL] > TerrainQuery::NO_GROUND + 1.f;
        bool rrOn = wheelGroundHeight_[RR] > TerrainQuery::NO_GROUND + 1.f;
        bool leftSideOn  = flOn || rlOn;
        bool rightSideOn = frOn || rrOn;
        if (leftSideOn != rightSideOn) {
            // Pivot on the supported side. Gravity torque = m*g*arm*cos(roll).
            float arm = params_.halfTrackFrontM;
            float sign = leftSideOn ? 1.f : -1.f; // +1 = tip right, -1 = tip left
            edgeTipTorque.z = sign * m * G * arm * std::cos(roll_);
        }
        // Fore/aft: if front or rear pair is off-mesh
        bool frontOn = flOn || frOn;
        bool rearOn  = rlOn || rrOn;
        if (frontOn != rearOn) {
            float arm = params_.wheelbaseM * 0.3f;
            float sign = rearOn ? 1.f : -1.f; // rear supported → nose dips
            edgeTipTorque.x = sign * m * G * arm * std::cos(pitch_);
        }
    }

    glm::vec3 totalTorque = totalSuspTorque + totalTyreTorque + bushingDamping + centeringTorque + edgeTipTorque;

    // -----------------------------------------------------------------------
    // 7b. Low-speed friction (static friction / stiction)
    //     At very low speed, tyres transition from rolling to static contact.
    //     This provides a smooth deceleration that brings the car to a full stop
    //     rather than creeping indefinitely on kinetic friction alone.
    // -----------------------------------------------------------------------
    {
        float spd = glm::length(vel_);
        // Static friction at very low speed: tyre contact patches resist motion
        // in all directions (not just rolling direction). Models the transition
        // from kinetic to static friction that brings the car to a full stop.
        constexpr float stictionSpeed = 1.5f;  // transition region (m/s)
        if (spd > EPS && spd < stictionSpeed) {
            float blend = 1.f - (spd / stictionSpeed);  // 1 at standstill, 0 at threshold
            float frictionForce = blend * m * G * 0.05f; // ~5% of weight as stiction
            totalForce -= (vel_ / spd) * frictionForce;
        }

        // Low-speed yaw damping: at low speed, tyre self-aligning torque is weak.
        // Model kingpin trail / caster geometry restoring yaw.
        if (spd < 3.0f) {
            float yawDampCoeff = 1500.f * (1.f - spd / 3.0f);  // fades out above 3 m/s
            totalTorque.y -= yawRate_ * yawDampCoeff;
        }

        // High-speed yaw stability: rear suspension compliance steer effect.
        // Real multi-link rear suspensions develop small toe changes under
        // lateral load that create a natural understeer tendency at speed.
        // Modeled as a gentle speed-proportional yaw damping.
        if (spd > 20.f) {
            float yawStabCoeff = (spd - 20.f) * 8.f;
            totalTorque.y -= yawRate_ * yawStabCoeff;
        }

        // Low-speed roll/pitch damping: at low speed, tyre lateral forces that
        // normally damp body motion through the contact patch are faded out.
        // Compensate with direct damping that models the static tyre contact
        // patch resisting body rotation. Fades out as speed increases and
        // normal tyre forces take over.
        // Reduced for restoring motion (toward terrain slope) to prevent trapping body at limits.
        if (spd < 3.0f) {
            float dampBlend = 1.f - spd / 3.0f;
            float rollDev2  = roll_  - terrainRoll;
            float pitchDev2 = pitch_ - terrainPitch;
            bool rollRestoring  = (rollDev2  > 0.f && rollRate_  < 0.f) ||
                                  (rollDev2  < 0.f && rollRate_  > 0.f);
            bool pitchRestoring = (pitchDev2 > 0.f && pitchRate_ < 0.f) ||
                                  (pitchDev2 < 0.f && pitchRate_ > 0.f);
            float rollMult  = rollRestoring  ? 0.3f : 1.f;
            float pitchMult = pitchRestoring ? 0.3f : 1.f;
            totalTorque.x -= pitchRate_ * 3000.f * dampBlend * pitchMult * contactFrac;
            totalTorque.z -= rollRate_  * 5000.f * dampBlend * rollMult * contactFrac;
        }
    }

    glm::vec3 linAccel = totalForce / m;

    // Angular acceleration (world frame, diagonal inertia approximation)
    float alphaPitch = totalTorque.x / Ipitch;
    float alphaYaw   = totalTorque.y / Iyaw;
    float alphaRoll  = totalTorque.z / Iroll;

// -----------------------------------------------------------------------
    // 8. Integration
    // -----------------------------------------------------------------------
    vel_  += linAccel * dt;
    pos_  += vel_     * dt;

    // Safety net: prevent CG from sinking below ground (only on-mesh wheels).
    // Uses a spring-damper instead of hard clamp to allow natural settling.
    {
        float sumGroundY = 0.f;
        int   onMeshCount = 0;
        for (int c = 0; c < 4; ++c) {
            if (wheelGroundHeight_[c] > TerrainQuery::NO_GROUND + 1.f) {
                sumGroundY += wheelGroundHeight_[c];
                ++onMeshCount;
            }
        }
        if (onMeshCount > 0) {
            float avgGroundY = sumGroundY / static_cast<float>(onMeshCount);
            float floorY = avgGroundY + params_.cgHeightM * 0.5f;
            if (pos_.y < floorY) {
                float penetration = floorY - pos_.y;
                // Strong spring pushes body back up
                float pushForce = penetration * 80000.f;  // N/m — very stiff
                float pushDamp  = -std::min(vel_.y, 0.f) * 10000.f; // damping
                vel_.y += (pushForce + pushDamp) / m * dt;
                // Hard backstop: never sink more than 0.1m below floor
                if (penetration > 0.1f) {
                    pos_.y = floorY - 0.1f;
                    if (vel_.y < 0.f) vel_.y = 0.f;
                }
            }
        }
    }

    pitchRate_ += alphaPitch * dt;
    yawRate_   += alphaYaw   * dt;
    rollRate_  += alphaRoll  * dt;

    pitch_ += pitchRate_ * dt;
    yaw_   += yawRate_   * dt;
    roll_  += rollRate_  * dt;

    // Hard limits: terrain-relative (deviation from terrain slope).
    // Tight when all 4 wheels on mesh; wide (90°) when off-mesh to allow barrel-roll.
    {
        float rollLim  = (onMeshWheels >= 4) ? 0.14f : 1.57f;
        float pitchLim = (onMeshWheels >= 4) ? 0.14f : 1.57f;
        constexpr float recoveryRate = 0.3f;

        float rollMax  = terrainRoll  + rollLim;
        float rollMin  = terrainRoll  - rollLim;
        float pitchMax = terrainPitch + pitchLim;
        float pitchMin = terrainPitch - pitchLim;

        if (roll_ >  rollMax)  { roll_ =  rollMax;  if (onMeshWheels >= 4) rollRate_  = -recoveryRate; }
        if (roll_ <  rollMin)  { roll_ =  rollMin;  if (onMeshWheels >= 4) rollRate_  =  recoveryRate; }
        if (pitch_ > pitchMax) { pitch_ = pitchMax; if (onMeshWheels >= 4) pitchRate_ = -recoveryRate; }
        if (pitch_ < pitchMin) { pitch_ = pitchMin; if (onMeshWheels >= 4) pitchRate_ =  recoveryRate; }
    }

    // NaN guard: if position goes NaN (e.g. from extreme free-fall), zero velocities.
    if (std::isnan(pos_.x) || std::isnan(pos_.y) || std::isnan(pos_.z)) {
        vel_ = {};
        yawRate_ = rollRate_ = pitchRate_ = 0.f;
        for (int c = 0; c < 4; ++c) wheels_[c].spinRate = 0.f;
    }

    // -----------------------------------------------------------------------
    // 8b. Tree collision — cylinder-circle overlap in XZ plane
    // -----------------------------------------------------------------------
    if (!trees_.empty()) {
        // Car collision radius: half-diagonal of bounding rectangle
        float halfDiag = std::sqrt(params_.lfDistM * params_.lfDistM
                                 + params_.halfTrackFrontM * params_.halfTrackFrontM);

        for (const auto& tree : trees_) {
            float dx = pos_.x - tree.position.x;
            float dz = pos_.z - tree.position.z;
            float dist2 = dx * dx + dz * dz;
            float minDist = halfDiag + tree.trunkRadius;
            if (dist2 < minDist * minDist && dist2 > 0.0001f) {
                float dist = std::sqrt(dist2);
                float penetration = minDist - dist;
                float nx = dx / dist;
                float nz = dz / dist;

                // Push car out of tree
                pos_.x += nx * penetration;
                pos_.z += nz * penetration;

                // Remove velocity into the tree (mostly inelastic)
                float vInto = vel_.x * nx + vel_.z * nz;
                if (vInto < 0.f) {
                    // Cancel velocity into tree + small bounce
                    vel_.x -= nx * vInto * 1.1f;
                    vel_.z -= nz * vInto * 1.1f;

                    // Energy absorption: hitting a tree is destructive
                    float impactSpeed = -vInto;
                    if (impactSpeed > 1.f) {
                        float energyLoss = std::min(0.5f, impactSpeed * 0.05f);
                        vel_ *= (1.f - energyLoss);
                        // Spin from off-center hit
                        float crossProduct = nx * std::sin(yaw_) + nz * std::cos(yaw_);
                        yawRate_ += crossProduct * impactSpeed * 0.3f;
                    }
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // 9. RPM + gear shift
    // -----------------------------------------------------------------------
    float avgRearSpin = (wheels_[RL].spinRate + wheels_[RR].spinRate) * 0.5f;
    float wheelRpm    = avgRearSpin * (60.f / (2.f * glm::pi<float>()));
    engineRpm_        = std::max(params_.idleRpm,
                                 wheelRpm * gearRatio * params_.finalDriveRatio);
    engineRpm_        = std::min(engineRpm_, params_.maxRpm);
    autoShift();

    // -----------------------------------------------------------------------
    // 10. Update VehicleState
    // -----------------------------------------------------------------------
    float vFwd = glm::dot(vel_, fwd);

    state_.position      = pos_;
    state_.velocity      = vel_;
    state_.headingRad    = yaw_;
    state_.speedMs       = vFwd;
    // Body slip angle: angle between velocity vector and heading direction.
    // Uses atan2(vLat, |vFwd|), faded by total speed. The fade ensures the
    // value goes to zero at standstill where the concept is meaningless.
    {
        float absVFwd = std::abs(vFwd);
        float totalSpd = glm::length(vel_);
        float rawSlip = (absVFwd > EPS)
                      ? std::atan2(glm::dot(vel_, right), absVFwd)
                      : 0.f;
        // Quadratic fade below 4 m/s — aggressive near zero to prevent phantom readings
        float t = std::clamp(totalSpd / 4.0f, 0.f, 1.f);
        state_.slipAngleRad = rawSlip * t * t;
    }
    state_.throttle      = input.throttle;
    state_.brake         = input.brake;
    state_.steer         = input.steer;
    state_.engineRpm     = engineRpm_;
    state_.currentGear   = currentGear_;
    state_.rollRad       = roll_;
    state_.pitchRad      = pitch_;
    state_.terrainRollRad  = terrainRoll;
    state_.terrainPitchRad = terrainPitch;
    state_.weightFront   = (totalNz > 0.f) ? (frontNz / totalNz) : 0.5f;
    state_.odoMeters    += std::max(0.f, vFwd) * dt;

    // Wheel hub world positions and suspension geometry (for renderer)
    for (int c = 0; c < 4; ++c) {
        glm::vec3 bodyPos = cornerBodyPos(c);
        glm::vec3 cornerW = pos_ + R * bodyPos;
        state_.suspTopPos[c] = cornerW;
        state_.suspLength[c] = susp_[c].currentLength;
        cornerW.y = wheelGroundHeight_[c] + rw;
        state_.wheelPos[c] = cornerW;
        state_.wheelGroundHeight[c] = wheelGroundHeight_[c];
    }

    updateSegmentTracking();
}

// ---------------------------------------------------------------------------
// Segment tracking
// ---------------------------------------------------------------------------
void MultiBodyVehicle::setCenterlinePoints(
    const std::vector<glm::vec3>& points,
    const std::vector<uint32_t>&  segmentStartVertex,
    int vertsPerRow)
{
    centerline_         = points;
    segmentStartVertex_ = segmentStartVertex;
    vertsPerRow_        = vertsPerRow;
}

void MultiBodyVehicle::setTerrainQuery(const TerrainQuery& terrain)
{
    terrain_ = terrain;
}

void MultiBodyVehicle::setTrees(const std::vector<TreeInstance>& trees)
{
    trees_ = trees;
}

void MultiBodyVehicle::updateSegmentTracking()
{
    if (centerline_.empty()) return;

    float minDist = 1e9f;
    int   bestIdx = 0;
    for (int i = 0; i < (int)centerline_.size(); ++i) {
        glm::vec3 d = state_.position - centerline_[i];
        d.y = 0.f;
        float dist = glm::dot(d, d);
        if (dist < minDist) { minDist = dist; bestIdx = i; }
    }

    int seg = 0;
    for (int s = (int)segmentStartVertex_.size() - 1; s >= 0; --s) {
        int firstPoint = segmentStartVertex_[s] / vertsPerRow_;
        if (bestIdx >= firstPoint) { seg = s; break; }
    }

    state_.segmentIndex    = seg;
    state_.segmentProgress = 0.f;
}
