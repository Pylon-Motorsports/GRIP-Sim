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
    // Static spring length at each corner (see staticSpringLength())
    float L_static = staticSpringLength(c);

    // World Y of body corner at static equilibrium:
    //   wheel hub is at world Y = wheel_radius
    //   spring goes from wheel hub UP to body corner
    //   => body_corner_world_Y = wheel_radius + L_static
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
    // Applied in reverse order of rotation: first pitch about X (body), then roll about Z, then yaw about Y.
    // This approximation is accurate for small roll/pitch, arbitrary yaw.
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
    return throttle * params_.maxTorqueNm * curve;
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
    pos_.y = params_.cgHeightM;   // place CG at correct height

    vel_      = {};
    yaw_      = headingRad;
    roll_     = 0.f;
    pitch_    = 0.f;
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

    // Convenience: body-frame basis vectors in world space
    glm::vec3 fwd   = glm::vec3(R[2]);   // body +Z → world forward
    glm::vec3 right = glm::vec3(R[0]);   // body +X → world right
    // glm::vec3 up   = glm::vec3(R[1]);  // body +Y → world up (unused directly)

    // -----------------------------------------------------------------------
    // 2. Steering angle (smooth input, but we apply directly for now)
    // -----------------------------------------------------------------------
    float steerAngle = input.steer * params_.maxSteerAngleRad;
    // Steer front wheels; rear wheels have 0 steer
    wheels_[FL].steerAngle = steerAngle;
    wheels_[FR].steerAngle = steerAngle;
    wheels_[RL].steerAngle = 0.f;
    wheels_[RR].steerAngle = 0.f;

    // -----------------------------------------------------------------------
    // 3. Suspension: compute normal forces from spring + damper
    // -----------------------------------------------------------------------
    // Angular velocity vector in world frame: (pitchRate about X, yawRate about Y, rollRate about Z)
    glm::vec3 angVel { pitchRate_, yawRate_, rollRate_ };

    float totalNz   = 0.f;
    float frontNz   = 0.f;

    for (int c = 0; c < 4; ++c) {
        // Corner attachment point in world space
        glm::vec3 bodyPos = cornerBodyPos(c);
        glm::vec3 cornerW = pos_ + R * bodyPos;

        // Wheel hub position: directly below corner along world Y
        // (simplified: spring acts vertically regardless of body roll)
        glm::vec3 hubW = cornerW;
        hubW.y = rw;   // hub sits at wheel_radius above ground (ground at Y=0)

        // Spring: acts from hub upward to corner
        float springLen = cornerW.y - hubW.y;
        springLen = std::max(springLen, 0.01f);  // prevent singularity

        float deflection = params_.springRestLengthM - springLen;

        // Damper: rate of change of deflection = -(rate of change of spring length)
        float prevDefl = params_.springRestLengthM - susp_[c].prevLength;
        float deflRate = (deflection - prevDefl) / dt;

        bool front = (c == FL || c == FR);
        float k = front ? params_.springRateFrontNm : params_.springRateRearNm;
        float d = front ? params_.damperFrontNsm    : params_.damperRearNsm;

        float Nz = k * deflection + d * deflRate;
        Nz = std::max(0.f, Nz);   // can't pull; wheel lifts off

        susp_[c].currentLength = springLen;
        susp_[c].prevLength    = springLen;
        susp_[c].normalForce   = Nz;
        wheels_[c].normalForce = Nz;
        wheels_[c].inContact   = (Nz > 0.f);

        totalNz += Nz;
        if (c == FL || c == FR) frontNz += Nz;
    }

    // -----------------------------------------------------------------------
    // 4. Tyre forces
    // -----------------------------------------------------------------------
    glm::vec3 totalTyreForce  { 0.f };
    glm::vec3 totalTyreTorque { 0.f };  // torque about CG

    // Drivetrain: compute drive torque per wheel
    float gearRatio  = params_.gearRatios[currentGear_ - 1];
    float tEngine    = engineTorque(input.throttle, engineRpm_);
    float driveAtWhl = tEngine * gearRatio * params_.finalDriveRatio * params_.drivetrainEff;

    // AWD split: front 45%, rear 55%
    float driveFrac[4] = {
        params_.frontTorqueFrac / 2.f,          // FL
        params_.frontTorqueFrac / 2.f,           // FR
        (1.f - params_.frontTorqueFrac) / 2.f,  // RL
        (1.f - params_.frontTorqueFrac) / 2.f   // RR
    };

    // Brake torque per wheel
    float brakeTotal = input.brake * params_.maxBrakeN;
    float brakeFwdFrac[4] = {
        params_.brakeBiasFront / 2.f,
        params_.brakeBiasFront / 2.f,
        (1.f - params_.brakeBiasFront) / 2.f,
        (1.f - params_.brakeBiasFront) / 2.f
    };

    for (int c = 0; c < 4; ++c) {
        if (!wheels_[c].inContact) continue;

        float Nz = wheels_[c].normalForce;
        float Fmax = params_.tyreMu * Nz;

        // Wheel forward direction in world space
        // (front wheels are steered; rear wheels point straight)
        float totalYaw = yaw_ + wheels_[c].steerAngle;
        glm::vec3 wFwd { std::sin(totalYaw), 0.f, std::cos(totalYaw) };
        glm::vec3 wLat { std::cos(totalYaw), 0.f, -std::sin(totalYaw) }; // left of wheel

        // Contact point position and velocity.
        // We use only the CG translational velocity for tyre slip calculations.
        // Including angular velocity (ω × r) would make the tyre model produce forces
        // from body pitch/roll oscillations even at standstill, causing phantom drift.
        glm::vec3 bodyPos    = cornerBodyPos(c);
        glm::vec3 cornerW    = pos_ + R * bodyPos;
        glm::vec3 contactPt  = cornerW; contactPt.y = 0.f;
        glm::vec3 rContact   = contactPt - pos_;
        glm::vec3 vContact   = vel_;   // translational velocity only

        float vLong = glm::dot(vContact, wFwd);   // forward speed at contact
        float vLat  = glm::dot(vContact, wLat);   // lateral speed at contact

        float vLongAbs = std::abs(vLong);
        // Ramp lateral grip to zero at very low speed to prevent steer-at-rest spinning.
        // Longitudinal grip is NOT ramped — car must be able to accelerate from standstill.
        float latRamp = std::min(1.f, vLongAbs / 1.5f);  // 0→1 over 0–1.5 m/s

        // --- Longitudinal slip ratio (well-conditioned, bounded ~[-1, 1]) ---
        float vWheel = wheels_[c].spinRate * rw;
        float kappa  = (vWheel - vLong)
                     / (std::max(std::abs(vWheel), vLongAbs) + EPS);

        // Longitudinal force (tanh traction curve)
        float Fx_raw = Fmax * std::tanh(kappa / params_.tyreLongPeakSlip);

        // --- Lateral slip angle (only meaningful when moving) ---
        float alpha = (vLongAbs > EPS)
                    ? -std::atan2(vLat, vLongAbs)
                    : 0.f;
        alpha *= latRamp;  // suppress at low speed to prevent yaw instability at rest

        // Lateral force (linear with friction saturation)
        float Fy_raw = params_.tyreLatStiff * alpha;
        Fy_raw = std::clamp(Fy_raw, -Fmax, Fmax);

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

        wheels_[c].Fx = Fx_raw;
        wheels_[c].Fy = Fy_raw;

        // --- Wheel spin integration ---
        float T_drive = driveAtWhl * driveFrac[c];
        // Brake torque always opposes forward spin.
        // When wheel is stopped (spinRate=0), no torque needed — max(0) clamp holds it.
        float T_brake = -brakeTotal * brakeFwdFrac[c];
        // Ground reaction torque: the longitudinal tyre force reacts on the wheel
        float T_ground = -Fx_raw * rw;
        float alpha_wheel = (T_drive + T_brake + T_ground) / params_.wheelInertiaKgM2;
        wheels_[c].spinRate += alpha_wheel * dt;
        // Clamp reverse spin (no reverse in this POC)
        wheels_[c].spinRate = std::max(0.f, wheels_[c].spinRate);
    }

    // -----------------------------------------------------------------------
    // 5. Suspension forces on body (vertical only, simplified)
    // -----------------------------------------------------------------------
    glm::vec3 totalSuspForce  { 0.f };
    glm::vec3 totalSuspTorque { 0.f };

    for (int c = 0; c < 4; ++c) {
        float Nz = susp_[c].normalForce;
        glm::vec3 F_susp { 0.f, Nz, 0.f };  // acts upward on body

        glm::vec3 bodyPos  = cornerBodyPos(c);
        glm::vec3 cornerW  = pos_ + R * bodyPos;
        glm::vec3 rCorner  = cornerW - pos_;

        totalSuspForce  += F_susp;
        totalSuspTorque += glm::cross(rCorner, F_susp);
    }

    // -----------------------------------------------------------------------
    // 6. Aerodynamic drag
    // -----------------------------------------------------------------------
    float speed = glm::length(vel_);
    glm::vec3 dragForce { 0.f };
    if (speed > EPS) {
        float dragMag = 0.5f * params_.airDensityKgM3 * params_.Cd
                      * params_.frontalAreaM2 * speed * speed;
        dragForce = -(vel_ / speed) * dragMag;
    }

    // -----------------------------------------------------------------------
    // 7. Equations of motion
    // -----------------------------------------------------------------------
    glm::vec3 gravity { 0.f, -m * G, 0.f };
    glm::vec3 totalForce  = gravity + totalSuspForce + totalTyreForce + dragForce;
    glm::vec3 totalTorque = totalSuspTorque + totalTyreTorque;

    glm::vec3 linAccel = totalForce / m;

    // Angular acceleration (world frame, diagonal inertia approximation)
    // torque.x → pitch (about world X = right)  → divided by Ipitch
    // torque.y → yaw   (about world Y = up)      → divided by Iyaw
    // torque.z → roll  (about world Z = forward) → divided by Iroll
    float alphaPitch = totalTorque.x / Ipitch;
    float alphaYaw   = totalTorque.y / Iyaw;
    float alphaRoll  = totalTorque.z / Iroll;

    // -----------------------------------------------------------------------
    // 8. Integration
    // -----------------------------------------------------------------------
    vel_  += linAccel * dt;
    pos_  += vel_     * dt;

    // Clamp body below ground (shouldn't happen but safety net)
    pos_.y = std::max(pos_.y, params_.cgHeightM * 0.5f);

    pitchRate_ += alphaPitch * dt;
    yawRate_   += alphaYaw   * dt;
    rollRate_  += alphaRoll  * dt;

    pitch_ += pitchRate_ * dt;
    yaw_   += yawRate_   * dt;
    roll_  += rollRate_  * dt;

    // Stabilising damping (prevents numerical runaway on roll/pitch)
    rollRate_  *= 0.98f;
    pitchRate_ *= 0.98f;

    // Clamp roll/pitch to realistic limits (~0.2 rad = ~11 deg)
    roll_  = std::clamp(roll_,  -0.20f, 0.20f);
    pitch_ = std::clamp(pitch_, -0.15f, 0.15f);

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
    float vFwd = glm::dot(vel_, fwd);  // speed along heading

    state_.position      = pos_;
    state_.velocity      = vel_;
    state_.headingRad    = yaw_;
    state_.speedMs       = vFwd;
    state_.slipAngleRad  = (wheels_[FL].inContact && std::abs(vFwd) > EPS)
                         ? std::atan2(glm::dot(vel_, right), std::abs(vFwd))
                         : 0.f;
    state_.throttle      = input.throttle;
    state_.brake         = input.brake;
    state_.steer         = input.steer;
    state_.engineRpm     = engineRpm_;
    state_.currentGear   = currentGear_;
    state_.rollRad       = roll_;
    state_.pitchRad      = pitch_;
    state_.weightFront   = (totalNz > 0.f) ? (frontNz / totalNz) : 0.5f;
    state_.odoMeters    += std::max(0.f, vFwd) * dt;

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
