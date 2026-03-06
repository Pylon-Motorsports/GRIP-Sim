#include "SemiRealisticVehicle.h"
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <algorithm>

static constexpr float G = 9.81f;

SemiRealisticVehicle::SemiRealisticVehicle(VehicleParams params)
    : params_(params)
{
    reset({}, 0.f);
}

void SemiRealisticVehicle::reset(glm::vec3 position, float headingRad)
{
    state_          = VehicleState{};
    state_.position = position;
    state_.headingRad = headingRad;
    vx_ = vy_ = yawRate_ = prevSpeedMs_ = 0.f;
    state_.currentGear = 1;
    state_.engineRpm   = params_.idleRpm;
}

// ---------------------------------------------------------------------------
// Simple torque curve: peak at 60% of max RPM, tapers at top end
// ---------------------------------------------------------------------------
float SemiRealisticVehicle::engineTorque(float throttle, float rpm) const
{
    float rpmNorm = rpm / params_.maxRpm;  // 0..1
    float curve = 1.f - 0.4f * (rpmNorm - 0.6f) * (rpmNorm - 0.6f) / 0.36f;
    curve = std::clamp(curve, 0.2f, 1.f);
    return throttle * params_.maxTorqueNm * curve;
}

// ---------------------------------------------------------------------------
// Auto gear shift
// ---------------------------------------------------------------------------
void SemiRealisticVehicle::autoShift()
{
    if (state_.engineRpm > params_.upshiftRpm && state_.currentGear < params_.numGears)
        ++state_.currentGear;
    else if (state_.engineRpm < params_.downshiftRpm && state_.currentGear > 1)
        --state_.currentGear;
}

// ---------------------------------------------------------------------------
// integrate — bicycle model, RWD
//
// Coordinate conventions (body frame, y-right):
//   vx_     > 0 = forward
//   vy_     > 0 = rightward (positive x in world when heading=0)
//   yawRate_> 0 = turning right  (heading increases → car points toward +X)
//
// Slip angle sign convention:
//   Front contact point lateral velocity: vy_ - yawRate_*lf
//     (front swings LEFT when turning right → negative for right turn)
//   Rear contact point lateral velocity:  vy_ + yawRate_*lr
//     (rear  swings RIGHT when turning right → positive for right turn)
//   Lateral tyre force: Fy = -C * alpha  (resists slip → centripetal for turns)
//
// Yaw moment:
//   Mz (right-hand about +Y/up): negative = CW = right turn
//   yawRate_ -= Mz/Iz  (so negative Mz increases yawRate_, turning right)
// ---------------------------------------------------------------------------
void SemiRealisticVehicle::integrate(const InputFrame& input, float dt)
{
    const float mass  = params_.massKg;
    const float lf    = params_.lfDistM;
    const float lr    = params_.lrDistM;
    const float wb    = params_.wheelbaseM;
    const float Iz    = params_.momentOfInertiaKgM2;
    const float rw    = params_.wheelRadiusM;
    const float Caf   = params_.corneringStiffFront;
    const float Car   = params_.corneringStiffRear;
    const float Nref  = params_.peakNormalForceN;

    // --- Weight transfer ---
    float longAccel   = (vx_ - prevSpeedMs_) / dt;
    prevSpeedMs_      = vx_;
    float deltaWeight = -(mass * longAccel * params_.cgHeightM) / wb;
    float Nf          = std::max(0.f, mass * G * (lr / wb) + deltaWeight);
    float Nr          = std::max(0.f, mass * G * (lf / wb) - deltaWeight);

    // --- Drive force (RWD) ---
    float gearRatio  = params_.gearRatios[state_.currentGear - 1];
    float torque     = engineTorque(input.throttle, state_.engineRpm);
    float driveForce = torque * gearRatio * params_.diffRatio
                     * params_.drivetrainEfficiency / rw;

    // --- Braking force ---
    float brakeForceFront = input.brake * params_.maxBrakeForceN * params_.brakeBiasFront;
    float brakeForceRear  = input.brake * params_.maxBrakeForceN * (1.f - params_.brakeBiasFront);
    float totalBrake      = brakeForceFront + brakeForceRear;

    // --- Aerodynamic drag ---
    float drag = 0.5f * params_.airDensityKgM3 * params_.Cd * params_.frontalAreaM2
               * vx_ * std::abs(vx_);

    // --- Steering angle: +input.steer=right, +steerAngle=right (consistent) ---
    float steerAngle = input.steer * params_.maxSteerAngleRad;

    // --- Slip angles (body frame, y-right convention) ---
    float alpha_f = 0.f, alpha_r = 0.f;
    float Fy_f = 0.f,    Fy_r = 0.f;

    if (std::abs(vx_) > 0.5f) {
        // Front contact point lateral velocity:
        //   front swings LEFT when turning right → vy_ - yawRate_*lf
        float v_lat_f = vy_ - yawRate_ * lf;
        // Rear contact point lateral velocity:
        //   rear swings RIGHT when turning right → vy_ + yawRate_*lr
        float v_lat_r = vy_ + yawRate_ * lr;

        alpha_f = steerAngle - std::atan2(v_lat_f, std::abs(vx_));
        alpha_r =              std::atan2(v_lat_r, std::abs(vx_));

        // Linear cornering with peak-load saturation
        float sat_f = std::min(1.f, Nf / Nref);
        float sat_r = std::min(1.f, Nr / Nref);
        Fy_f = -Caf * alpha_f * sat_f;
        Fy_r = -Car * alpha_r * sat_r;

        // Friction circle: cap lateral force at tyre friction limit (μ × normal load).
        // Prevents the linear model from producing unrealistic forces at large slip angles
        // (e.g. full keyboard steer at low speed → correct physics, no spin-out).
        static constexpr float TYRE_MU = 1.2f;
        Fy_f = std::clamp(Fy_f, -TYRE_MU * Nf, TYRE_MU * Nf);
        Fy_r = std::clamp(Fy_r, -TYRE_MU * Nr, TYRE_MU * Nr);

        // Low-speed ramp: smoothly bring lateral forces to zero below 3 m/s.
        // Avoids the force discontinuity at the 0.5 m/s threshold and prevents
        // the car spinning out from a near-standstill steer input.
        float speedRamp = std::min(1.f, (std::abs(vx_) - 0.5f) / 2.5f);
        Fy_f *= speedRamp;
        Fy_r *= speedRamp;
    }

    state_.slipAngleRad = alpha_f;

    // --- Equations of motion (body frame, y-right) ---
    float netX = driveForce - totalBrake - drag + Fy_f * std::sin(steerAngle);
    float netY = Fy_f * std::cos(steerAngle) + Fy_r;
    // Mz: right-hand moment about +Y (up); negative = CW = turning right
    float Mz   = Fy_f * std::cos(steerAngle) * lf - Fy_r * lr;

    float ax    = netX / mass + vy_ * yawRate_;
    float ay    = netY / mass - vx_ * yawRate_;
    float alphaZ= Mz  / Iz;

    vx_      += ax     * dt;
    vy_      += ay     * dt;
    // Note: -= because negative Mz (CW) should INCREASE yawRate_ (right turn)
    yawRate_ -= alphaZ * dt;

    // Clamp reverse (no reverse gear in POC)
    vx_ = std::max(0.f, vx_);

    // Lateral damping for numerical stability
    vy_      *= 0.98f;
    yawRate_ *= 0.995f;

    // --- Integrate world position & heading ---
    float fwd_x = std::sin(state_.headingRad);
    float fwd_z = std::cos(state_.headingRad);
    glm::vec3 forward { fwd_x, 0.f, fwd_z };
    glm::vec3 right   { fwd_z, 0.f, -fwd_x };

    state_.position   += (forward * vx_ + right * vy_) * dt;
    state_.headingRad += yawRate_ * dt;
    state_.speedMs     = vx_;

    // --- RPM update ---
    float wheelRPM  = (vx_ / rw) * (60.f / (2.f * glm::pi<float>()));
    state_.engineRpm = std::max(params_.idleRpm, wheelRPM * gearRatio * params_.diffRatio);
    autoShift();

    // --- Copy input state for observers ---
    state_.throttle = input.throttle;
    state_.brake    = input.brake;
    state_.steer    = input.steer;
    state_.odoMeters += vx_ * dt;

    // Weight front ratio (for body roll visual feedback via camera)
    float totalN = Nf + Nr;
    state_.weightFront = (totalN > 0.f) ? Nf / totalN : 0.5f;

    updateSegmentTracking();
}

// ---------------------------------------------------------------------------
// Segment tracking
// ---------------------------------------------------------------------------
void SemiRealisticVehicle::setCenterlinePoints(
    const std::vector<glm::vec3>& points,
    const std::vector<uint32_t>&  segmentStartVertex,
    int vertsPerRow)
{
    centerline_         = points;
    segmentStartVertex_ = segmentStartVertex;
    vertsPerRow_        = vertsPerRow;
}

void SemiRealisticVehicle::updateSegmentTracking()
{
    if (centerline_.empty()) return;

    float minDist = 1e9f;
    int   bestIdx = 0;
    for (int i = 0; i < (int)centerline_.size(); ++i) {
        glm::vec3 d = state_.position - centerline_[i];
        d.y = 0.f;
        float dist = glm::dot(d, d);
        if (dist < minDist) {
            minDist = dist;
            bestIdx = i;
        }
    }

    int seg = 0;
    for (int s = (int)segmentStartVertex_.size() - 1; s >= 0; --s) {
        int firstPoint = segmentStartVertex_[s] / vertsPerRow_;
        if (bestIdx >= firstPoint) {
            seg = s;
            break;
        }
    }

    state_.segmentIndex    = seg;
    state_.segmentProgress = 0.f;
}
