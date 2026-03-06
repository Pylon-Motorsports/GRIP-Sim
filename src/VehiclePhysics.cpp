#include "VehiclePhysics.h"
#include <cmath>
#include <algorithm>

void VehiclePhysics::init()
{
    // Wheel local offsets relative to body CG.
    float dy = Vehicle::WHEEL_Y - Vehicle::BODY_Y;  // negative (wheels below CG)
    wheels_[0] = {};  wheels_[0].localOffset = {-Vehicle::HALF_TRACK, dy,  Vehicle::FRONT_AXLE};
    wheels_[1] = {};  wheels_[1].localOffset = { Vehicle::HALF_TRACK, dy,  Vehicle::FRONT_AXLE};
    wheels_[2] = {};  wheels_[2].localOffset = {-Vehicle::HALF_TRACK, dy, -Vehicle::REAR_AXLE};
    wheels_[3] = {};  wheels_[3].localOffset = { Vehicle::HALF_TRACK, dy, -Vehicle::REAR_AXLE};

    for (auto& w : wheels_) {
        w.massKg = 20.f;
        w.tire.radius = Vehicle::WHEEL_RADIUS;
    }

    // Suspension mounting points: nearest point on body surface to each wheel.
    // Body extends ±BODY_HALF_W in X, ±BODY_HALF_H in Y, ±BODY_HALF_L in Z.
    // Mount at body side wall (X = ±BODY_HALF_W), bottom edge (Y = -BODY_HALF_H),
    // at the axle Z position.
    float mountY = -Vehicle::BODY_HALF_H;  // bottom of body
    suspensions_[0].mountPoint = {-Vehicle::BODY_HALF_W, mountY,  Vehicle::FRONT_AXLE};
    suspensions_[1].mountPoint = { Vehicle::BODY_HALF_W, mountY,  Vehicle::FRONT_AXLE};
    suspensions_[2].mountPoint = {-Vehicle::BODY_HALF_W, mountY, -Vehicle::REAR_AXLE};
    suspensions_[3].mountPoint = { Vehicle::BODY_HALF_W, mountY, -Vehicle::REAR_AXLE};

    // Moments of inertia for a uniform box: I = m*(a^2+b^2)/12
    float m = bodyMassKg_;
    float w2 = (2.f * Vehicle::BODY_HALF_W) * (2.f * Vehicle::BODY_HALF_W);
    float h2 = (2.f * Vehicle::BODY_HALF_H) * (2.f * Vehicle::BODY_HALF_H);
    float l2 = (2.f * Vehicle::BODY_HALF_L) * (2.f * Vehicle::BODY_HALF_L);
    pitchInertia_ = m * (h2 + l2) / 12.f;  // rotation about X axis
    rollInertia_  = m * (h2 + w2) / 12.f;   // rotation about Z axis

    // Start resting on ground
    pos_.y = Vehicle::BODY_Y;

    // Engine torque curve (500cc single-cylinder, dune buggy style)
    engine_.torqueCurve = TorqueCurve({
        { 1000.f, 30.f },
        { 2000.f, 55.f },
        { 3000.f, 72.f },
        { 3500.f, 80.f },
        { 4500.f, 75.f },
        { 5500.f, 60.f },
        { 6500.f, 45.f },
    });
    engine_.gearRatio       = 4.0f;
    engine_.idleRpm         = 1200.f;
    engine_.clutchEngageRpm = 1800.f;
    engine_.clutchFullRpm   = 2500.f;
    engine_.rpmLimit        = 6500.f;
    engine_.engineBrakingNm = 15.f;
    engine_.rpm             = 1200.f;
}

void VehiclePhysics::reset()
{
    pos_ = {0.f, Vehicle::BODY_Y, 0.f};
    vel_ = {0.f, 0.f, 0.f};
    heading_ = 0.f;
    forwardSpeed_ = 0.f;
    pitch_ = 0.f;
    roll_ = 0.f;
    pitchRate_ = 0.f;
    rollRate_ = 0.f;
    engine_.rpm = engine_.idleRpm;
    for (auto& w : wheels_) {
        w.angularVel = 0.f;
        w.tire.normalLoad = 0.f;
    }
}

void VehiclePhysics::update(float dt, const InputState& input)
{
    // --- Forward speed (projection of velocity onto heading) ---
    glm::vec3 fwd = forwardDir();
    forwardSpeed_ = glm::dot(vel_, fwd);

    // --- Engine ---
    float rearAngVel = (wheels_[2].angularVel + wheels_[3].angularVel) * 0.5f;
    float totalDriveTorque = engine_.update(input.throttle, rearAngVel, dt);
    float perWheelDrive = totalDriveTorque * 0.5f;

    // Drivetrain friction
    float eps = wheels_[2].tire.epsilon;
    float regSign = forwardSpeed_ / (std::abs(forwardSpeed_) + eps);
    float drivetrainDrag = 0.f;
    if (input.throttle < 0.01f) {
        drivetrainDrag = -(drivetrainFrictionNm_ / wheels_[2].tire.radius) * regSign;
    }

    // --- Vertical forces (gravity + ground normal) ---
    glm::vec3 bodyForce{0.f, -bodyMassKg_ * GRAVITY, 0.f};
    glm::vec3 bodyTorque{0.f};  // torque about CG in body-local frame

    for (int i = 0; i < 4; ++i) {
        glm::vec3 wpos = wheelWorldPos(i);
        float gy = wheelGroundY(i);
        glm::vec3 vForce = wheels_[i].computeVerticalForces(wpos, vel_.y, gy);
        glm::vec3 transmitted = suspensions_[i].transmitToBody(vForce);
        bodyForce += transmitted;
        bodyTorque += suspensions_[i].torqueOnBody(transmitted);
    }

    // --- Longitudinal forces (drive, brake, friction) ---
    float totalLongForce = 0.f;
    for (int i = 0; i < 4; ++i) {
        glm::vec3 wpos = wheelWorldPos(i);
        float gy = wheelGroundY(i);
        bool grounded = wheels_[i].onGround(wpos, gy);

        float drive = (i >= 2) ? perWheelDrive : 0.f;

        float longForce = wheels_[i].computeLongitudinalForce(
            drive, input.brake, forwardSpeed_, grounded);

        totalLongForce += suspensions_[i].transmitToBody(glm::vec3{0, 0, longForce}).z;

        // Longitudinal force creates pitch torque via mounting point
        // Force is along body Z, so torque = mountPoint × (0,0,F)
        // This gives pitch torque = mountPoint.y * F (around X axis)
        bodyTorque.x += suspensions_[i].mountPoint.y * longForce;
    }

    totalLongForce += drivetrainDrag;
    bodyForce += fwd * totalLongForce;

    // --- Angular damping (prevents oscillation) ---
    constexpr float PITCH_DAMP = 800.f;  // N·m·s/rad
    constexpr float ROLL_DAMP  = 600.f;
    bodyTorque.x -= PITCH_DAMP * pitchRate_;
    bodyTorque.z -= ROLL_DAMP  * rollRate_;

    // --- Integrate linear ---
    float mass = totalMass();
    glm::vec3 accel = bodyForce / mass;
    vel_ += accel * dt;
    pos_ += vel_ * dt;

    // --- Integrate angular ---
    float pitchAccel = bodyTorque.x / pitchInertia_;
    float rollAccel  = bodyTorque.z / rollInertia_;
    pitchRate_ += pitchAccel * dt;
    rollRate_  += rollAccel  * dt;
    pitch_ += pitchRate_ * dt;
    roll_  += rollRate_  * dt;

    // Clamp to reasonable range (prevent runaway)
    constexpr float MAX_ANGLE = 0.15f;  // ~8.6 degrees
    pitch_ = std::clamp(pitch_, -MAX_ANGLE, MAX_ANGLE);
    roll_  = std::clamp(roll_,  -MAX_ANGLE, MAX_ANGLE);

    // Update forward speed after integration
    forwardSpeed_ = glm::dot(vel_, fwd);
}

void VehiclePhysics::fillVehicle(Vehicle& veh) const
{
    veh.position = pos_;
    veh.heading  = heading_;
    veh.pitch    = pitch_;
    veh.roll     = roll_;
    for (int i = 0; i < 4; ++i) {
        veh.wheelPos[i] = wheelWorldPos(i);
        veh.mountPos[i] = mountWorldPos(i);
    }
}

float VehiclePhysics::wheelGroundY(int i) const
{
    if (!bumps_ || bumps_->empty()) return 0.f;
    glm::vec3 wpos = wheelWorldPos(i);
    return groundHeight(*bumps_, wpos.x, wpos.z);
}

glm::vec3 VehiclePhysics::wheelWorldPos(int i) const
{
    return pos_ + rotateByBody(wheels_[i].localOffset);
}

glm::vec3 VehiclePhysics::mountWorldPos(int i) const
{
    return pos_ + rotateByBody(suspensions_[i].mountPoint);
}

// Rotate a body-local vector by heading, pitch, and roll.
// Order: roll (about Z) → pitch (about X) → heading (about Y)
glm::vec3 VehiclePhysics::rotateByBody(const glm::vec3& v) const
{
    // Roll about local Z
    float sr = std::sin(roll_), cr = std::cos(roll_);
    glm::vec3 r1{v.x * cr - v.y * sr, v.x * sr + v.y * cr, v.z};

    // Pitch about local X
    float sp = std::sin(pitch_), cp = std::cos(pitch_);
    glm::vec3 r2{r1.x, r1.y * cp - r1.z * sp, r1.y * sp + r1.z * cp};

    // Heading about world Y
    float sh = std::sin(heading_), ch = std::cos(heading_);
    return {r2.x * ch - r2.z * sh, r2.y, r2.x * sh + r2.z * ch};
}

glm::vec3 VehiclePhysics::forwardDir() const
{
    return {std::sin(heading_), 0.f, std::cos(heading_)};
}

float VehiclePhysics::totalMass() const
{
    float m = bodyMassKg_;
    for (int i = 0; i < 4; ++i) m += wheels_[i].massKg;
    return m;
}
