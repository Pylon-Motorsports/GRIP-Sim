#include "VehiclePhysics.h"
#include <cmath>
#include <algorithm>

// Map wheel index (0-3) to its subframe and side (0=left, 1=right)
// Wheels: 0=FL, 1=FR, 2=RL, 3=RR

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
        w.tire.width  = Vehicle::WHEEL_HALF_W * 2.f;
    }

    // Subframe setup: wire wheel pointers and suspension mounting points
    float mountY = -Vehicle::BODY_HALF_H;  // bottom of body

    front_.wheel[0] = &wheels_[0];
    front_.wheel[1] = &wheels_[1];
    front_.suspension[0].mountPoint = {-Vehicle::BODY_HALF_W, mountY,  Vehicle::FRONT_AXLE};
    front_.suspension[1].mountPoint = { Vehicle::BODY_HALF_W, mountY,  Vehicle::FRONT_AXLE};

    rear_.wheel[0] = &wheels_[2];
    rear_.wheel[1] = &wheels_[3];
    rear_.suspension[0].mountPoint = {-Vehicle::BODY_HALF_W, mountY, -Vehicle::REAR_AXLE};
    rear_.suspension[1].mountPoint = { Vehicle::BODY_HALF_W, mountY, -Vehicle::REAR_AXLE};

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
        w.tire.deflection = 0.f;
        w.tire.normalLoad = 0.f;
    }
}

void VehiclePhysics::update(float dt, const InputState& input)
{
    // --- Forward speed (projection of velocity onto heading) ---
    glm::vec3 fwd = forwardDir();
    forwardSpeed_ = glm::dot(vel_, fwd);

    // --- Engine (RWD: drive goes to rear subframe) ---
    float rearAngVel = (wheels_[2].angularVel + wheels_[3].angularVel) * 0.5f;
    float totalDriveTorque = engine_.update(input.throttle, rearAngVel, dt);
    float perWheelDrive = totalDriveTorque * 0.5f;

    // --- Per-wheel tire forces ---
    // Compute forces for all 4 wheels, then route through subframes
    Wheel::Forces wf[4];
    float longF[4];

    for (int i = 0; i < 4; ++i) {
        glm::vec3 wpos = wheelWorldPos(i);
        float gy = wheelGroundY(i);
        float drive = (i >= 2) ? perWheelDrive : 0.f;

        wf[i] = wheels_[i].computeForces(
            wpos.y, vel_.y, gy, forwardSpeed_, drive, input.brake, dt);

        longF[i] = wf[i].longitudinalForce + wf[i].rollingResistance;
    }

    // --- Route through subframes ---
    glm::vec3 bodyForce{0.f, -bodyMassKg_ * GRAVITY, 0.f};
    glm::vec3 bodyTorque{0.f};

    // Front subframe
    {
        auto af = front_.transmitVertical(
            {0.f, wf[0].normalForce, 0.f},
            {0.f, wf[1].normalForce, 0.f});
        bodyForce  += af.bodyForce;
        bodyTorque += af.bodyTorque;
        bodyTorque.x += front_.pitchTorqueFromLongitudinal(longF[0], longF[1]);
    }

    // Rear subframe
    {
        auto af = rear_.transmitVertical(
            {0.f, wf[2].normalForce, 0.f},
            {0.f, wf[3].normalForce, 0.f});
        bodyForce  += af.bodyForce;
        bodyTorque += af.bodyTorque;
        bodyTorque.x += rear_.pitchTorqueFromLongitudinal(longF[2], longF[3]);
    }

    float totalLongForce = front_.transmitLongitudinal(longF[0], longF[1])
                         + rear_.transmitLongitudinal(longF[2], longF[3]);
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
    const Suspension& s = (i < 2) ? front_.suspension[i]
                                  : rear_.suspension[i - 2];
    return pos_ + rotateByBody(s.mountPoint);
}

// Rotate a body-local vector by heading, pitch, and roll.
// Order: roll (about Z) -> pitch (about X) -> heading (about Y)
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
