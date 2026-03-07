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
        w.tire.width  = Vehicle::WHEEL_HALF_W * 2.f;
    }

    // Front calipers: larger pistons + bigger discs than rear
    wheels_[0].brakePistonAreaM2 = 0.0022f;  // 22 cm²
    wheels_[1].brakePistonAreaM2 = 0.0022f;
    wheels_[0].brakeDiscRadiusM  = 0.15f;    // 300mm disc
    wheels_[1].brakeDiscRadiusM  = 0.15f;
    // Rear: defaults (16 cm², 280mm disc)

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
    rear_.lsdLockRatio = 0.25f;  // mild 1-way LSD on rear axle

    // Moments of inertia for a uniform box: I = m*(a^2+b^2)/12
    float m = bodyMassKg_;
    float w2 = (2.f * Vehicle::BODY_HALF_W) * (2.f * Vehicle::BODY_HALF_W);
    float h2 = (2.f * Vehicle::BODY_HALF_H) * (2.f * Vehicle::BODY_HALF_H);
    float l2 = (2.f * Vehicle::BODY_HALF_L) * (2.f * Vehicle::BODY_HALF_L);
    pitchInertia_ = m * (h2 + l2) / 12.f;  // rotation about X axis
    rollInertia_  = m * (h2 + w2) / 12.f;   // rotation about Z axis
    yawInertia_   = m * (w2 + l2) / 12.f;   // rotation about Y axis

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
    // 5-speed gearbox: final drive 3.9 × gear ratios
    // BRZ-inspired: 1st=3.63, 2nd=2.19, 3rd=1.54, 4th=1.13, 5th=0.91
    engine_.gearRatios[0] = 3.9f * 3.63f;  // 14.2 — 1st
    engine_.gearRatios[1] = 3.9f * 2.19f;  // 8.5  — 2nd
    engine_.gearRatios[2] = 3.9f * 1.54f;  // 6.0  — 3rd
    engine_.gearRatios[3] = 3.9f * 1.13f;  // 4.4  — 4th
    engine_.gearRatios[4] = 3.9f * 0.91f;  // 3.5  — 5th
    engine_.numGears      = 5;
    engine_.currentGear   = 0;
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
    yawRate_ = 0.f;
    front_.steerAngle = 0.f;
    engine_.rpm = engine_.idleRpm;
    engine_.currentGear = 0;
    for (auto& w : wheels_) {
        w.angularVel = 0.f;
        w.tire.deflection = 0.f;
        w.tire.normalLoad = 0.f;
    }
}

BodyState VehiclePhysics::buildBodyState(float lateralSpeed) const
{
    BodyState bs;
    bs.pos = pos_;
    bs.vel = vel_;
    bs.heading = heading_;
    bs.pitch = pitch_;
    bs.roll = roll_;
    bs.forwardSpeed = forwardSpeed_;
    bs.lateralSpeed = lateralSpeed;
    bs.pitchRate = pitchRate_;
    bs.rollRate = rollRate_;
    bs.yawRate = yawRate_;
    return bs;
}

void VehiclePhysics::update(float dt, const InputState& input)
{
    // --- Body velocity in body-local frame ---
    BodyState bs = buildBodyState(0.f);
    glm::vec3 fwd = bs.forwardDir();
    glm::vec3 rht = bs.rightDir();
    forwardSpeed_ = glm::dot(vel_, fwd);
    float lateralSpeed = glm::dot(vel_, rht);
    bs.forwardSpeed = forwardSpeed_;
    bs.lateralSpeed = lateralSpeed;

    // --- Steering (rate-limited: real steering rack has finite speed) ---
    float steerInput = applyDeadzone(input.steer, STEER_DEADZONE);
    float speedFactor = 1.f / (1.f + std::abs(forwardSpeed_) / 30.f);
    float targetSteer = steerInput * MAX_STEER_ANGLE * speedFactor;
    float maxDelta = MAX_STEER_RATE * dt;
    float delta = targetSteer - front_.steerAngle;
    front_.steerAngle += std::clamp(delta, -maxDelta, maxDelta);
    rear_.steerAngle = 0.f;

    // --- Engine (RWD: drive goes to rear subframe) ---
    float rearAngVel = (wheels_[2].angularVel + wheels_[3].angularVel) * 0.5f;
    float totalDriveTorque = engine_.update(input.throttle, input.clutchIn, rearAngVel, dt);
    float perWheelDrive = totalDriveTorque * 0.5f;

    // --- Query terrain at wheel positions (ask subframes for positions) ---
    float groundY[4];
    glm::vec3 surfNormal[4];

    for (int i = 0; i < 4; ++i) {
        Subframe& sf = (i < 2) ? front_ : rear_;
        int side = (i < 2) ? i : i - 2;
        glm::vec3 wpos = sf.wheelWorldPos(side, bs);

        if (bumps_ && !bumps_->empty()) {
            groundY[i]   = groundHeight(*bumps_, wpos.x, wpos.z);
            surfNormal[i] = groundNormal(*bumps_, wpos.x, wpos.z);
        } else {
            groundY[i]   = 0.f;
            surfNormal[i] = {0.f, 1.f, 0.f};
        }
    }

    // --- Brake pressure from pedal with front/rear bias ---
    float frontPressure = input.brake * MAX_BRAKE_PRESSURE * BRAKE_BIAS_FRONT;
    float rearPressure  = input.brake * MAX_BRAKE_PRESSURE * (1.f - BRAKE_BIAS_FRONT);
    float frontBrake[2] = { frontPressure, frontPressure };
    float rearBrake[2]  = { rearPressure,  rearPressure  };

    // --- Subframe force computation ---
    float frontDrive[2] = {0.f, 0.f};
    auto frontResult = front_.computeForces(
        bs, &groundY[0], &surfNormal[0], frontDrive, frontBrake, dt);

    float rearDrive[2] = {perWheelDrive, perWheelDrive};
    auto rearResult = rear_.computeForces(
        bs, &groundY[2], &surfNormal[2], rearDrive, rearBrake, dt);

    // --- Accumulate forces on body ---
    glm::vec3 bodyForce{0.f, -bodyMassKg_ * GRAVITY, 0.f};
    bodyForce += frontResult.bodyForce + rearResult.bodyForce;

    glm::vec3 bodyTorque = frontResult.bodyTorque + rearResult.bodyTorque;

    // --- Integrate linear ---
    float mass = totalMass();
    glm::vec3 accel = bodyForce / mass;
    vel_ += accel * dt;
    pos_ += vel_ * dt;

    // --- Integrate pitch and roll ---
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

    // --- Integrate yaw ---
    float yawAccel = bodyTorque.y / yawInertia_;
    yawRate_ += yawAccel * dt;
    heading_ += yawRate_ * dt;

    // Update forward speed after integration
    forwardSpeed_ = glm::dot(vel_, bs.forwardDir());
}

void VehiclePhysics::fillVehicle(Vehicle& veh) const
{
    veh.position = pos_;
    veh.heading  = heading_;
    veh.pitch    = pitch_;
    veh.roll     = roll_;
    veh.frontSteerAngle = front_.steerAngle;

    BodyState bs = buildBodyState(0.f);

    // Body rotation matrix — built from the same rotateLocal used for
    // wheel/mount positions, so everything is guaranteed consistent.
    veh.bodyRotation = glm::mat3(
        bs.rotateLocal({1,0,0}),   // column 0: body-right in world
        bs.rotateLocal({0,1,0}),   // column 1: body-up in world
        bs.rotateLocal({0,0,1})    // column 2: body-forward in world
    );

    for (int i = 0; i < 4; ++i) {
        const Subframe& sf = (i < 2) ? front_ : rear_;
        int side = (i < 2) ? i : i - 2;
        veh.wheelPos[i] = sf.wheelWorldPos(side, bs);
        veh.mountPos[i] = sf.mountWorldPos(side, bs);

        veh.wheelSlipRatio[i]    = wheels_[i].lastSlipRatio;
        veh.wheelSlipAngle[i]    = wheels_[i].lastSlipAngle;
        veh.wheelNormalLoad[i]   = wheels_[i].tire.normalLoad;
        veh.wheelContactWidth[i] = wheels_[i].tire.width;
    }
}

float VehiclePhysics::totalMass() const
{
    float m = bodyMassKg_;
    for (int i = 0; i < 4; ++i) m += wheels_[i].massKg;
    return m;
}

float VehiclePhysics::applyDeadzone(float value, float deadzone)
{
    if (std::abs(value) < deadzone) return 0.f;
    float sign = value > 0.f ? 1.f : -1.f;
    return sign * (std::abs(value) - deadzone) / (1.f - deadzone);
}
