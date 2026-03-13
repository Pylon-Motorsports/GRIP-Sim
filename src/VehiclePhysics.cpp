#include "VehiclePhysics.hpp"
#include <cmath>
#include <algorithm>

void VehiclePhysics::init()
{
    // Wheel local offsets relative to body CG.
    float dy = Vehicle::WHEEL_Y - Vehicle::BODY_Y;  // negative (wheels below CG)
    wheels_[0].localOffset = {-Vehicle::HALF_TRACK, dy,  Vehicle::FRONT_AXLE};
    wheels_[1].localOffset = { Vehicle::HALF_TRACK, dy,  Vehicle::FRONT_AXLE};
    wheels_[2].localOffset = {-Vehicle::HALF_TRACK, dy, -Vehicle::REAR_AXLE};
    wheels_[3].localOffset = { Vehicle::HALF_TRACK, dy, -Vehicle::REAR_AXLE};

    for (auto& w : wheels_) {
        w.massKg = 20.f;
        w.tire = std::make_unique<BrushTire>();
        w.tire->radius = Vehicle::WHEEL_RADIUS;
        w.tire->width  = Vehicle::WHEEL_HALF_W * 2.f;
    }

    // Front calipers: larger pistons + bigger discs than rear
    wheels_[0].brakePistonAreaM2 = 0.0040f;  // 40 cm²
    wheels_[1].brakePistonAreaM2 = 0.0040f;
    wheels_[0].brakeDiscRadiusM  = 0.16f;    // 320mm disc
    wheels_[1].brakeDiscRadiusM  = 0.16f;
    // Rear
    wheels_[2].brakePistonAreaM2 = 0.0030f;  // 30 cm²
    wheels_[3].brakePistonAreaM2 = 0.0030f;
    wheels_[2].brakeDiscRadiusM  = 0.15f;    // 300mm disc
    wheels_[3].brakeDiscRadiusM  = 0.15f;

    // Subframe setup
    float mountY = -Vehicle::BODY_HALF_H;

    front_.wheel[0] = &wheels_[0];
    front_.wheel[1] = &wheels_[1];
    front_.suspension[0].mountPoint = {-Vehicle::BODY_HALF_W, mountY,  Vehicle::FRONT_AXLE};
    front_.suspension[1].mountPoint = { Vehicle::BODY_HALF_W, mountY,  Vehicle::FRONT_AXLE};

    rear_.wheel[0] = &wheels_[2];
    rear_.wheel[1] = &wheels_[3];
    rear_.suspension[0].mountPoint = {-Vehicle::BODY_HALF_W, mountY, -Vehicle::REAR_AXLE};
    rear_.suspension[1].mountPoint = { Vehicle::BODY_HALF_W, mountY, -Vehicle::REAR_AXLE};
    rear_.lsdLockRatio = 0.25f;  // mild 1-way LSD on rear axle

    // Body inertia
    body_.computeInertia();
    body_.pos.y = Vehicle::BODY_Y;

    // Drivetrain (engine, gearbox)
    drivetrain_.init();
}

void VehiclePhysics::reset()
{
    body_.reset();
    front_.steerAngle = 0.f;
    drivetrain_.reset();
    for (auto& w : wheels_) {
        w.angularVel = 0.f;
        w.tire->deflection = 0.f;
        w.tire->normalLoad = 0.f;
    }
}

void VehiclePhysics::update(float dt, const InputState& input)
{
    // --- Body velocity in body-local frame ---
    BodyState bs = body_.bodyState(0.f);
    glm::vec3 fwd = bs.forwardDir();
    glm::vec3 rht = bs.rightDir();
    body_.forwardSpeed = glm::dot(body_.vel, fwd);
    float lateralSpeed = glm::dot(body_.vel, rht);
    bs.forwardSpeed = body_.forwardSpeed;
    bs.lateralSpeed = lateralSpeed;

    // --- Steering ---
    front_.steerAngle = drivetrain_.computeSteerAngle(
        input.steer, front_.steerAngle, body_.forwardSpeed, dt);
    rear_.steerAngle = 0.f;

    // --- Engine / drive torque (RWD: rear wheels) ---
    float perWheelDrive = drivetrain_.computeDriveTorque(
        input.throttle, input.clutchIn, wheels_[2], wheels_[3], dt);

    // --- Query terrain at wheel positions ---
    float groundY[4];
    glm::vec3 surfNormal[4];

    for (int i = 0; i < 4; ++i) {
        Subframe& sf = (i < 2) ? front_ : rear_;
        int side = (i < 2) ? i : i - 2;
        glm::vec3 wpos = sf.wheelWorldPos(side, bs);

        if (terrain_) {
            groundY[i]   = terrain_->heightAt(wpos.x, wpos.z);
            surfNormal[i] = terrain_->normalAt(wpos.x, wpos.z);
        } else {
            groundY[i]   = 0.f;
            surfNormal[i] = {0.f, 1.f, 0.f};
        }
    }

    // --- Brake pressure with proportioning valve ---
    float frontPressure, rearPressure;
    drivetrain_.computeBrakePressure(input.brake, frontPressure, rearPressure);

    // Handbrake: rear-only, full pressure, bypasses proportioning valve
    float handbrakeP = input.handbrake * Drivetrain::MAX_BRAKE_PRESSURE;
    rearPressure = std::max(rearPressure, handbrakeP);

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
    glm::vec3 bodyForce{0.f, -body_.massKg * Body::GRAVITY, 0.f};
    bodyForce += frontResult.bodyForce + rearResult.bodyForce;

    glm::vec3 bodyTorque = frontResult.bodyTorque + rearResult.bodyTorque;

    // --- Body collider (wall contact) ---
    if (terrain_)
        body_.applyCollider(*terrain_, bs, bodyForce, bodyTorque);

    // --- Aerodynamic forces ---
    body_.applyAero(fwd, bodyForce, bodyTorque);

    // --- Integrate ---
    body_.integrate(bodyForce, bodyTorque, totalMass(), dt);

    // Update forward speed after integration
    body_.forwardSpeed = glm::dot(body_.vel, bs.forwardDir());
}

void VehiclePhysics::fillVehicle(Vehicle& veh) const
{
    veh.position = body_.pos;
    veh.heading  = body_.heading;
    veh.pitch    = body_.pitch;
    veh.roll     = body_.roll;
    veh.frontSteerAngle = front_.steerAngle;

    BodyState bs = body_.bodyState(0.f);

    veh.bodyRotation = glm::mat3(
        bs.rotateLocal({1,0,0}),
        bs.rotateLocal({0,1,0}),
        bs.rotateLocal({0,0,1})
    );

    for (int i = 0; i < 4; ++i) {
        const Subframe& sf = (i < 2) ? front_ : rear_;
        int side = (i < 2) ? i : i - 2;
        veh.wheelPos[i] = sf.wheelWorldPos(side, bs);
        veh.mountPos[i] = sf.mountWorldPos(side, bs);

        veh.wheelSlipRatio[i]    = wheels_[i].lastSlipRatio;
        veh.wheelSlipAngle[i]    = wheels_[i].lastSlipAngle;
        veh.wheelNormalLoad[i]   = wheels_[i].tire->normalLoad;
        veh.wheelContactWidth[i] = wheels_[i].tire->width;
    }
}

float VehiclePhysics::totalMass() const
{
    float m = body_.massKg;
    for (int i = 0; i < 4; ++i) m += wheels_[i].massKg;
    return m;
}
