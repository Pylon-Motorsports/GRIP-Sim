#include "VehiclePhysics.h"
#include <cmath>
#include <algorithm>

void VehiclePhysics::init()
{
    // Wheel local offsets relative to body CG.
    // Body CG is at BODY_Y above ground; wheel centers at WHEEL_Y.
    float dy = Vehicle::WHEEL_Y - Vehicle::BODY_Y;  // negative (wheels below CG)
    wheels_[0] = { 20.f, Vehicle::WHEEL_RADIUS, {-Vehicle::HALF_TRACK, dy,  Vehicle::FRONT_AXLE} };
    wheels_[1] = { 20.f, Vehicle::WHEEL_RADIUS, { Vehicle::HALF_TRACK, dy,  Vehicle::FRONT_AXLE} };
    wheels_[2] = { 20.f, Vehicle::WHEEL_RADIUS, {-Vehicle::HALF_TRACK, dy, -Vehicle::REAR_AXLE}  };
    wheels_[3] = { 20.f, Vehicle::WHEEL_RADIUS, { Vehicle::HALF_TRACK, dy, -Vehicle::REAR_AXLE}  };

    // Start with wheels resting on the ground
    pos_.y = Vehicle::BODY_Y;
}

void VehiclePhysics::update(float dt)
{
    // --- Accumulate forces ---

    // Body gravity
    glm::vec3 bodyForce{0.f, -bodyMassKg_ * GRAVITY, 0.f};

    // Each wheel computes its own forces, then transmits through suspension to body
    for (int i = 0; i < 4; ++i) {
        glm::vec3 wpos = wheelWorldPos(i);
        glm::vec3 wheelForce = wheels_[i].computeForces(wpos, vel_.y, GROUND_Y);

        // Suspension transmits wheel force to body
        bodyForce += suspensions_[i].transmitToBody(wheelForce);
    }

    // --- Integrate ---
    float totalMass = bodyMassKg_;
    for (int i = 0; i < 4; ++i)
        totalMass += wheels_[i].massKg;

    glm::vec3 accel = bodyForce / totalMass;
    vel_ += accel * dt;
    pos_ += vel_ * dt;

    // --- Ground constraint (hard floor) ---
    // Lowest wheel bottom must not go below ground
    float lowestBottom = pos_.y + wheels_[0].localOffset.y - wheels_[0].radius;
    if (lowestBottom < GROUND_Y) {
        pos_.y += GROUND_Y - lowestBottom;
        if (vel_.y < 0.f) vel_.y = 0.f;
    }
}

void VehiclePhysics::fillVehicle(Vehicle& veh) const
{
    veh.position = pos_;
    veh.heading  = heading_;
    for (int i = 0; i < 4; ++i)
        veh.wheelPos[i] = wheelWorldPos(i);
}

glm::vec3 VehiclePhysics::wheelWorldPos(int i) const
{
    return pos_ + rotateByHeading(wheels_[i].localOffset);
}

glm::vec3 VehiclePhysics::rotateByHeading(const glm::vec3& v) const
{
    float s = std::sin(heading_);
    float c = std::cos(heading_);
    return {
        v.x * c - v.z * s,
        v.y,
        v.x * s + v.z * c
    };
}
