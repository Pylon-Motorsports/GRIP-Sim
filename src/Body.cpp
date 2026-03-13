#include "Body.hpp"
#include <cmath>
#include <algorithm>

void Body::computeInertia()
{
    float w = 2.f * Vehicle::HALF_TRACK;
    float l = Vehicle::FRONT_AXLE + Vehicle::REAR_AXLE;
    float h = 2.f * Vehicle::BODY_HALF_H;
    pitchInertia = massKg * (h * h + l * l) / 12.f;
    rollInertia  = massKg * (h * h + w * w) / 12.f;
    yawInertia   = massKg * (w * w + l * l) / 12.f;
}

BodyState Body::bodyState(float lateralSpeed) const
{
    BodyState bs;
    bs.pos = pos;
    bs.vel = vel;
    bs.heading = heading;
    bs.pitch = pitch;
    bs.roll = roll;
    bs.forwardSpeed = forwardSpeed;
    bs.lateralSpeed = lateralSpeed;
    bs.pitchRate = pitchRate;
    bs.rollRate = rollRate;
    bs.yawRate = yawRate;
    return bs;
}

void Body::applyAero(const glm::vec3& fwd,
                     glm::vec3& bodyForce, glm::vec3& bodyTorque) const
{
    float speed = std::abs(forwardSpeed);
    float drag = aero.dragForce(speed);
    float dragSign = (forwardSpeed > 0.f) ? -1.f : 1.f;
    bodyForce += fwd * (drag * dragSign);

    // Drag acts at center of pressure above CG -> nose-down pitch moment.
    bodyTorque.x -= drag * aero.dragCopHeight;

    float df = aero.downforceTotal(speed);
    bodyForce.y -= df;

    float dfFront = df * aero.frontSplit;
    float dfRear  = df * (1.f - aero.frontSplit);
    bodyTorque.x += -dfFront * Vehicle::FRONT_AXLE + dfRear * Vehicle::REAR_AXLE;
}

void Body::applyCollider(const Terrain& terrain, const BodyState& bs,
                         glm::vec3& bodyForce, glm::vec3& bodyTorque) const
{
    static constexpr float HW = Vehicle::HALF_TRACK + Vehicle::WHEEL_HALF_W;
    static constexpr float FA = Vehicle::FRONT_AXLE;
    static constexpr float RA = Vehicle::REAR_AXLE;
    static constexpr float STIFFNESS = 300000.f;
    static constexpr float DAMPING   = 10000.f;

    static constexpr float BOT_Y = (Vehicle::WHEEL_Y - Vehicle::BODY_Y)
                                   + Vehicle::WHEEL_RADIUS * 0.5f;
    static constexpr float TOP_Y = Vehicle::BODY_HALF_H;

    const glm::vec3 corners[] = {
        {-HW, BOT_Y,  FA}, { HW, BOT_Y,  FA},
        {-HW, BOT_Y, -RA}, { HW, BOT_Y, -RA},
        {-HW, TOP_Y,  FA}, { HW, TOP_Y,  FA},
        {-HW, TOP_Y, -RA}, { HW, TOP_Y, -RA},
    };

    for (auto& local : corners) {
        glm::vec3 world = bs.pos + bs.rotateLocal(local);
        auto tc = terrain.contactAt(world);
        if (tc.penetration > 0.f) {
            float vInto = -glm::dot(vel, tc.normal);
            float f = STIFFNESS * tc.penetration + DAMPING * std::max(vInto, 0.f);
            glm::vec3 force = tc.normal * f;
            bodyForce += force;

            bool isGround = std::abs(tc.normal.y) > 0.5f;
            if (isGround) {
                glm::vec3 leverArm = world - bs.pos;
                bodyTorque += glm::cross(leverArm, force);
            }
        }
    }
}

void Body::integrate(const glm::vec3& bodyForce, const glm::vec3& bodyTorque,
                     float totalMass, float dt)
{
    glm::vec3 accel = bodyForce / totalMass;
    vel += accel * dt;
    pos += vel * dt;

    float pitchAccel = bodyTorque.x / pitchInertia;
    float rollAccel  = bodyTorque.z / rollInertia;
    pitchRate += pitchAccel * dt;
    rollRate  += rollAccel  * dt;

    float pitchDamp = std::min(BUSHING_PITCH_DAMPING / pitchInertia * dt, 1.f);
    float rollDamp  = std::min(BUSHING_ROLL_DAMPING  / rollInertia  * dt, 1.f);
    pitchRate *= (1.f - pitchDamp);
    rollRate  *= (1.f - rollDamp);

    pitch += pitchRate * dt;
    roll  += rollRate  * dt;

    float yawAccel = bodyTorque.y / yawInertia;
    yawRate += yawAccel * dt;
    heading += yawRate * dt;
}

void Body::reset()
{
    pos = {0.f, Vehicle::BODY_Y, 0.f};
    vel = {0.f, 0.f, 0.f};
    heading = 0.f;
    forwardSpeed = 0.f;
    pitch = 0.f;
    roll = 0.f;
    pitchRate = 0.f;
    rollRate = 0.f;
    yawRate = 0.f;
}
