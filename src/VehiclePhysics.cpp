#include "VehiclePhysics.hpp"
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Build the component tree
// ---------------------------------------------------------------------------
void VehiclePhysics::init()
{
    // CG position: center, 0.35m up, 0.05m behind geometric center
    auto body = std::make_unique<Body>(glm::vec3(0.f, 0.35f, -0.05f));

    auto splitter = std::make_unique<AeroSurface>("Splitter",
        glm::vec3(0.f, 0.15f, 1.8f),   0.18f, -0.25f);

    auto rearWing = std::make_unique<AeroSurface>("RearWing",
        glm::vec3(0.f, 0.9f, -1.6f),   0.12f, -0.40f);

    // Front subframe: at front axle centerline, steered
    auto front = std::make_unique<Subframe>("FrontSubframe",
        glm::vec3(0.f, 0.f, 1.35f), true, false);

    // Rear subframe: at rear axle centerline, driven (RWD)
    auto rear = std::make_unique<Subframe>("RearSubframe",
        glm::vec3(0.f, 0.f, -1.25f), false, true);

    body_          = body.get();
    splitter_      = splitter.get();
    rearWing_      = rearWing.get();
    frontSubframe_ = front.get();
    rearSubframe_  = rear.get();

    components_.push_back(std::move(body));
    components_.push_back(std::move(splitter));
    components_.push_back(std::move(rearWing));
    components_.push_back(std::move(front));
    components_.push_back(std::move(rear));

    for (auto& c : components_) {
        c->init();
    }
}

// ---------------------------------------------------------------------------
void VehiclePhysics::reset()
{
    state_ = {};
    drivetrain_.reset();
    for (auto& c : components_) {
        c->reset();
    }
}

// ---------------------------------------------------------------------------
ComponentInput VehiclePhysics::buildRootInput(float dt) const
{
    return {
        .dt           = dt,
        .position     = state_.position,
        .velocity     = state_.velocity,
        .angularVel   = state_.angularVel,
        .bodyRotation = state_.bodyRotation,
        .normalLoad   = 0.f,
        .surfaceGrip  = surfaceGrip_,
        .groundNormal = groundNormal_,
    };
}

// ---------------------------------------------------------------------------
Drawable VehiclePhysics::collectDrawables() const
{
    ComponentInput ci{};
    ci.position     = state_.position;
    ci.velocity     = state_.velocity;
    ci.angularVel   = state_.angularVel;
    ci.bodyRotation = state_.bodyRotation;

    Drawable all;
    for (auto& c : components_) {
        all.merge(c->collectDrawables(ci));
    }
    return all;
}

// ---------------------------------------------------------------------------
void VehiclePhysics::routeControls(const InputState& input)
{
    // --- Steering: front tires only ---
    float steerAngle = input.steer * maxSteerAngle_;
    frontSubframe_->left()->tire()->setSteerAngle(steerAngle);
    frontSubframe_->right()->tire()->setSteerAngle(steerAngle);
    rearSubframe_->left()->tire()->setSteerAngle(0.f);
    rearSubframe_->right()->tire()->setSteerAngle(0.f);

    // --- Drivetrain: rear tires only (RWD) ---
    // Estimate forward speed in body frame
    glm::vec3 bodyVel = glm::transpose(state_.bodyRotation) * state_.velocity;
    float fwdSpeed = bodyVel.z;
    float tireRadius = rearSubframe_->left()->tire()->radius;

    auto driveOut = drivetrain_.update(input.throttle, fwdSpeed, tireRadius);
    rearSubframe_->left()->tire()->setDriveTorque(driveOut.leftTorqueNm);
    rearSubframe_->right()->tire()->setDriveTorque(driveOut.rightTorqueNm);
    frontSubframe_->left()->tire()->setDriveTorque(0.f);
    frontSubframe_->right()->tire()->setDriveTorque(0.f);

    // --- Brakes: all four wheels with bias ---
    auto brakeOut = brakeSystem_.update(input.brake, input.handBrake);
    frontSubframe_->left()->tire()->setBrakeTorque(brakeOut.fl);
    frontSubframe_->right()->tire()->setBrakeTorque(brakeOut.fr);
    rearSubframe_->left()->tire()->setBrakeTorque(brakeOut.rl);
    rearSubframe_->right()->tire()->setBrakeTorque(brakeOut.rr);
}

// ---------------------------------------------------------------------------
void VehiclePhysics::update(float dt, const InputState& input)
{
    // Route pedal/wheel inputs through Drivetrain, BrakeSystem, and steering
    routeControls(input);

    ComponentInput rootInput = buildRootInput(dt);

    // Gather forces from all top-level components.
    // Each component returns force+torque about its own attachment point.
    // We must also add the moment arm torque: cross(attachPt, force).
    ComponentOutput totalOutput;
    for (auto& c : components_) {
        ComponentOutput out = c->update(rootInput);
        totalOutput.force  += out.force;
        totalOutput.torque += out.torque;
        totalOutput.torque += glm::cross(state_.bodyRotation * c->attachmentPoint(), out.force);
    }

    // --- Integration --------------------------------------------------------
    float totalMass = body_ ? body_->totalMassKg() : 1300.f;
    if (totalMass < 1.f) {
        totalMass = 1.f;
    }

    // Linear
    glm::vec3 accel = totalOutput.force / totalMass;
    state_.velocity += accel * dt;
    state_.position += state_.velocity * dt;

    // Angular — work in body frame so inertia tensor axes are correct.
    // World torque → body frame, divide by body-frame inertia, integrate body ω.
    glm::vec3 inertia = body_ ? body_->inertia() : glm::vec3(500.f, 2000.f, 500.f);
    glm::vec3 bodyTorque = glm::transpose(state_.bodyRotation) * totalOutput.torque;
    glm::vec3 angAccel = bodyTorque / inertia;
    state_.angularVel += angAccel * dt;

    // Speed-dependent angular damping:
    // At standstill, tire contact patches strongly resist rotation.
    // At speed, rely on tire lateral forces for natural yaw damping.
    float speed = glm::length(state_.velocity);
    float dampPerStep = 0.999f;
    if (speed < 3.f) {
        float blend = speed / 3.f;
        dampPerStep = 0.95f + blend * (0.999f - 0.95f);
    }
    state_.angularVel *= dampPerStep;

    // Hard cap: no car spins faster than ~1.5 rev/s in any axis
    constexpr float MAX_ANG_VEL = 3.f * 3.14159265f;  // ~540°/s
    float angMag = glm::length(state_.angularVel);
    if (angMag > MAX_ANG_VEL)
        state_.angularVel *= MAX_ANG_VEL / angMag;

    // --- Rotation update via Rodrigues' formula (gimbal-lock-free) ---
    // Integrate body-frame angular velocity directly into the rotation matrix
    // instead of through Euler angles, which break at pitch ≈ ±90°.
    glm::vec3 dTheta = state_.angularVel * dt;
    float angle = glm::length(dTheta);
    if (angle > 1e-8f) {
        float kx = dTheta.x / angle, ky = dTheta.y / angle, kz = dTheta.z / angle;
        float c = std::cos(angle), s = std::sin(angle), t = 1.f - c;
        // Rodrigues: R_delta = I + sin(θ)·K + (1-cos(θ))·K²
        glm::mat3 dR(
            glm::vec3(t*kx*kx + c,      t*kx*ky + kz*s,  t*kx*kz - ky*s),
            glm::vec3(t*kx*ky - kz*s,   t*ky*ky + c,     t*ky*kz + kx*s),
            glm::vec3(t*kx*kz + ky*s,   t*ky*kz - kx*s,  t*kz*kz + c)
        );
        state_.bodyRotation = state_.bodyRotation * dR;

        // Re-orthogonalize (Gram-Schmidt) to prevent drift over time
        glm::vec3 x = state_.bodyRotation[0];
        glm::vec3 y = state_.bodyRotation[1];
        glm::vec3 z = state_.bodyRotation[2];
        x = glm::normalize(x);
        y = glm::normalize(y - glm::dot(y, x) * x);
        z = glm::cross(x, y);
        state_.bodyRotation = glm::mat3(x, y, z);
    }

    // Extract Euler angles from rotation matrix (for display/HUD only)
    // These are derived FROM the matrix, not used to rebuild it.
    state_.pitch   = std::asin(std::clamp(-state_.bodyRotation[2][1], -1.f, 1.f));
    state_.heading = std::atan2(state_.bodyRotation[2][0], state_.bodyRotation[2][2]);
    state_.roll    = std::atan2(state_.bodyRotation[0][1], state_.bodyRotation[1][1]);
}

// ---------------------------------------------------------------------------
void VehiclePhysics::applyCollisions(const std::vector<CollisionContact>& contacts, float dt)
{
    if (body_) {
        body_->applyCollisionResponse(contacts, state_, dt);
    }
}
