#pragma once
#include "VehiclePhysicsComponent.hpp"
#include "Body.hpp"
#include "AeroSurface.hpp"
#include "Subframe.hpp"
#include "Drivetrain.hpp"
#include "BrakeSystem.hpp"
#include <glm/glm.hpp>

struct InputState {
    float steer     = 0.f;   // -1..1
    float throttle  = 0.f;   //  0..1
    float brake     = 0.f;   //  0..1
    float handBrake = 0.f;   //  0..1
};

struct VehicleState {
    glm::vec3 position    { 0.f };
    glm::vec3 velocity    { 0.f };
    glm::vec3 angularVel  { 0.f };
    glm::mat3 bodyRotation{ 1.f };
    float     heading     { 0.f };
    float     pitch       { 0.f };
    float     roll        { 0.f };
};

class VehiclePhysics {
public:
    void init();
    void reset();
    void update(float dt, const InputState& input);

    const VehicleState& state() const { return state_; }
    VehicleState& mutableState() { return state_; }

    void setSurfaceGrip(float g) { surfaceGrip_ = g; }
    void setGroundNormal(glm::vec3 n) { groundNormal_ = n; }
    void applyCollisions(const std::vector<CollisionContact>& contacts, float dt);

    Drawable collectDrawables() const;

    Body*        body()          { return body_; }
    AeroSurface* splitter()      { return splitter_; }
    AeroSurface* rearWing()      { return rearWing_; }
    Subframe*    frontSubframe() { return frontSubframe_; }
    Subframe*    rearSubframe()  { return rearSubframe_; }
    Drivetrain*  drivetrain()    { return &drivetrain_; }
    BrakeSystem* brakeSystem()   { return &brakeSystem_; }

private:
    VehicleState state_;

    std::vector<std::unique_ptr<VehiclePhysicsComponent>> components_;
    Body*        body_          = nullptr;
    AeroSurface* splitter_      = nullptr;
    AeroSurface* rearWing_      = nullptr;
    Subframe*    frontSubframe_ = nullptr;
    Subframe*    rearSubframe_  = nullptr;

    Drivetrain  drivetrain_;
    BrakeSystem brakeSystem_;

    float maxSteerAngle_ = 0.55f;  // ~31 degrees
    float surfaceGrip_   = 1.f;
    glm::vec3 groundNormal_ { 0.f, 1.f, 0.f };

    ComponentInput buildRootInput(float dt) const;
    void routeControls(const InputState& input);
};
