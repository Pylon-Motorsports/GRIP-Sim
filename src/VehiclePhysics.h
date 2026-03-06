#pragma once
#include "Wheel.h"
#include "Suspension.h"
#include "Vehicle.h"
#include <glm/glm.hpp>

class VehiclePhysics {
public:
    void init();
    void update(float dt);

    // Export state for rendering
    void fillVehicle(Vehicle& veh) const;

private:
    static constexpr float GRAVITY  = 9.81f;
    static constexpr float GROUND_Y = 0.f;

    float     bodyMassKg_ = 300.f;
    glm::vec3 pos_{0.f};
    glm::vec3 vel_{0.f};
    float     heading_ = 0.f;

    Wheel      wheels_[4];
    Suspension suspensions_[4];

    glm::vec3 wheelWorldPos(int i) const;
    glm::vec3 rotateByHeading(const glm::vec3& v) const;
};
