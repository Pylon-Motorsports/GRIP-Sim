#pragma once
#include "VehiclePhysicsComponent.hpp"

class Mass : public VehiclePhysicsComponent {
public:
    float massKg    = 0.f;
    glm::vec3 inertia { 0.f };  // Ixx, Iyy, Izz (kg*m^2)

    Mass(float kg, glm::vec3 inertia_);

protected:
    ComponentOutput compute(const ComponentInput& input) override;
};
