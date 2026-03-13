#pragma once
#include "VehiclePhysicsComponent.hpp"

class AeroSurface : public VehiclePhysicsComponent {
public:
    float dragCoeff     = 0.f;   // 0.5 * Cd * A * rho  (pre-combined)
    float liftCoeff     = 0.f;   // 0.5 * Cl * A * rho  (negative = downforce)

    AeroSurface(std::string name, glm::vec3 attachPt, float drag, float lift);

protected:
    ComponentOutput compute(const ComponentInput& input) override;
};
