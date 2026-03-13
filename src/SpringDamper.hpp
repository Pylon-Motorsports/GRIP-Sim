#pragma once
#include "VehiclePhysicsComponent.hpp"

class SpringDamper : public VehiclePhysicsComponent {
public:
    float springRate     = 35000.f;   // N/m
    float dampRate       = 3000.f;    // N*s/m
    float restLength     = 0.25f;     // m
    float maxTravel      = 0.10f;     // m (bump)
    float maxRebound     = 0.12f;     // m

    float compression    = 0.f;       // current compression (m)
    float compressionVel = 0.f;       // compression velocity (m/s)

    SpringDamper(std::string name, glm::vec3 attachPt);

    void setCompression(float c, float cVel);

protected:
    ComponentOutput compute(const ComponentInput& input) override;
    Drawable generateDrawable(const ComponentInput& input) const override;
};
