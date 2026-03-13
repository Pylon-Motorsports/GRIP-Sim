#pragma once
#include "SpringDamper.hpp"
#include "Tire.hpp"

class SuspensionCorner : public VehiclePhysicsComponent {
public:
    glm::vec3 tireOffset { 0.f, -0.30f, 0.f };
    bool steered = false;
    bool driven  = false;

    SuspensionCorner(std::string name, glm::vec3 attachPt, glm::vec3 tireOff,
                     bool steered_, bool driven_);

    void init() override;

    SpringDamper* spring();
    Tire*         tire();

protected:
    ComponentInput makeChildInput(const ComponentInput& parentInput,
                                  const VehiclePhysicsComponent& child) override;
};
