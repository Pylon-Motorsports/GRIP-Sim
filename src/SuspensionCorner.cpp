#include "SuspensionCorner.hpp"
#include "SimpleTire.hpp"

SuspensionCorner::SuspensionCorner(std::string name, glm::vec3 attachPt,
                                   glm::vec3 tireOff, bool steered_, bool driven_)
    : VehiclePhysicsComponent(std::move(name), attachPt)
    , tireOffset(tireOff), steered(steered_), driven(driven_) {}

void SuspensionCorner::init() {
    addChild(std::make_unique<SpringDamper>("Spring", glm::vec3(0.f)));
    addChild(std::make_unique<SimpleTire>("Tire", tireOffset, steered, driven));
    VehiclePhysicsComponent::init();
}

SpringDamper* SuspensionCorner::spring() {
    return findChild<SpringDamper>("Spring");
}

Tire* SuspensionCorner::tire() {
    return findChild<Tire>("Tire");
}

ComponentInput SuspensionCorner::makeChildInput(const ComponentInput& parentInput,
                                                 const VehiclePhysicsComponent& child) {
    ComponentInput ci = VehiclePhysicsComponent::makeChildInput(parentInput, child);

    if (child.name() == "Tire") {
        auto* sd = spring();
        if (sd)
            ci.normalLoad = sd->lastOutput().force.y;
    }
    return ci;
}
