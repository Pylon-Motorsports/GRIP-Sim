#include "Subframe.hpp"

Subframe::Subframe(std::string name, glm::vec3 attachPt, bool steered_, bool driven_)
    : VehiclePhysicsComponent(std::move(name), attachPt)
    , steered(steered_), driven(driven_) {}

void Subframe::init() {
    glm::vec3 tireOff = { 0.f, -tireDropM, 0.f };

    addChild(std::make_unique<SuspensionCorner>(
        "Left",
        glm::vec3(-halfTrack, rideHeight, 0.f),
        tireOff, steered, driven));

    addChild(std::make_unique<SuspensionCorner>(
        "Right",
        glm::vec3( halfTrack, rideHeight, 0.f),
        tireOff, steered, driven));

    VehiclePhysicsComponent::init();
}

SuspensionCorner* Subframe::left() {
    return findChild<SuspensionCorner>("Left");
}

SuspensionCorner* Subframe::right() {
    return findChild<SuspensionCorner>("Right");
}
