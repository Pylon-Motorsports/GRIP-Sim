#include "VehiclePhysicsComponent.hpp"

// ---------------------------------------------------------------------------
// ComponentOutput
// ---------------------------------------------------------------------------
ComponentOutput& ComponentOutput::operator+=(const ComponentOutput& rhs) {
    force  += rhs.force;
    torque += rhs.torque;
    return *this;
}

ComponentOutput operator+(ComponentOutput a, const ComponentOutput& b) {
    a += b;
    return a;
}

// ---------------------------------------------------------------------------
// Drawable
// ---------------------------------------------------------------------------
void Drawable::merge(const Drawable& other) {
    if (other.empty()) return;
    uint32_t base = static_cast<uint32_t>(vertices.size());
    vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
    for (uint32_t idx : other.indices)
        indices.push_back(base + idx);
}

// ---------------------------------------------------------------------------
// VehiclePhysicsComponent
// ---------------------------------------------------------------------------
VehiclePhysicsComponent::VehiclePhysicsComponent(std::string name, glm::vec3 attachmentPoint)
    : name_(std::move(name))
    , attachmentPoint_(attachmentPoint) {}

void VehiclePhysicsComponent::setControls(const VehicleControls* ctrl) {
    controls_ = ctrl;
    for (auto& child : children_) child->setControls(ctrl);
}

void VehiclePhysicsComponent::init() {
    for (auto& child : children_) child->init();
}

const std::vector<std::unique_ptr<VehiclePhysicsComponent>>&
VehiclePhysicsComponent::children() const {
    return children_;
}

ComponentOutput VehiclePhysicsComponent::update(const ComponentInput& input) {
    lastOutput_ = compute(input);

    for (auto& child : children_) {
        ComponentInput childInput = makeChildInput(input, *child);
        ComponentOutput childOut  = child->update(childInput);

        // Moment arm: rotate child's body-local offset to world frame
        // before crossing with the world-space force.
        lastOutput_.torque += glm::cross(input.bodyRotation * child->attachmentPoint(), childOut.force);
        lastOutput_.force  += childOut.force;
        lastOutput_.torque += childOut.torque;
    }

    return lastOutput_;
}

Drawable VehiclePhysicsComponent::collectDrawables(const ComponentInput& input) const {
    Drawable result;
    result.merge(generateDrawable(input));
    for (auto& child : children_) {
        result.merge(child->collectDrawables(input));
    }
    return result;
}

void VehiclePhysicsComponent::reset() {
    lastOutput_ = {};
    for (auto& child : children_) child->reset();
}

void VehiclePhysicsComponent::addChild(std::unique_ptr<VehiclePhysicsComponent> child) {
    children_.push_back(std::move(child));
}

ComponentOutput VehiclePhysicsComponent::compute(const ComponentInput& /*input*/) {
    return {};
}

Drawable VehiclePhysicsComponent::generateDrawable(const ComponentInput& /*input*/) const {
    return {};
}

ComponentInput VehiclePhysicsComponent::makeChildInput(const ComponentInput& parentInput,
                                                        const VehiclePhysicsComponent& child) {
    ComponentInput ci  = parentInput;
    ci.position       += parentInput.bodyRotation * child.attachmentPoint();
    return ci;
}
