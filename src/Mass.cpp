#include "Mass.hpp"

Mass::Mass(float kg, glm::vec3 inertia_)
    : VehiclePhysicsComponent("Mass")
    , massKg(kg), inertia(inertia_) {}

ComponentOutput Mass::compute(const ComponentInput& /*input*/) {
    return { .force = glm::vec3(0.f, -massKg * 9.81f, 0.f) };
}
