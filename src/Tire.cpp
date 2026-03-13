#include "Tire.hpp"
#include <cmath>

Tire::Tire(std::string name, glm::vec3 attachPt, bool steered_, bool driven_)
    : VehiclePhysicsComponent(std::move(name), attachPt)
    , steered(steered_), driven(driven_) {}

Drawable Tire::generateDrawable(const ComponentInput& input) const {
    Drawable d;
    glm::vec3 center = input.bodyRotation * attachmentPoint_ + input.position;
    glm::vec4 col{0.15f, 0.15f, 0.15f, 1.f};
    glm::vec3 up{0.f, 1.f, 0.f};

    const int N = 12;
    d.vertices.push_back({center, up, col});
    for (int i = 0; i < N; ++i) {
        float a = float(i) / float(N) * 6.2832f;
        glm::vec3 rim = center + glm::vec3(radius * std::cos(a), 0.f, radius * std::sin(a));
        d.vertices.push_back({rim, up, col});
    }
    for (int i = 0; i < N; ++i) {
        uint32_t next = 1 + (i + 1) % N;
        d.indices.insert(d.indices.end(), {0u, uint32_t(1 + i), next});
    }
    return d;
}
