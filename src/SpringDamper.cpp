#include "SpringDamper.hpp"
#include <cmath>

SpringDamper::SpringDamper(std::string name, glm::vec3 attachPt)
    : VehiclePhysicsComponent(std::move(name), attachPt) {}

void SpringDamper::setCompression(float c, float cVel) {
    compression    = c;
    compressionVel = cVel;
}

ComponentOutput SpringDamper::compute(const ComponentInput& input) {
    float springForce = springRate * compression;
    float dampForce   = dampRate * compressionVel;
    float total       = springForce + dampForce;
    if (total < 0.f) total = 0.f;

    // Force acts perpendicular to the ground surface
    return { .force = input.groundNormal * total };
}

Drawable SpringDamper::generateDrawable(const ComponentInput& input) const {
    Drawable d;
    float length = restLength - compression;
    glm::vec3 base = input.bodyRotation * attachmentPoint_ + input.position;
    glm::vec3 top  = base + glm::vec3(0.f, length, 0.f);
    float r = 0.02f;
    glm::vec4 col{0.6f, 0.6f, 0.65f, 1.f};

    const int N = 6;
    for (int i = 0; i < N; ++i) {
        float a = float(i) / float(N) * 6.2832f;
        float cx = r * std::cos(a), cz = r * std::sin(a);
        glm::vec3 n = glm::normalize(glm::vec3(cx, 0.f, cz));
        d.vertices.push_back({base + glm::vec3(cx, 0.f, cz), n, col});
        d.vertices.push_back({top  + glm::vec3(cx, 0.f, cz), n, col});
    }
    for (int i = 0; i < N; ++i) {
        int next = (i + 1) % N;
        uint32_t b0 = i * 2, b1 = i * 2 + 1;
        uint32_t n0 = next * 2, n1 = next * 2 + 1;
        d.indices.insert(d.indices.end(), {b0, n0, b1, b1, n0, n1});
    }
    return d;
}
