#include "TireTrail.hpp"
#include <cmath>
#include <algorithm>

void TireTrail::addPoint(const glm::vec3& wheelPos, float normalLoad,
                          float contactWidth, float slipRatio, float slipAngle,
                          const glm::vec3& wheelRight)
{
    if (normalLoad <= 0.f) return;

    glm::vec3 groundPos = wheelPos;
    groundPos.y = wheelPos.y - 0.30f + 0.025f;  // ~2.5cm above ground to clear terrain Z-fighting

    if (count_ > 0) {
        glm::vec3 last = points_[(head_ + count_ - 1) % MAX_POINTS].pos;
        float dist = glm::length(groundPos - last);
        if (dist < MIN_DIST) return;
    }

    float combinedSlip = std::sqrt(slipRatio * slipRatio + slipAngle * slipAngle);
    float intensity = std::min(combinedSlip * 3.f, 1.f);

    Point pt;
    pt.pos = groundPos;
    pt.right = glm::normalize(glm::vec3{wheelRight.x, 0.f, wheelRight.z});
    pt.halfWidth = contactWidth * 0.5f;
    pt.slipIntensity = intensity;

    int writeIdx = (head_ + count_) % MAX_POINTS;
    if (count_ < MAX_POINTS) {
        points_[writeIdx] = pt;
        count_++;
    } else {
        points_[head_] = pt;
        head_ = (head_ + 1) % MAX_POINTS;
    }
}

void TireTrail::buildGeometry(std::vector<Vertex>& verts, std::vector<uint32_t>& idx) const
{
    if (count_ < 2) return;

    uint32_t baseVert = (uint32_t)verts.size();

    for (int i = 0; i < count_; ++i) {
        const Point& pt = points_[(head_ + i) % MAX_POINTS];

        float alpha = 0.15f + 0.70f * pt.slipIntensity;

        if (pt.slipIntensity < 0.3f) {
            int segIdx = i / 3;
            if (segIdx % 2 == 1) alpha = 0.f;
        }

        glm::vec4 color{0.1f, 0.1f, 0.1f, alpha};
        glm::vec3 normal{0.f, 1.f, 0.f};

        glm::vec3 offset = pt.right * pt.halfWidth;
        verts.push_back({pt.pos - offset, normal, color});
        verts.push_back({pt.pos + offset, normal, color});
    }

    for (int i = 0; i < count_ - 1; ++i) {
        // Skip quad if consecutive points are too far apart (airborne gap)
        const Point& a = points_[(head_ + i) % MAX_POINTS];
        const Point& b = points_[(head_ + i + 1) % MAX_POINTS];
        float dx = b.pos.x - a.pos.x, dz = b.pos.z - a.pos.z;
        if (dx*dx + dz*dz > 1.f) continue;  // >1m gap = was airborne

        uint32_t bl = baseVert + i * 2;
        uint32_t br = bl + 1;
        uint32_t tl = bl + 2;
        uint32_t tr = bl + 3;
        idx.insert(idx.end(), {bl, tl, br, br, tl, tr});
    }
}

void TireTrails::update(const Vehicle& veh)
{
    for (int i = 0; i < 4; ++i) {
        glm::vec3 wheelRight = veh.bodyRotation[0];
        if (i < 2) {
            float cs = std::cos(veh.frontSteerAngle);
            float ss = std::sin(veh.frontSteerAngle);
            glm::vec3 fwd = veh.bodyRotation[2];
            wheelRight = wheelRight * cs + fwd * ss;
        }

        trails[i].addPoint(
            veh.wheelPos[i],
            veh.wheelNormalLoad[i],
            veh.wheelContactWidth[i],
            veh.wheelSlipRatio[i],
            veh.wheelSlipAngle[i],
            wheelRight);
    }
}

void TireTrails::buildGeometry(std::vector<Vertex>& verts, std::vector<uint32_t>& idx) const
{
    for (int i = 0; i < 4; ++i)
        trails[i].buildGeometry(verts, idx);
}
