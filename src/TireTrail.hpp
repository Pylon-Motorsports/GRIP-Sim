#pragma once
#include "Vehicle.hpp"
#include "Renderer.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

// Per-wheel tire trail: ring buffer of ground contact points.
class TireTrail {
public:
    static constexpr int MAX_POINTS = 2000;
    static constexpr float MIN_DIST = 0.03f;

    struct Point {
        glm::vec3 pos;
        glm::vec3 right;
        float halfWidth;
        float slipIntensity;
    };

    void addPoint(const glm::vec3& wheelPos, float normalLoad,
                  float contactWidth, float slipRatio, float slipAngle,
                  const glm::vec3& wheelRight);

    void clear() { head_ = 0; count_ = 0; }

    void buildGeometry(std::vector<Vertex>& verts, std::vector<uint32_t>& idx) const;

private:
    Point points_[MAX_POINTS];
    int head_ = 0;
    int count_ = 0;
};

// Manages trails for all 4 wheels
struct TireTrails {
    TireTrail trails[4];

    void update(const Vehicle& veh);
    void clear() { for (auto& t : trails) t.clear(); }
    void buildGeometry(std::vector<Vertex>& verts, std::vector<uint32_t>& idx) const;
};
