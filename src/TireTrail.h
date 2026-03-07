#pragma once
#include "Vehicle.h"
#include "Renderer.h"
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

// Per-wheel tire trail: ring buffer of ground contact points.
// Each sample stores position, width, and slip intensity.
// The trail is rendered as a triangle strip ribbon on the ground.

class TireTrail {
public:
    static constexpr int MAX_POINTS = 2000;  // ~16 seconds at 120 Hz
    static constexpr float MIN_DIST = 0.03f; // minimum distance between samples (m)

    struct Point {
        glm::vec3 pos;       // world position (ground contact)
        glm::vec3 right;     // right vector (perpendicular to travel direction)
        float halfWidth;     // half of contact patch width
        float slipIntensity; // 0 = grip, 1 = sliding
    };

    void addPoint(const glm::vec3& wheelPos, float normalLoad,
                  float contactWidth, float slipRatio, float slipAngle,
                  const glm::vec3& wheelRight)
    {
        if (normalLoad <= 0.f) return;  // airborne, no mark

        // Ground contact point: directly below wheel center
        glm::vec3 groundPos = wheelPos;
        groundPos.y = wheelPos.y - 0.30f + 0.008f;  // tire radius offset + lift above ground surface

        // Skip if too close to last point
        if (count_ > 0) {
            glm::vec3 last = points_[(head_ + count_ - 1) % MAX_POINTS].pos;
            float dist = glm::length(groundPos - last);
            if (dist < MIN_DIST) return;
        }

        // Slip intensity: combined longitudinal + lateral
        float combinedSlip = std::sqrt(slipRatio * slipRatio + slipAngle * slipAngle);
        float intensity = std::min(combinedSlip * 3.f, 1.f);  // saturates around 0.33 combined

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

    void clear() { head_ = 0; count_ = 0; }

    // Build triangle-strip geometry as indexed triangles.
    // Low slip = hatched/dashed pattern, high slip = solid.
    void buildGeometry(std::vector<Vertex>& verts, std::vector<uint32_t>& idx) const
    {
        if (count_ < 2) return;

        uint32_t baseVert = (uint32_t)verts.size();

        for (int i = 0; i < count_; ++i) {
            const Point& pt = points_[(head_ + i) % MAX_POINTS];

            // Color: dark rubber marks, opacity based on slip intensity
            // Low slip: faint translucent marks (textured feel via dashing)
            // High slip: solid dark marks
            float alpha = 0.15f + 0.70f * pt.slipIntensity;

            // Dash pattern for low slip: skip every other segment
            if (pt.slipIntensity < 0.3f) {
                // Create a dashed pattern: alternate segments visible/invisible
                int segIdx = i / 3;  // every 3 points = one dash
                if (segIdx % 2 == 1) alpha = 0.f;
            }

            glm::vec4 color{0.1f, 0.1f, 0.1f, alpha};
            glm::vec3 normal{0.f, 1.f, 0.f};

            glm::vec3 offset = pt.right * pt.halfWidth;
            verts.push_back({pt.pos - offset, normal, color});
            verts.push_back({pt.pos + offset, normal, color});
        }

        // Build triangles from the strip
        for (int i = 0; i < count_ - 1; ++i) {
            uint32_t bl = baseVert + i * 2;
            uint32_t br = bl + 1;
            uint32_t tl = bl + 2;
            uint32_t tr = bl + 3;
            idx.insert(idx.end(), {bl, tl, br, br, tl, tr});
        }
    }

private:
    Point points_[MAX_POINTS];
    int head_ = 0;
    int count_ = 0;
};

// Manages trails for all 4 wheels
struct TireTrails {
    TireTrail trails[4];

    void update(const Vehicle& veh)
    {
        for (int i = 0; i < 4; ++i) {
            // Compute wheel right direction from body rotation + steering
            glm::vec3 wheelRight = veh.bodyRotation[0]; // body right
            if (i < 2) {
                // Front wheels: rotate right vector by steer angle about body up
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

    void clear()
    {
        for (auto& t : trails) t.clear();
    }

    void buildGeometry(std::vector<Vertex>& verts, std::vector<uint32_t>& idx) const
    {
        for (int i = 0; i < 4; ++i)
            trails[i].buildGeometry(verts, idx);
    }
};
