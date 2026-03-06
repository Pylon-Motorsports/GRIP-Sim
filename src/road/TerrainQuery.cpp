#include "TerrainQuery.h"
#include <cmath>

void TerrainQuery::buildFromMesh(const std::vector<glm::vec3>& positions,
                                  const std::vector<uint32_t>& indices)
{
    triangles_.clear();
    triangles_.reserve(indices.size() / 3);
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        triangles_.push_back({
            positions[indices[i]],
            positions[indices[i + 1]],
            positions[indices[i + 2]]
        });
    }
}

float TerrainQuery::heightAt(float x, float z, float hintY) const
{
    if (triangles_.empty()) return 0.f;

    float bestY    = NO_GROUND;
    float bestDist = 1e18f;
    bool  useHint  = (hintY > NO_GROUND + 1.f);

    for (const auto& tri : triangles_) {
        // Project triangle to XZ plane and test if (x,z) is inside using
        // barycentric coordinates.
        float x0 = tri.v0.x, z0 = tri.v0.z;
        float x1 = tri.v1.x, z1 = tri.v1.z;
        float x2 = tri.v2.x, z2 = tri.v2.z;

        float dx0 = x1 - x0, dz0 = z1 - z0;
        float dx1 = x2 - x0, dz1 = z2 - z0;
        float dxp = x  - x0, dzp = z  - z0;

        float det = dx0 * dz1 - dx1 * dz0;
        if (std::abs(det) < 1e-10f) continue;  // degenerate triangle

        float invDet = 1.f / det;
        float u = (dxp * dz1 - dx1 * dzp) * invDet;
        float v = (dx0 * dzp - dxp * dz0) * invDet;

        if (u >= -1e-4f && v >= -1e-4f && (u + v) <= 1.0001f) {
            float y = tri.v0.y + u * (tri.v1.y - tri.v0.y) + v * (tri.v2.y - tri.v0.y);
            if (useHint) {
                // Pick the triangle closest to the hint Y (resolves overlapping road)
                float dist = std::abs(y - hintY);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestY = y;
                }
            } else {
                if (y > bestY) bestY = y;
            }
        }
    }

    return bestY;
}
