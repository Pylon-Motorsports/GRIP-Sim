#pragma once
#include <vector>
#include <glm/glm.hpp>

/// Ground-height query built from road mesh triangles.
/// Returns actual mesh Y at any (x,z) position, or NO_GROUND if off-mesh.
class TerrainQuery {
public:
    static constexpr float NO_GROUND = -1e6f;

    /// Build from road mesh vertex positions and index buffer.
    void buildFromMesh(const std::vector<glm::vec3>& positions,
                       const std::vector<uint32_t>& indices);

    /// Query ground height at world (x, z). Returns NO_GROUND if off-mesh.
    /// When hintY is provided, picks the triangle closest to that Y (resolves
    /// overlapping road sections like hairpins).
    float heightAt(float x, float z, float hintY = NO_GROUND) const;

    [[nodiscard]] bool empty() const { return triangles_.empty(); }

private:
    struct Triangle {
        glm::vec3 v0, v1, v2;
    };
    std::vector<Triangle> triangles_;
};
