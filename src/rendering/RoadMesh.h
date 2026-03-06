#pragma once
#include <vector>
#include <cstdint>
#include <glm/glm.hpp>

/// Per-vertex data for the road mesh.
/// Layout must match the vertex input description in VulkanPipeline (36 bytes, no padding).
struct RoadVertex {
    glm::vec3 position;   ///< World space
    glm::vec3 normal;     ///< World space surface normal
    glm::vec2 uv;         ///< (u along road, v across road 0..1)
    float     surfaceId;  ///< Maps to Surface enum; used by fragment shader for color lookup
};

static_assert(sizeof(RoadVertex) == 36, "RoadVertex must be 36 bytes");

/// A single tree placed along the road (generated from RoadsideLiner).
struct TreeInstance {
    glm::vec3 position;     ///< Base of trunk (world space, on ground)
    float     trunkRadius;  ///< Collision cylinder radius (m)
    float     height;       ///< Total visual height (m)
};

/// CPU-side road geometry ready for upload to the GPU.
struct RoadMesh {
    std::vector<RoadVertex> vertices;
    std::vector<uint32_t>   indices;

    /// segmentStartVertex[i] = first vertex index belonging to segment i.
    /// Used to map VehicleState::segmentIndex back to geometry.
    std::vector<uint32_t>   segmentStartVertex;

    /// World-space centerline spine points (one per tessellation step).
    /// Used by Camera, IPaceNoteGenerator, and segment tracking.
    std::vector<glm::vec3>  centerlinePoints;

    /// Yaw (rad) at each centerline point (parallel to centerlinePoints).
    /// Used by TerrainQuery for ground height lookups.
    std::vector<float>      centerlineHeadings;

    /// Tree instances placed along the road (from RoadsideLiner with Trees type).
    std::vector<TreeInstance> trees;

    /// Set to true when vertices/indices have changed and need re-uploading.
    bool dirty { true };
};
