#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

/// Per-vertex data for vehicle debug geometry (body box + wheel cylinders).
struct VehicleVertex {
    glm::vec3 pos;    ///<  12 bytes
    glm::vec3 normal; ///<  12 bytes
    glm::vec4 color;  ///<  16 bytes  (rgba — alpha controls opacity)
};
static_assert(sizeof(VehicleVertex) == 40, "VehicleVertex must be 40 bytes");

struct VehicleMesh {
    std::vector<VehicleVertex> vertices;
    std::vector<uint32_t>      indices;
};

namespace vehicle_geom {

/// Axis-aligned box centred at origin.
/// y_bot/y_top allow a non-symmetric Y extent (CG is not at the visual centre).
VehicleMesh makeBox(float half_x, float y_bot, float y_top, float half_z_rear, float half_z_front,
                    glm::vec4 color);

/// Cylinder with its axis along local +X (wheel axle direction).
/// radius = wheel radius, halfLen = half tyre width.
VehicleMesh makeCylinder(float radius, float halfLen, int segs, glm::vec4 color);

/// Thin beam (rectangular cross-section) along local +Y, centred at origin.
/// Used for lower control arm visualization.
VehicleMesh makeBeam(float halfLength, float halfWidth, float halfHeight, glm::vec4 color);

/// Strut (thin cylinder) along local +Y, centred at origin.
/// Used for spring/damper strut visualization.
VehicleMesh makeStrut(float radius, float halfLength, int segs, glm::vec4 color);

/// Batch all trees into one mesh (brown trunks + green canopies).
/// Each tree = 6-sided trunk cylinder + 6-sided canopy cone.
VehicleMesh makeTreeBatch(const std::vector<glm::vec3>& positions,
                          const std::vector<float>& heights,
                          const std::vector<float>& radii);

} // namespace vehicle_geom
