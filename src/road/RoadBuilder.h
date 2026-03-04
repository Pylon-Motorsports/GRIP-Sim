#pragma once
#include "GripTypes.h"
#include "rendering/RoadMesh.h"
#include <vector>

/// Converts a sequence of GripSegments into a RoadMesh suitable for GPU upload.
/// Uses a Frenet-frame walk: for each segment, tessellates the curved spine into
/// a series of cross-section quads (left verge | road | right verge).
///
/// Also populates GripSegment::startPosition and startHeadingRad on the input segments
/// so that other systems (vehicle tracking, pacenotes) can use world-space geometry.
class RoadBuilder {
public:
    /// Build the road mesh from segments. Mutates segments to fill derived fields.
    /// Returns the built mesh (dirty = true, ready for upload).
    [[nodiscard]] static RoadMesh build(std::vector<grip_schema::GripSegment>& segments);

};
