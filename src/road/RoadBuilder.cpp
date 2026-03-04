#include "RoadBuilder.h"
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <algorithm>

using namespace grip_schema;

static constexpr float DEG2RAD = glm::pi<float>() / 180.f;
static constexpr glm::vec3 UP  { 0.f, 1.f, 0.f };

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static int tessSteps(float angleDeltaDeg)
{
    float absAngle = std::abs(angleDeltaDeg);
    if (absAngle < 0.5f) return 2;
    return std::max(2, static_cast<int>(std::ceil(absAngle / 5.f)));
}

static float surfaceId(Surface s)
{
    switch (s) {
        case Surface::TarmacSmooth:
        case Surface::TarmacShiny:
        case Surface::TarmacDirty:
        case Surface::TarmacBumpy:    return 0.f;  // dark grey
        case Surface::GravelLoose:
        case Surface::GravelCompact:
        case Surface::GravelMarbles:
        case Surface::GravelDusty:    return 1.f;  // tan
        case Surface::Grass:          return 2.f;  // green-brown
        case Surface::Snow:           return 3.f;  // white-blue
        case Surface::IcePacked:
        case Surface::IceClear:
        case Surface::IceSnowCovered: return 4.f;  // light blue
        case Surface::Rock:           return 5.f;  // grey-brown
        case Surface::Sand:           return 6.f;  // sandy
        default:                      return 0.f;
    }
}

/// Cross-section describes 4 points across the road: outerLeft, roadLeft, roadRight, outerRight.
struct CrossSection {
    float leftVergeM  { 0.8f };  ///< Width of left verge in metres
    float roadHalfM   { 3.5f };  ///< Half-width of driveable road (left of centreline)
    float rightHalfM  { 3.5f };  ///< Half-width of driveable road (right of centreline)
    float rightVergeM { 0.8f };  ///< Width of right verge in metres
    float centreId    { 0.f };
    float leftId      { 1.f };
    float rightId     { 1.f };
};

static CrossSection computeCrossSection(const GripSegment& seg)
{
    CrossSection cs;

    // Left side: strips are ordered from road centre outward.
    // Last strip = outermost verge.
    if (!seg.leftSurfaces.empty()) {
        // First left strip = road surface
        cs.centreId = surfaceId(seg.leftSurfaces.front().surface);
        cs.roadHalfM = seg.leftSurfaces.front().widthCentimeters / 100.f;

        cs.leftVergeM = 0.f;
        cs.leftId = cs.centreId;
        for (size_t i = 1; i < seg.leftSurfaces.size(); ++i) {
            cs.leftVergeM += seg.leftSurfaces[i].widthCentimeters / 100.f;
            cs.leftId = surfaceId(seg.leftSurfaces[i].surface);
        }
    }

    if (!seg.rightSurfaces.empty()) {
        // First right strip = road surface
        cs.centreId  = surfaceId(seg.rightSurfaces.front().surface);
        cs.rightHalfM = seg.rightSurfaces.front().widthCentimeters / 100.f;

        cs.rightVergeM = 0.f;
        cs.rightId = cs.centreId;
        for (size_t i = 1; i < seg.rightSurfaces.size(); ++i) {
            cs.rightVergeM += seg.rightSurfaces[i].widthCentimeters / 100.f;
            cs.rightId = surfaceId(seg.rightSurfaces[i].surface);
        }
    }

    return cs;
}

// Emit 4 vertices at the given spine position and heading.
// Vertices: [outerLeft, roadLeft, roadRight, outerRight]
static void emitRow(RoadMesh& mesh, glm::vec3 pos, float heading, const CrossSection& cs, float u)
{
    float fwd_x = std::sin(heading);
    float fwd_z = std::cos(heading);
    glm::vec3 forward { fwd_x, 0.f, fwd_z };
    glm::vec3 right = glm::cross(forward, UP);

    glm::vec3 roadLeft   = pos - right * cs.roadHalfM;
    glm::vec3 outerLeft  = roadLeft  - right * cs.leftVergeM;
    glm::vec3 roadRight  = pos + right * cs.rightHalfM;
    glm::vec3 outerRight = roadRight + right * cs.rightVergeM;

    mesh.vertices.push_back({ outerLeft,  UP, { u, 0.f  }, cs.leftId   });
    mesh.vertices.push_back({ roadLeft,   UP, { u, 0.33f }, cs.centreId });
    mesh.vertices.push_back({ roadRight,  UP, { u, 0.67f }, cs.centreId });
    mesh.vertices.push_back({ outerRight, UP, { u, 1.f  }, cs.rightId  });
}

// ---------------------------------------------------------------------------
// RoadBuilder::build
// ---------------------------------------------------------------------------

RoadMesh RoadBuilder::build(std::vector<GripSegment>& segments)
{
    RoadMesh mesh;
    mesh.dirty = true;

    glm::vec3 pos     { 0.f, 0.f, 0.f };
    float     heading { 0.f };   // radians; 0 = +Z forward, CCW positive
    float     odoU    { 0.f };   // running U coordinate along road

    static constexpr uint32_t VERTS_PER_ROW = 4;  // outerL, roadL, roadR, outerR

    for (auto& seg : segments) {
        seg.startPosition   = pos;
        seg.startHeadingRad = heading;

        mesh.segmentStartVertex.push_back(static_cast<uint32_t>(mesh.vertices.size()));

        CrossSection cs = computeCrossSection(seg);

        int   steps       = tessSteps(seg.horizontalAngleDeltaDegrees);
        float deltaAngle  = (seg.horizontalAngleDeltaDegrees * DEG2RAD) / static_cast<float>(steps);
        float deltaLength = seg.centerLineLengthMeters / static_cast<float>(steps);

        uint32_t rowBase = static_cast<uint32_t>(mesh.vertices.size());

        for (int step = 0; step <= steps; ++step) {
            emitRow(mesh, pos, heading, cs, odoU);
            mesh.centerlinePoints.push_back(pos);

            if (step < steps) {
                float fwd_x = std::sin(heading);
                float fwd_z = std::cos(heading);
                pos += glm::vec3{ fwd_x, 0.f, fwd_z } * deltaLength;
                heading += deltaAngle;
                odoU    += deltaLength;
            }
        }

        // Build index buffer: 3 quads per step (left verge, road, right verge)
        for (int step = 0; step < steps; ++step) {
            uint32_t row0 = rowBase + step * VERTS_PER_ROW;
            uint32_t row1 = row0 + VERTS_PER_ROW;
            for (uint32_t q = 0; q < 3; ++q) {
                uint32_t tl = row0 + q, tr = row0 + q + 1;
                uint32_t bl = row1 + q, br = row1 + q + 1;
                mesh.indices.insert(mesh.indices.end(), { tl, bl, tr, tr, bl, br });
            }
        }
    }

    return mesh;
}
