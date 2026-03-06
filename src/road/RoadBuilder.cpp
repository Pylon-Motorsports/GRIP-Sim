#include "RoadBuilder.h"
#include <glm/gtc/constants.hpp>
#include <array>
#include <cmath>
#include <algorithm>

using namespace grip_schema;

static constexpr float DEG2RAD = glm::pi<float>() / 180.f;
static constexpr glm::vec3 UP  { 0.f, 1.f, 0.f };

// Road geometry constants
static constexpr float SHOULDER_DROP   = 0.5f;   ///< Ground drops 0.5m at the road edge
static constexpr float SLOPE_WIDTH     = 1.5f;   ///< Width of slope from road edge to flat ground
static constexpr float GROUND_WIDTH    = 8.f;    ///< Flat ground beyond the slope (metres)
static constexpr float CROWN_HEIGHT    = 0.06f;  ///< Road crown: centre 6cm higher than edges
static constexpr float CAMBER_MAX_DEG  = 5.f;    ///< Max camber in tight corners (degrees)
static constexpr float GROUND_ID       = 8.f;    ///< Surface ID for ground/runoff (grass green)

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static int tessSteps(float horizAngleDeg, float vertAngleDeg = 0.f)
{
    float absAngle = std::max(std::abs(horizAngleDeg), std::abs(vertAngleDeg));
    if (absAngle < 0.5f) return 2;
    return std::max(2, static_cast<int>(std::ceil(absAngle / 5.f)));
}

static float surfaceId(Surface s)
{
    switch (s) {
        case Surface::TarmacSmooth:
        case Surface::TarmacShiny:
        case Surface::TarmacDirty:
        case Surface::TarmacBumpy:    return 0.f;  // white
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

/// Cross-section road geometry derived from a GripSegment.
struct CrossSection {
    float leftVergeM  { 0.8f };  ///< Width of left verge in metres
    float roadHalfM   { 3.5f };  ///< Half-width of driveable road left of centreline
    float rightHalfM  { 3.5f };  ///< Half-width of driveable road right of centreline
    float rightVergeM { 0.8f };  ///< Width of right verge in metres
    float centreId    { 0.f };
    float leftId      { 1.f };
    float rightId     { 1.f };
    float camberRad   { 0.f };   ///< Road cross-slope (positive = left higher than right)
};

static CrossSection computeCrossSection(const GripSegment& seg)
{
    CrossSection cs;

    // Left side: strips are ordered from road centre outward.
    if (!seg.leftSurfaces.empty()) {
        cs.centreId  = surfaceId(seg.leftSurfaces.front().surface);
        cs.roadHalfM = seg.leftSurfaces.front().widthCentimeters / 100.f;
        cs.leftVergeM = 0.f;
        cs.leftId     = cs.centreId;
        for (size_t i = 1; i < seg.leftSurfaces.size(); ++i) {
            cs.leftVergeM += seg.leftSurfaces[i].widthCentimeters / 100.f;
            cs.leftId = surfaceId(seg.leftSurfaces[i].surface);
        }
    }

    if (!seg.rightSurfaces.empty()) {
        cs.centreId   = surfaceId(seg.rightSurfaces.front().surface);
        cs.rightHalfM = seg.rightSurfaces.front().widthCentimeters / 100.f;
        cs.rightVergeM = 0.f;
        cs.rightId     = cs.centreId;
        for (size_t i = 1; i < seg.rightSurfaces.size(); ++i) {
            cs.rightVergeM += seg.rightSurfaces[i].widthCentimeters / 100.f;
            cs.rightId = surfaceId(seg.rightSurfaces[i].surface);
        }
    }

    // Camber: proportional to corner severity. Positive angle = right turn = road tilts
    // right (left side higher) so the inside of the turn is lower. This helps cornering.
    float absAngle = std::abs(seg.horizontalAngleDeltaDegrees);
    float camberDeg = std::min(absAngle / 20.f, 1.f) * CAMBER_MAX_DEG;
    // Sign: positive horizontal angle = right turn → positive camber (left higher)
    if (seg.horizontalAngleDeltaDegrees > 0.f)
        cs.camberRad = camberDeg * DEG2RAD;
    else if (seg.horizontalAngleDeltaDegrees < 0.f)
        cs.camberRad = -camberDeg * DEG2RAD;

    return cs;
}

// ---------------------------------------------------------------------------
// emitRow
//
// Emits 7 vertices for one cross-section slice:
//
//   [0]  leftGroundOuter  – flat ground, far left       (UP, GROUND_ID)
//   [1]  leftSlopeBottom  – bottom of slope from road   (UP, GROUND_ID)
//   [2]  roadLeft         – left road edge              (roadNorm, centreId)
//   [3]  roadCenter       – road centreline (crown)     (roadNorm, centreId)
//   [4]  roadRight        – right road edge             (roadNorm, centreId)
//   [5]  rightSlopeBottom – bottom of slope from road   (UP, GROUND_ID)
//   [6]  rightGroundOuter – flat ground, far right      (UP, GROUND_ID)
//
// 6 quads: 0-1, 1-2, 2-3, 3-4, 4-5, 5-6.
// ---------------------------------------------------------------------------

static constexpr uint32_t VERTS_PER_ROW = 7;

static constexpr std::array<uint32_t, 6> QUAD_STARTS { 0, 1, 2, 3, 4, 5 };

static void emitRow(RoadMesh& mesh, glm::vec3 pos, float heading, float pitch,
                    const CrossSection& cs, float u)
{
    const float fwd_x = std::sin(heading);
    const float fwd_z = std::cos(heading);
    const glm::vec3 fwd   { fwd_x, 0.f, fwd_z };
    const glm::vec3 right  = glm::cross(fwd, UP);

    // Road surface normal tilted by pitch
    const float sinP = std::sin(pitch);
    const float cosP = std::cos(pitch);
    const glm::vec3 roadNormal { -sinP * fwd_x, cosP, -sinP * fwd_z };

    // Camber + crown
    const float camber = cs.camberRad;
    const float totalLeftW  = cs.roadHalfM + cs.leftVergeM;
    const float totalRightW = cs.rightHalfM + cs.rightVergeM;

    float leftEdgeY  = totalLeftW  * std::sin(camber) - CROWN_HEIGHT;
    float rightEdgeY = -totalRightW * std::sin(camber) - CROWN_HEIGHT;

    // Road positions
    const glm::vec3 roadCenter = pos;
    const glm::vec3 roadL      = pos - right * totalLeftW  + UP * leftEdgeY;
    const glm::vec3 roadR      = pos + right * totalRightW + UP * rightEdgeY;

    // Ground: slopes down from road edge, then continues flat
    const float groundDropL = leftEdgeY  - SHOULDER_DROP;
    const float groundDropR = rightEdgeY - SHOULDER_DROP;

    const glm::vec3 slopeBottomL  = roadL  - right * SLOPE_WIDTH + UP * (groundDropL - leftEdgeY);
    const glm::vec3 groundOuterL  = slopeBottomL - right * GROUND_WIDTH;
    const glm::vec3 slopeBottomR  = roadR  + right * SLOPE_WIDTH + UP * (groundDropR - rightEdgeY);
    const glm::vec3 groundOuterR  = slopeBottomR + right * GROUND_WIDTH;

    // [0] leftGroundOuter
    mesh.vertices.push_back({ groundOuterL,  UP,         { u, 0.00f }, GROUND_ID   });
    // [1] leftSlopeBottom
    mesh.vertices.push_back({ slopeBottomL,  UP,         { u, 0.15f }, GROUND_ID   });
    // [2] roadLeft
    mesh.vertices.push_back({ roadL,         roadNormal, { u, 0.25f }, cs.centreId });
    // [3] roadCenter (crown apex)
    mesh.vertices.push_back({ roadCenter,    roadNormal, { u, 0.50f }, cs.centreId });
    // [4] roadRight
    mesh.vertices.push_back({ roadR,         roadNormal, { u, 0.75f }, cs.centreId });
    // [5] rightSlopeBottom
    mesh.vertices.push_back({ slopeBottomR,  UP,         { u, 0.85f }, GROUND_ID   });
    // [6] rightGroundOuter
    mesh.vertices.push_back({ groundOuterR,  UP,         { u, 1.00f }, GROUND_ID   });
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
    float     pitch   { 0.f };   // radians; positive = uphill
    float     odoU    { 0.f };   // running U coordinate along road

    uint32_t prevLastRow = 0;    // first vertex of previous segment's last row
    bool     hasPrev     = false;

    for (auto& seg : segments) {
        seg.startPosition   = pos;
        seg.startHeadingRad = heading;
        seg.startPitchRad   = pitch;

        mesh.segmentStartVertex.push_back(static_cast<uint32_t>(mesh.vertices.size()));

        CrossSection cs = computeCrossSection(seg);

        int   steps       = tessSteps(seg.horizontalAngleDeltaDegrees,
                                      seg.verticalAngleDeltaDegrees);
        float deltaYaw    = (seg.horizontalAngleDeltaDegrees * DEG2RAD) / static_cast<float>(steps);
        float deltaPitch  = (seg.verticalAngleDeltaDegrees   * DEG2RAD) / static_cast<float>(steps);
        float deltaLength = seg.centerLineLengthMeters / static_cast<float>(steps);

        uint32_t rowBase = static_cast<uint32_t>(mesh.vertices.size());

        for (int step = 0; step <= steps; ++step) {
            emitRow(mesh, pos, heading, pitch, cs, odoU);
            mesh.centerlinePoints.push_back(pos);
            mesh.centerlineHeadings.push_back(heading);

            if (step < steps) {
                const float cosP  = std::cos(pitch);
                const float fwd_x = std::sin(heading) * cosP;
                const float fwd_y = std::sin(pitch);
                const float fwd_z = std::cos(heading) * cosP;
                pos     += glm::vec3{ fwd_x, fwd_y, fwd_z } * deltaLength;
                heading += deltaYaw;
                pitch   += deltaPitch;
                odoU    += deltaLength;
            }
        }

        // Stitch to previous segment: quads from prev last row → this first row.
        if (hasPrev) {
            for (uint32_t q : QUAD_STARTS) {
                const uint32_t tl = prevLastRow + q, tr = prevLastRow + q + 1;
                const uint32_t bl = rowBase + q,     br = rowBase + q + 1;
                mesh.indices.insert(mesh.indices.end(), { tl, bl, tr, tr, bl, br });
            }
        }

        // Build index buffer for all non-degenerate quads per tessellation step.
        for (int step = 0; step < steps; ++step) {
            const uint32_t row0 = rowBase + static_cast<uint32_t>(step) * VERTS_PER_ROW;
            const uint32_t row1 = row0 + VERTS_PER_ROW;
            for (uint32_t q : QUAD_STARTS) {
                const uint32_t tl = row0 + q, tr = row0 + q + 1;
                const uint32_t bl = row1 + q, br = row1 + q + 1;
                mesh.indices.insert(mesh.indices.end(), { tl, bl, tr, tr, bl, br });
            }
        }

        prevLastRow = rowBase + static_cast<uint32_t>(steps) * VERTS_PER_ROW;
        hasPrev = true;
    }

    // -----------------------------------------------------------------------
    // Tree placement from RoadsideLiner data
    // -----------------------------------------------------------------------
    {
        constexpr float TREE_SPACING = 1.5f;   // metres between tree trunks
        constexpr float TRUNK_RADIUS = 0.20f;  // collision cylinder radius
        constexpr float DEFAULT_HEIGHT = 6.f;   // metres

        // Walk each segment and place trees for liners with LinerType::Trees
        size_t clIdx = 0;
        for (const auto& seg : segments) {
            int steps = tessSteps(seg.horizontalAngleDeltaDegrees,
                                  seg.verticalAngleDeltaDegrees);
            size_t segStart = clIdx;
            size_t segEnd   = clIdx + static_cast<size_t>(steps);
            if (segEnd >= mesh.centerlinePoints.size())
                segEnd = mesh.centerlinePoints.size() - 1;

            auto placeTrees = [&](const std::vector<RoadsideLiner>& liners, float sideSign) {
                for (const auto& liner : liners) {
                    if (liner.type != LinerType::Trees) continue;
                    float dist   = liner.distanceFromRoadCenterM;
                    float height = liner.heightCentimeters / 100.f;
                    if (height < 0.5f) height = DEFAULT_HEIGHT;

                    float odo      = 0.f;
                    float nextTree = 0.f;
                    for (size_t i = segStart; i <= segEnd; ++i) {
                        float stepLen = 0.f;
                        if (i > segStart) {
                            glm::vec3 d = mesh.centerlinePoints[i] - mesh.centerlinePoints[i - 1];
                            stepLen = glm::length(d);
                            odo += stepLen;
                        }
                        // Place trees at regular intervals along this segment
                        while (nextTree <= odo) {
                            // Lerp between previous and current centerline point
                            float t = (stepLen > 0.01f)
                                    ? 1.f - (odo - nextTree) / stepLen
                                    : 1.f;
                            t = std::clamp(t, 0.f, 1.f);
                            size_t prev = (i > segStart) ? i - 1 : i;
                            glm::vec3 p   = glm::mix(mesh.centerlinePoints[prev],
                                                      mesh.centerlinePoints[i], t);
                            float     hdg = glm::mix(mesh.centerlineHeadings[prev],
                                                      mesh.centerlineHeadings[i], t);

                            glm::vec3 rightDir { std::cos(hdg), 0.f, -std::sin(hdg) };
                            glm::vec3 treePos = p + rightDir * (sideSign * dist);
                            treePos.y = p.y - SHOULDER_DROP;
                            mesh.trees.push_back({ treePos, TRUNK_RADIUS, height });
                            nextTree += TREE_SPACING;
                        }
                    }
                }
            };

            placeTrees(seg.leftLiners,  -1.f);
            placeTrees(seg.rightLiners, +1.f);

            clIdx = segEnd;
        }
    }

    return mesh;
}
