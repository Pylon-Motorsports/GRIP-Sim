#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <glm/glm.hpp>

/// GRIP schema types — mirrors stage-geometry.schema.json from Pylon-Motorsports/GRIP.
/// These are pure data; no logic lives here.
namespace grip_schema {

/// Surface type enum — matches surfaceEnum in the schema.
enum class Surface : uint8_t {
    TarmacSmooth,
    TarmacShiny,
    TarmacDirty,
    TarmacBumpy,
    GravelLoose,
    GravelCompact,
    GravelMarbles,
    GravelDusty,
    Sand,
    Snow,
    IcePacked,
    IceClear,
    IceSnowCovered,
    Rock,
    Grass,
    Water,
    Bridge
};

/// A lateral strip of a single surface type across part of the road width.
struct SurfaceStrip {
    Surface surface           { Surface::TarmacSmooth };
    float   widthCentimeters  { 300.f };
    float   angleDeltaDegrees { 0.f };   ///< Cross-slope delta (optional, 0 = flat)
    float   depthCentimeters  { 0.f };   ///< 0 = packed; >0 = loose on top
};

/// Roadside liner types — matches roadsideLinerEnum.
enum class LinerType : uint8_t {
    GuardRail,
    Trees,
    Wall,
    Fence
};

struct RoadsideLiner {
    LinerType type                    { LinerType::GuardRail };
    float     distanceFromRoadCenterM { 4.f };
    float     widthCentimeters        { 20.f };
    float     heightCentimeters       { 80.f };
};

/// Roadside feature types — matches roadsideFeatureEnum.
enum class FeatureType : uint8_t {
    Boulder,
    Tree,
    Post,
    Hole,
    TcStartSign,
    TcEndSign,
    TcCheckInSign,
    StartSign,
    FinishSign,
    FinishWarningSign
};

struct RoadsideFeature {
    FeatureType type                              { FeatureType::Boulder };
    int         distanceFromRoadCenterCentimeters { 200 };
    int         distanceAlongRoadCentimeters      { 0 };  ///< From segment start
    int         widthCentimeters                  { 50 };
    int         heightCentimeters                 { 80 };
};

/// One segment of a rally stage.
/// Matches RallyStageGeometry schema properties.
/// The segment describes a curved road section: the car travels
/// centerLineLengthMeters while yaw changes by horizontalAngleDeltaDegrees.
struct GripSegment {
    // Schema fields
    int   indexSequence               { 0 };
    float centerLineLengthMeters      { 0.f };
    float horizontalAngleDeltaDegrees { 0.f };  ///< Total yaw change over this segment
    float verticalAngleDeltaDegrees   { 0.f };  ///< Total pitch change (positive = uphill)

    std::vector<SurfaceStrip>    leftSurfaces;
    std::vector<SurfaceStrip>    rightSurfaces;
    std::vector<RoadsideLiner>   leftLiners;
    std::vector<RoadsideLiner>   rightLiners;
    std::vector<RoadsideFeature> leftFeatures;
    std::vector<RoadsideFeature> rightFeatures;

    // Derived (populated by RoadBuilder::build())
    glm::vec3 startPosition   { 0.f };
    float     startHeadingRad { 0.f };
    float     startPitchRad   { 0.f };
};

} // namespace grip_schema
