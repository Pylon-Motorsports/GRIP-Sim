#include "TestStageGenerator.h"
#include <algorithm>

using namespace grip_schema;

/// Minimum segment length (metres) to prevent tiny steps in elevation.
static constexpr float MIN_SEGMENT_LENGTH = 3.f;

/// Tree liner distance from road centre (metres). Placed on flat ground beyond slope.
static constexpr float TREE_DIST_FROM_CENTER = 12.f;

/// Tree height (cm) for liner.
static constexpr float TREE_HEIGHT_CM = 600.f;

static GripSegment makeSeg(
    int   index,
    float lengthM,
    float horizAngleDeg,
    float vertAngleDeg,
    Surface surfLeft,
    float   leftWidthCm,
    Surface surfCenter,
    float   centerWidthCm,
    Surface surfRight,
    float   rightWidthCm,
    bool    addTrees = true)
{
    // Enforce minimum segment length
    lengthM = std::max(lengthM, MIN_SEGMENT_LENGTH);

    GripSegment seg;
    seg.indexSequence               = index;
    seg.centerLineLengthMeters      = lengthM;
    seg.horizontalAngleDeltaDegrees = horizAngleDeg;
    seg.verticalAngleDeltaDegrees   = vertAngleDeg;

    // Left verge
    seg.leftSurfaces.push_back({ surfLeft,   leftWidthCm });
    // Centre (driveable road — split evenly left/right of centreline)
    seg.leftSurfaces.push_back({ surfCenter, centerWidthCm / 2.f });
    seg.rightSurfaces.push_back({ surfCenter, centerWidthCm / 2.f });
    // Right verge
    seg.rightSurfaces.push_back({ surfRight,  rightWidthCm });

    // Add tree liners on both sides (dense forest lining the road)
    if (addTrees) {
        seg.leftLiners.push_back({
            LinerType::Trees, TREE_DIST_FROM_CENTER, 30.f, TREE_HEIGHT_CM });
        seg.rightLiners.push_back({
            LinerType::Trees, TREE_DIST_FROM_CENTER, 30.f, TREE_HEIGHT_CM });
    }

    return seg;
}

std::vector<GripSegment> TestStageGenerator::generate()
{
    //         idx  len    hAngle vAngle  leftVerge              lW    centre               cW    rightVerge             rW
    return {
        // --- Pre-start pad (flat, behind start line) ---
        makeSeg(-1, 20.f,   0.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Opening straight ---
        makeSeg( 0, 80.f,   0.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Left 3: gentle approach, sharp apex, gentle exit ---
        makeSeg( 1, 20.f, -15.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg( 2, 30.f, -45.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg( 3, 15.f, -10.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Medium straight: gradual uphill climb ---
        makeSeg( 4, 50.f,   0.f, 8.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Right 1: very tight, short (level on the hill) ---
        makeSeg( 5, 10.f, +20.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg( 6, 20.f, +70.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg( 7, 10.f, +15.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Descent from crest ---
        makeSeg( 8, 60.f,   0.f,-10.f,Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- S-bend: Left 4 tightening to Left 5, then Right 5 ---
        makeSeg( 9, 25.f, -45.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg(10, 15.f, -30.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg(11, 10.f,  +5.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg(12, 40.f, +30.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Long straight into hairpin: gradual climb ---
        makeSeg(13, 70.f,   0.f, 5.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Right Hairpin ---
        makeSeg(14, 15.f, +25.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg(15, 20.f, +95.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg(16, 15.f, +20.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Flowing straight ---
        makeSeg(17, 45.f,   0.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Left 2: wide, fast ---
        makeSeg(18, 15.f, -20.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg(19, 35.f, -55.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),

        // --- Finish straight with sharp crest for airborne testing ---
        makeSeg(20, 60.f,   0.f, 0.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
        makeSeg(21, 15.f,   0.f,18.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),  // steep ramp up
        makeSeg(22, 15.f,   0.f,-18.f,Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),  // steep descent — airborne!

        // --- Gentle descent to finish ---
        makeSeg(23, 60.f,   0.f,-3.f, Surface::Grass,         150.f, Surface::TarmacSmooth,900.f, Surface::Grass,        150.f),
    };
}
