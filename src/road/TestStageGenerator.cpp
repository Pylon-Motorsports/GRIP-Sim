#include "TestStageGenerator.h"

using namespace grip_schema;

static GripSegment makeSeg(
    int   index,
    float lengthM,
    float angleDeg,
    Surface surfLeft,
    float   leftWidthCm,
    Surface surfCenter,
    float   centerWidthCm,
    Surface surfRight,
    float   rightWidthCm)
{
    GripSegment seg;
    seg.indexSequence               = index;
    seg.centerLineLengthMeters      = lengthM;
    seg.horizontalAngleDeltaDegrees = angleDeg;

    // Left verge
    seg.leftSurfaces.push_back({ surfLeft,   leftWidthCm });
    // Centre (driveable road)
    seg.leftSurfaces.push_back({ surfCenter, centerWidthCm / 2.f });
    seg.rightSurfaces.push_back({ surfCenter, centerWidthCm / 2.f });
    // Right verge
    seg.rightSurfaces.push_back({ surfRight,  rightWidthCm });

    return seg;
}

std::vector<GripSegment> TestStageGenerator::generate()
{
    //  idx  len   angle   leftVerge       leftWidth   centre       ctrW  rightVerge      rightWidth
    return {
        makeSeg( 0, 80.f,   0.f, Surface::GravelLoose,  80.f, Surface::TarmacSmooth, 700.f, Surface::GravelLoose,   80.f),
        makeSeg( 1, 30.f, -60.f, Surface::GravelLoose,  80.f, Surface::GravelCompact,600.f, Surface::GravelLoose,   80.f),
        makeSeg( 2, 50.f,   0.f, Surface::GravelLoose,  80.f, Surface::TarmacSmooth, 700.f, Surface::GravelLoose,   80.f),
        makeSeg( 3, 20.f, +90.f, Surface::GravelLoose,  80.f, Surface::GravelCompact,600.f, Surface::GravelLoose,   80.f),
        makeSeg( 4, 60.f,   0.f, Surface::GravelLoose,  80.f, Surface::TarmacSmooth, 700.f, Surface::GravelLoose,   80.f),
        makeSeg( 5, 25.f, -45.f, Surface::GravelLoose,  80.f, Surface::GravelCompact,600.f, Surface::GravelLoose,   80.f),
        makeSeg( 6, 15.f, -30.f, Surface::GravelLoose,  80.f, Surface::GravelCompact,600.f, Surface::GravelLoose,   80.f),
        makeSeg( 7, 40.f, +30.f, Surface::GravelLoose,  80.f, Surface::GravelCompact,600.f, Surface::GravelLoose,   80.f),
        makeSeg( 8, 70.f,   0.f, Surface::GravelLoose,  80.f, Surface::TarmacSmooth, 700.f, Surface::GravelLoose,   80.f),
        makeSeg( 9, 20.f,+120.f, Surface::Grass,        100.f,Surface::TarmacDirty,  600.f, Surface::Grass,         100.f),
        makeSeg(10, 45.f,   0.f, Surface::GravelLoose,  80.f, Surface::TarmacSmooth, 700.f, Surface::GravelLoose,   80.f),
        makeSeg(11, 35.f, -75.f, Surface::Grass,        100.f,Surface::TarmacDirty,  600.f, Surface::Grass,         100.f),
    };
}
