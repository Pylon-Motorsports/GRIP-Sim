#pragma once
#include <vector>
#include "GripTypes.h"

/// Module interface for road/stage generation.
/// Swap implementations for authored stages (loaded from file/DB), procedural generation, etc.
class IRoadGenerator {
public:
    virtual ~IRoadGenerator() = default;

    /// Generate and return a sequence of GRIP segments describing the stage.
    /// Called once at startup by the Engine.
    [[nodiscard]] virtual std::vector<grip::GripSegment> generate() = 0;
};
